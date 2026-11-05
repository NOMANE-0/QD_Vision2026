// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_solver_node.hpp"
#include "armor_solver/armor_measurement_gate.hpp"

#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace qd::auto_aim {
namespace {
    cv::Scalar makeDrawColor(int blue, int green, int red, bool is_rgb) {
        if (is_rgb) {
            return cv::Scalar(red, green, blue);
        }
        return cv::Scalar(blue, green, red);
    }

    void getCameraParams(
        const sensor_msgs::msg::CameraInfo& camera_info,
        cv::Mat& camera_matrix,
        cv::Mat& distortion_coefficients
    ) {
        camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info.k.data())).clone();

        if (camera_info.d.empty()) {
            distortion_coefficients = cv::Mat();
            return;
        }

        distortion_coefficients = cv::Mat(
                                      1,
                                      static_cast<int>(camera_info.d.size()),
                                      CV_64F,
                                      const_cast<double*>(camera_info.d.data())
        )
                                      .clone();
    }
} // namespace

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options):
    Node("armor_solver", options),
    solver_(nullptr),
    solveYawPnP_(nullptr),
    state_machine_(qd::utils::VisionStateMachine::getInstance()) {
    // Register logger
    FYT_REGISTER_LOGGER("armor_solver", "qd2026-log", INFO);
    FYT_INFO("armor_solver", "Starting ArmorSolverNode!");

    debug_mode_ = this->declare_parameter("debug", true);
    enable_ekf_ = this->declare_parameter("enable_ekf", true);
    yaw_command_filter_ = YawCommandFilter(
        this->declare_parameter("solver.yaw_filter_alpha", 0.25),
        this->declare_parameter("solver.yaw_filter_deadband_deg", 0.15)
    );
    yaw_filter_fire_tolerance_deg_ =
        this->declare_parameter("solver.yaw_filter_fire_tolerance_deg", 0.5);

    // Tracker 初始化参数
    double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
    double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
    tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
    tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
    lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);
    filter_reset_dt_thres_ = this->declare_parameter("tracker.filter_reset_dt_thres", 1.0);

    // tf2 relevant
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    target_frame_ = this->declare_parameter("target_frame", "odom");
    gimble_frame_ = this->declare_parameter("gimble_frame", "gimbal_link");

    // Measurement publisher (for debug usage)
    measure_pub_ = this->create_publisher<rm_interfaces::msg::Measurement>(
        "armor_solver/measurement",
        rclcpp::SensorDataQoS()
    );

    // Publisher
    armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
        "armor_solver/armors",
        rclcpp::SensorDataQoS()
    );
    target_pub_ = this->create_publisher<rm_interfaces::msg::Target>(
        "armor_solver/target",
        rclcpp::SensorDataQoS()
    );
    gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
        "armor_solver/cmd_gimbal",
        rclcpp::SensorDataQoS()
    );
    result_img_pub_ = image_transport::create_publisher(this, "armor_solver/result_img");
    result_image_publish_thread_ = std::thread(&ArmorSolverNode::resultImagePublishLoop, this);

    // Subscriber
    serial_receive_data_sub_ = this->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "serial/receive",
        rclcpp::SensorDataQoS(),
        std::bind(&ArmorSolverNode::serialCallback, this, std::placeholders::_1)
    );

    // Timer 250 Hz
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(4),
        std::bind(&ArmorSolverNode::timerCallback, this)
    );
    armor_target_.header.frame_id = "";

    // Enable/Disable Armor Solver
    enable_ = true;
    set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
        "armor_solver/set_mode",
        std::bind(
            &ArmorSolverNode::setModeCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    initMarkers();

    // Heartbeat
    heartbeat_ = HeartBeatPublisher::create(this);

    //平移运动模型ekf
    if (enable_ekf_) {
        init_ArmorStateEKF();
    } else {
        FYT_INFO("armor_solver", "EKF disabled: publishing PnP poses and images only");
    }

    enable_YawPnP = this->declare_parameter("enable_solveYawPnp", false);
    max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);
    armor_frame_worker_thread_ = std::thread(&ArmorSolverNode::armorFrameWorkerLoop, this);
}

ArmorSolverNode::~ArmorSolverNode() {
    should_stop_armor_frame_worker_ = true;
    // 唤醒可能阻塞在 waitAndPopLatestArmorFrame 的工作线程，
    // 避免析构时多等一个 50ms 超时窗口。
    state_machine_.notifyArmorFrameWaiters();

    if (armor_frame_worker_thread_.joinable()) {
        armor_frame_worker_thread_.join();
    }

    should_stop_result_image_publish_ = true;
    result_image_cv_.notify_one();

    if (result_image_publish_thread_.joinable()) {
        result_image_publish_thread_.join();
    }
}

/**
 * @brief 定时回调，保证发给电控的消息连贯
 * 
 */
void ArmorSolverNode::timerCallback() {
    enable_ = state_machine_.shouldDetect();
    if (!enable_ || !enable_ekf_) {
        std::lock_guard<std::mutex> lock(solver_state_mutex_);
        yaw_command_filter_.reset();
        if (solver_) {
            solver_->reset_top_level();
        }
        return;
    }

    // Init message
    rm_interfaces::msg::GimbalCmd control_msg;
    // 无目标指令也使用当前 ROS 时间，保证图表与串口调试消息具有有效时间戳。
    control_msg.header.stamp = this->now();
    control_msg.header.frame_id = target_frame_;
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    control_msg.id = "0";

    std::lock_guard<std::mutex> lock(solver_state_mutex_);
    if (solver_ == nullptr) {
        return;
    }

    // If target never detected
    if (armor_target_.header.frame_id.empty()) {
        yaw_command_filter_.reset();
        solver_->reset_top_level();
        gimbal_pub_->publish(std::make_unique<rm_interfaces::msg::GimbalCmd>(std::move(control_msg))
        );
        return;
    }

    if (armor_target_.tracking) {
        try {
            rclcpp::Time solve_start_time = this->now();
            FYT_DEBUG(
                "armor_solver",
                "Latency(img to predict): {:.2f} ms",
                (solve_start_time - armor_target_.header.stamp).seconds() * 1e3
            );
            control_msg = solver_->solve(armor_target_, solve_start_time, tf2_buffer_);
            // 延迟计算
            FYT_DEBUG(
                "armor_solver",
                "Latency(armor solve): {:.2f} ms",
                (this->now() - solve_start_time).seconds() * 1e3
            );

            control_msg.header.stamp = solve_start_time;

            if (std::isfinite(control_msg.yaw) && std::isfinite(control_msg.pitch)
                && std::isfinite(control_msg.distance) && control_msg.distance > 0.0)
            {
                const double raw_yaw_deg = control_msg.yaw;
                control_msg.yaw = yaw_command_filter_.update(
                    raw_yaw_deg, rclcpp::Time(armor_target_.header.stamp).nanoseconds(),
                    armor_target_.id
                );
                if (std::abs(std::remainder(raw_yaw_deg - control_msg.yaw, 360.0))
                    > yaw_filter_fire_tolerance_deg_)
                {
                    control_msg.fire_advice = false;
                }
            } else {
                yaw_command_filter_.reset();
                control_msg.id = "0";
                control_msg.distance = -1.0;
                control_msg.fire_advice = false;
                control_msg.yaw = 0.0;
                control_msg.pitch = 0.0;
            }

        } catch (...) {
            yaw_command_filter_.reset();
            solver_->reset_top_level();
            FYT_ERROR("armor_solver", "Something went wrong in solver!");
        }
    } else {
        yaw_command_filter_.reset();
        solver_->reset_top_level();
    }

    publishMarkers(armor_target_, control_msg);

    gimbal_pub_->publish(std::make_unique<rm_interfaces::msg::GimbalCmd>(std::move(control_msg)));
}

/**
 * @brief 初始化 marker 类型
 * 
 */
void ArmorSolverNode::initMarkers() noexcept {
    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 0.5;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 0.5;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 0.5;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armors_marker_.ns = "filtered_armors";
    armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armors_marker_.scale.x = 0.03;
    armors_marker_.scale.z = 0.125;
    armors_marker_.color.a = 0.5;
    armors_marker_.color.b = 1.0;
    selection_marker_.ns = "selection";
    selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
    selection_marker_.color.a = 0.5;
    selection_marker_.color.g = 1.0;
    selection_marker_.color.r = 1.0;
    trajectory_marker_.ns = "trajectory";
    trajectory_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    trajectory_marker_.scale.x = trajectory_marker_.scale.y = trajectory_marker_.scale.z = 0.0173;
    trajectory_marker_.color.a = 0.5;
    trajectory_marker_.color.r = 1.0;
    trajectory_marker_.color.g = 0.0;
    trajectory_marker_.color.b = 0.0;
    trajectory_marker_.points.clear();
    armor_position_marker_.ns = "armor_position";
    armor_position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    armor_position_marker_.scale.x = armor_position_marker_.scale.y =
        armor_position_marker_.scale.z = 0.1;
    armor_position_marker_.color.a = 0.5;
    armor_position_marker_.color.g = 1.0;
    armor_linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    armor_linear_v_marker_.ns = "armor_linear_v_";
    armor_linear_v_marker_.scale.x = 0.03;
    armor_linear_v_marker_.scale.y = 0.05;
    armor_linear_v_marker_.color.a = 0.5;
    armor_linear_v_marker_.color.r = 1.0;
    armor_linear_v_marker_.color.g = 1.0;

    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);
}

/**
 * @brief 初始化卡尔曼滤波器
 * 
 */
void ArmorSolverNode::init_ArmorStateEKF() {
    // update_Q - process noise covariance matrix
    q_pos_v_ = declare_parameter("ekf.lmtd_top_model.q_pos_v", 20.0);
    q_orientation_yaw_v_ = declare_parameter("ekf.lmtd_top_model.q_orientation_yaw_v", 800.0);

    q_armor_x_ = declare_parameter("ekf.armor_model.q_x", 0.01);
    q_armor_v_ = declare_parameter("ekf.armor_model.q_v", 100.0);

    r_armor_yaw_ = declare_parameter("ekf.armor_model.r_yaw", 1.0);
    r_armor_pitch_ = declare_parameter("ekf.armor_model.r_pitch", 1.5);
    r_armor_dis_ = declare_parameter("ekf.armor_model.r_distance", 50.0);

    // 辅助函数：生成 2x2 的 CV 模型协方差块，减少重复代码
    // sp_vision
    auto make_q_block = [this](double s2) {
        double t = dt_, t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        Eigen::Matrix2d block;
        // 这里的 t几 是 t^x ,不是 t*x，注意区分
        // clang-format off
        block <<    t4 * s2 / 4, t3 * s2 / 2, 
                    t3 * s2 / 2, t2 * s2;
        // clang-format on
        return block;
    };

    // ==========================================
    // 1. Robot State EKF (整车模型)
    // ==========================================

    // 整车模型
    // xa = x_armor, xc = x_robot_center
    // state: xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc
    // measurement: yaw, pitch, distance, angle
    // f - Process function
    auto f_sys = Predict(0.005); // 初始值，后续会更新
    // h - Observation function
    auto h_sys = Measure();

    auto u_q_sys = [this, make_q_block]() {
        Eigen::Matrix<double, X_N, X_N> q = Eigen::Matrix<double, X_N, X_N>::Zero();
        // 使用块操作填充对角线，逻辑清晰且不易出错
        // sp_vison
        q.block<2, 2>(0, 0) = make_q_block(q_pos_v_); // X
        q.block<2, 2>(2, 2) = make_q_block(q_pos_v_); // Y
        q.block<2, 2>(4, 4) = make_q_block(q_pos_v_); // Z
        q.block<2, 2>(6, 6) = make_q_block(q_orientation_yaw_v_); // Yaw

        q(8, 8) = 0.0; // r
        q(9, 9) = 0.0; // dr
        q(10, 10) = 0.0; // dz1
        q(11, 11) = 0.0; // dz2

        return q;
    };

    // sp_vision
    // 观测噪声协方差矩阵，即当角度误差较大时，适当增加噪声协方差，降低该观测的权重
    auto u_r_sys = [](const Eigen::Matrix<double, Z_N, 1>& z) {
        auto center_yaw = z[0];
        auto delta_angle = reduced_angle(z[3] - center_yaw);
        Eigen::DiagonalMatrix<double, Z_N> r;
        r.diagonal() << 4e-3, 4e-3, std::log(std::abs(delta_angle) + 1) + 1,
            std::log(std::abs(z[2]) + 1) / 200 + 9e-2;
        return r;
    };
    // 观测量 ypda 的角度保护
    auto u_i_sys = [](const Eigen::Matrix<double, Z_N, 1>& z,
                      const Eigen::Matrix<double, Z_N, 1>& z_pri) {
        Eigen::Matrix<double, Z_N, 1> innovation = z - z_pri;
        innovation(0) = reduced_angle(innovation(0));
        innovation(3) = reduced_angle(innovation(3));
        return innovation;
    };

    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, X_N> p0_sys;
    Eigen::VectorXd P0_dig { { 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1, 1 } };
    p0_sys = P0_dig.asDiagonal();
    tracker_->ekf =
        std::make_unique<RobotStateEKF>(f_sys, h_sys, u_q_sys, u_r_sys, p0_sys, u_i_sys);

    // ==========================================
    // 2. Armor State EKF (平移运动模型)
    // ==========================================

    // 平移运动模型
    auto f_xyz = EkfPredict(0.005);
    auto h_xyz = EkfMeasure();

    auto u_q_xyz = [this]() {
        Eigen::Matrix<double, X_N_, X_N_> q = Eigen::Matrix<double, X_N_, X_N_>::Zero();
        q.diagonal() << q_armor_x_, q_armor_v_, q_armor_x_, q_armor_v_, q_armor_x_, q_armor_v_;
        return q;
    };

    auto u_r_xyz = [this](const Eigen::Matrix<double, Z_N_, 1>& z) {
        Eigen::DiagonalMatrix<double, Z_N_> r;
        r.diagonal() << r_armor_yaw_, r_armor_pitch_, r_armor_dis_ * pow(z[2], 2);
        return r;
    };
    auto u_i_xyz = [](const Eigen::Matrix<double, Z_N_, 1>& z,
                      const Eigen::Matrix<double, Z_N_, 1>& z_pri) {
        Eigen::Matrix<double, Z_N_, 1> innovation = z - z_pri;
        innovation(0) = reduced_angle(innovation(0));
        return innovation;
    };

    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, X_N_> p0_xyz;
    p0_xyz.setIdentity();
    tracker_->ekf_point =
        std::make_unique<ArmorStateEKF>(f_xyz, h_xyz, u_q_xyz, u_r_xyz, p0_xyz, u_i_xyz);
}

void ArmorSolverNode::armorFrameWorkerLoop() {
    while (!should_stop_armor_frame_worker_.load()) {
        // 获得数据
        qd::utils::ArmorFrame armor_frame;
        if (!state_machine_.waitAndPopLatestArmorFrame(armor_frame)) {
            continue;
        }

        // ekf 更新处理
        auto armors_msg = std::make_shared<rm_interfaces::msg::Armors>();
        armors_msg->header = armor_frame.header;
        armors_msg->armors = std::move(armor_frame.armors);

        std::vector<rm_interfaces::msg::Armor> reprojected_armors;
        if (!processArmors(armors_msg, armor_frame.camera_info, reprojected_armors)) {
            continue;
        }

        // 发布装甲板（move 出 armors，避免再次拷贝）
        publishArmors(std::move(*armors_msg));

        // 发布可视化
        if (armor_frame.image.empty()) {
            continue;
        }

        ResultImageTask result_image_task;
        result_image_task.header = std::move(armor_frame.header);
        result_image_task.image = std::move(armor_frame.image);
        result_image_task.reprojected_armors = std::move(reprojected_armors);
        {
            std::lock_guard<std::mutex> lock(result_image_mutex_);
            pending_result_image_task_ = std::move(result_image_task);
        }
        result_image_cv_.notify_one();
    }
}

void ArmorSolverNode::publishArmors(rm_interfaces::msg::Armors&& armors_msg) {
    auto result_msg = std::make_unique<rm_interfaces::msg::Armors>(std::move(armors_msg));
    armors_pub_->publish(std::move(result_msg));
}

std::vector<rm_interfaces::msg::Armor> ArmorSolverNode::getReprojectedArmors() {
    if (solveYawPnP_ == nullptr || !armor_target_.tracking || tracker_ == nullptr
        || !solveYawPnP_->check_camera_params())
    {
        return {};
    }

    std::vector<rm_interfaces::msg::Armor> reprojected_armors;
    const auto armor_poses = qd::utils::getArmorPosesFromState(
        tracker_->target_state,
        armor_target_.armors_num,
        armor_target_.radius_list,
        armor_target_.dz_list
    );

    const std::string tracked_number = tracker_->tracked_id;
    const std::string armor_type = tracker_->tracked_armor.type;
    reprojected_armors.reserve(armor_poses.size());

    for (const auto& pose: armor_poses) {
        rm_interfaces::msg::Armor reprojected_armor;
        reprojected_armor.number = tracked_number;
        reprojected_armor.type = armor_type;

        reprojected_armor.pose.position.x = pose.position.x();
        reprojected_armor.pose.position.y = pose.position.y();
        reprojected_armor.pose.position.z = pose.position.z();

        const auto proj_pts =
            solveYawPnP_->projected_points(pose.position, pose.yaw, tracked_number, armor_type);

        for (size_t j = 0; j < 4 && j < proj_pts.size(); ++j) {
            reprojected_armor.image_points[j].x = proj_pts[j].x;
            reprojected_armor.image_points[j].y = proj_pts[j].y;
        }

        reprojected_armors.push_back(std::move(reprojected_armor));
    }

    return reprojected_armors;
}

void ArmorSolverNode::drawArmors(
    cv::Mat& img,
    const std::vector<rm_interfaces::msg::Armor>& armors,
    const std::vector<rm_interfaces::msg::Armor>& reprojected_armors,
    bool is_rgb
) {
    const auto reprojected_point_color = makeDrawColor(255, 255, 0, is_rgb);
    const auto reprojected_line_color = makeDrawColor(0, 255, 255, is_rgb);
    const auto connect_line_color = makeDrawColor(255, 165, 0, is_rgb);

    std::vector<std::vector<cv::Point>> all_armor_pts;
    for (const auto& armor: reprojected_armors) {
        std::vector<cv::Point> pts;
        for (const auto& pt: armor.image_points) {
            cv::Point p(static_cast<int>(pt.x), static_cast<int>(pt.y));
            pts.push_back(p);
            cv::circle(img, p, 4, reprojected_point_color, 1);
        }
        if (pts.size() == 4) {
            cv::line(img, pts[0], pts[1], reprojected_line_color, 1);
            cv::line(img, pts[1], pts[2], reprojected_line_color, 1);
            cv::line(img, pts[2], pts[3], reprojected_line_color, 1);
            cv::line(img, pts[3], pts[0], reprojected_line_color, 1);
            all_armor_pts.push_back(std::move(pts));
        }
    }

    // 未滤波的检测框彼此独立，不绘制整车模型的连接线。
    if (!enable_ekf_) {
        return;
    }
    const size_t num = all_armor_pts.size();
    for (size_t i = 0; i < num; ++i) {
        const auto& curr_pts = all_armor_pts[i];
        const auto& next_pts = all_armor_pts[(i + 1) % num];
        cv::line(img, curr_pts[3], next_pts[0], connect_line_color, 2);
        cv::line(img, curr_pts[2], next_pts[1], connect_line_color, 1);
    }
}

void ArmorSolverNode::resultImagePublishLoop() {
    while (true) {
        std::optional<ResultImageTask> result_image_task;

        {
            std::unique_lock<std::mutex> lock(result_image_mutex_);
            result_image_cv_.wait(lock, [this] {
                return should_stop_result_image_publish_.load()
                    || pending_result_image_task_.has_value();
            });

            if (should_stop_result_image_publish_.load() && !pending_result_image_task_) {
                break;
            }

            result_image_task = std::move(pending_result_image_task_);
            pending_result_image_task_.reset();
        }

        if (result_image_task) {
            publishResultImage(std::move(*result_image_task));
        }
    }
}

void ArmorSolverNode::publishResultImage(ResultImageTask result_image_task) {
    if (result_image_task.image.empty()) {
        return;
    }

    if (debug_mode_) {
        drawArmors(result_image_task.image, {}, result_image_task.reprojected_armors, true);
    }

    auto result_msg = cv_bridge::CvImage(
                          result_image_task.header,
                          sensor_msgs::image_encodings::RGB8,
                          result_image_task.image
    )
                          .toImageMsg();
    result_img_pub_.publish(result_msg);
}

bool ArmorSolverNode::processArmors(
    const rm_interfaces::msg::Armors::SharedPtr& armors_msg,
    const sensor_msgs::msg::CameraInfo& camera_info,
    std::vector<rm_interfaces::msg::Armor>& reprojected_armors
) {
    std::lock_guard<std::mutex> lock(solver_state_mutex_);

    // Lazy initialize solver owing to weak_from_this() can't be called in constructor
    if (enable_ekf_ && solver_ == nullptr) {
        solver_ = std::make_unique<Solver>(weak_from_this());
    }

    if (solveYawPnP_ == nullptr && enable_YawPnP) {
        FYT_INFO("armor_solver", "init solveYawPnP_");
        solveYawPnP_ = std::make_unique<SolveYawPnP>(weak_from_this());
        FYT_INFO("armor_solver", "init solveYawPnP_ success");
    }

    // 设置 solveYawPnP 的内参（仅在内参实际变化时执行 clone + set_camera_params）
    updateSolveYawPnPCameraIfChanged(camera_info);

    const auto source_header = armors_msg->header;

    // Tranform armor position from image frame to world coordinate
    for (auto& armor: armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = source_header;
        ps.pose = armor.pose;
        try {
            // 加 0.01s 等待 tf ，服务通信过来时太快，tf 还没准备好，导致帧率砍半
            armor.pose = tf2_buffer_->transform(ps, target_frame_, tf2::durationFromSec(0.01)).pose;

            //听其他队伍会把前哨识别成哨兵，这里加保险
            if (armor.pose.position.z > 0.5 && armor.number == "sentry") {
                armor.number = "outpost";
                FYT_WARN("armor_solver", "armor number detector error ");
            }

        } catch (const tf2::TransformException& ex) {
            FYT_ERROR("armor_solver", "Transform error: {}", ex.what());
            return false;
        }
    }
    armors_msg->header.frame_id = target_frame_;

    // Filter abnormal armors 去除不符合的装甲板
    armors_msg->armors.erase(
        std::remove_if(
            armors_msg->armors.begin(),
            armors_msg->armors.end(),
            [this](const rm_interfaces::msg::Armor& armor) {
                auto p = armor.pose.position;
                auto armor_pose = Eigen::Vector3d(p.x, p.y, p.z);
                // 判断不要的装甲板
                return armor_pose.norm() > max_armor_distance_;
            }
        ),
        armors_msg->armors.end()
    );

    // 跟踪期间排除同号地面反光：它的高度远离所有预测板位，会把 EKF 整体拉低。
    constexpr double kMaximumHeightResidualM = 0.30;
    const rclcpp::Time time = armors_msg->header.stamp;
    if (enable_ekf_ && tracker_->tracker_state != TrackerState::LOST && !tracker_->dz_list_.empty()
        && (time - last_time_).seconds() > 0.0 && (time - last_time_).seconds() <= 0.15)
    {
        armors_msg->armors.erase(
            std::remove_if(
                armors_msg->armors.begin(),
                armors_msg->armors.end(),
                [this, kMaximumHeightResidualM](const rm_interfaces::msg::Armor& armor) {
                    return armor.number == tracker_->tracked_id
                        && !is_armor_height_consistent(
                               armor.pose.position.z,
                               tracker_->dz_list_,
                               kMaximumHeightResidualM
                        );
                }
            ),
            armors_msg->armors.end()
        );
    }

    // Init message
    rm_interfaces::msg::Measurement measure_msg;
    rm_interfaces::msg::Target target_msg;
    target_msg.header.stamp = time;
    target_msg.header.frame_id = target_frame_;

    if (!enable_ekf_) {
        // 按本帧观测独立解算 yaw，不读取 EKF 的预测状态。
        if (solveYawPnP_) {
            std::vector<std::string> solved_ids;
            for (const auto& armor: armors_msg->armors) {
                if (std::find(solved_ids.begin(), solved_ids.end(), armor.number)
                    != solved_ids.end())
                {
                    continue;
                }
                solved_ids.push_back(armor.number);
                solveYawPnP_->solve(
                    armors_msg, source_header, M_PI / 4., armor.number, tf2_buffer_
                );
            }
        }

        // 观测量取离画面中心最近的装甲板；位置与朝向均来自本帧 PnP。
        if (!armors_msg->armors.empty()) {
            const auto nearest = std::min_element(
                armors_msg->armors.begin(),
                armors_msg->armors.end(),
                [](const auto& lhs, const auto& rhs) {
                    return lhs.distance_to_image_center < rhs.distance_to_image_center;
                }
            );
            const auto& position = nearest->pose.position;
            measure_msg.x = position.x;
            measure_msg.y = position.y;
            measure_msg.z = position.z;
            const double horizontal_distance = std::hypot(position.x, position.y);
            measure_msg.center_yaw = std::atan2(position.y, position.x);
            measure_msg.center_yaw_deg = measure_msg.center_yaw * 180.0 / M_PI;
            measure_msg.pitch = std::atan2(position.z, horizontal_distance);
            measure_msg.distance = std::hypot(horizontal_distance, position.z);
            tf2::Quaternion orientation;
            tf2::fromMsg(nearest->pose.orientation, orientation);
            tf2::Matrix3x3(orientation).getRPY(
                measure_msg.armor_roll, measure_msg.armor_pitch, measure_msg.yaw
            );
            measure_pub_->publish(measure_msg);
        }

        target_msg.tracking = false;
        armor_target_ = target_msg;
        target_pub_->publish(target_msg);
        reprojected_armors = armors_msg->armors;
        heartbeat_->publish();
        return true;
    }

    // Update tracker
    if (tracker_->tracker_state == TrackerState::LOST) {
        //对所有装甲板进行三分法求yaw
        if (solveYawPnP_) {
            solveYawPnP_
                ->solve(armors_msg, source_header, M_PI / 4., tracker_->tracked_id, tf2_buffer_);
        }

        tracker_->init(armors_msg);

    } else {
        //更新predictEKF的dt
        dt_ = (time - last_time_).seconds();

        if (dt_ > filter_reset_dt_thres_) {
            FYT_WARN(
                "armor_solver",
                "dt {:.3f}s > {:.3f}s, reset tracker EKF",
                dt_,
                filter_reset_dt_thres_
            );
            tracker_->init(armors_msg);
            target_msg.tracking = false;
            armor_target_ = target_msg;
            target_pub_->publish(std::make_unique<rm_interfaces::msg::Target>(std::move(target_msg))
            );
            last_time_ = time;
            heartbeat_->publish();
            reprojected_armors.clear();
            return true;
        }

        tracker_->lost_thres = std::max(1, std::abs(static_cast<int>(lost_time_thres_ / dt_)));

        // 更新 EKF 的预测时间 dt
        tracker_->ekf->setPredictFunc(Predict { dt_ });
        tracker_->ekf_point->setPredictFunc(EkfPredict { dt_ });

        //三分法求yaw
        if (solveYawPnP_) {
            solveYawPnP_->solve(
                armors_msg,
                source_header,
                tracker_->target_state(6),
                tracker_->tracked_id,
                tf2_buffer_
            );
        }

        // 更新模型
        tracker_->update(armors_msg);

        // 本帧没有有效观测时，不让预测沿用的自转速度累计旋转模式确认次数。
        if (tracker_->tracker_state == TrackerState::TEMP_LOST) {
            solver_->reset_top_level();
        }

        // 更新跟踪状态
        if (tracker_->tracker_state == TrackerState::DETECTING) {
            target_msg.tracking = false;
        } else if (tracker_->tracker_state == TrackerState::TRACKING || tracker_->tracker_state == TrackerState::TEMP_LOST)
        {
            target_msg.tracking = true;
        }
    }

    if (target_msg.tracking) {
        const auto& tracked_pose = tracker_->tracked_armor.pose;
        tf2::Quaternion tracked_q;
        tf2::fromMsg(tracked_pose.orientation, tracked_q);
        double armor_roll = 0.0;
        double armor_pitch = 0.0;
        double armor_yaw = 0.0;
        tf2::Matrix3x3(tracked_q).getRPY(armor_roll, armor_pitch, armor_yaw);
        // 同时发布直角坐标观测量和对应球坐标，便于调试 EKF 输入
        measure_msg.x = tracked_pose.position.x;
        measure_msg.y = tracked_pose.position.y;
        measure_msg.z = tracked_pose.position.z;
        measure_msg.center_yaw = tracker_->measurement(0);
        measure_msg.center_yaw_deg = measure_msg.center_yaw * 180.0 / M_PI;
        measure_msg.pitch = tracker_->measurement(1);
        measure_msg.distance = tracker_->measurement(2);
        measure_msg.yaw = tracker_->measurement(3);
        measure_msg.armor_pitch = armor_pitch;
        measure_msg.armor_roll = armor_roll;
        measure_pub_->publish(
            std::make_unique<rm_interfaces::msg::Measurement>(std::move(measure_msg))
        );

        // Store and Publish the target_msg
        // Fill target message
        const auto& state = tracker_->target_state;
        target_msg.id = tracker_->tracked_id;
        target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
        target_msg.position.x = state(0);
        target_msg.velocity.x = state(1);
        target_msg.position.y = state(2);
        target_msg.velocity.y = state(3);
        target_msg.position.z = state(4);
        target_msg.velocity.z = state(5);
        target_msg.yaw = state(6);
        target_msg.v_yaw = state(7);
        target_msg.top_level = solver_->get_top_level();
        //填充每块装甲板的高度差
        target_msg.dz_list.clear();
        for (const auto& dz: tracker_->dz_list_) {
            target_msg.dz_list.emplace_back(dz);
        }
        target_msg.radius_list.clear();
        for (const auto& r: tracker_->radius_list_) {
            target_msg.radius_list.emplace_back(r);
        }

        //平移运动模型
        {
            const auto& state_armor = tracker_->armor_state;
            target_msg.position_armor.x = state_armor(0);
            target_msg.velocity_armor.x = state_armor(1);
            target_msg.position_armor.y = state_armor(2);
            target_msg.velocity_armor.y = state_armor(3);
            target_msg.position_armor.z = state_armor(4);
            target_msg.velocity_armor.z = state_armor(5);
        }

        // NIS chi-square consistency test
        target_msg.nis = tracker_->ekf->getNIS();
    }
    armor_target_ = target_msg;
    target_pub_->publish(std::make_unique<rm_interfaces::msg::Target>(std::move(target_msg)));

    last_time_ = time;
    reprojected_armors = getReprojectedArmors();

    heartbeat_->publish();
    return true;
}

/**
 * @brief 发布 marker
 * 
 * @param target_msg 机器人状态信息
 * @param gimbal_cmd 瞄准目标位置
 */
void ArmorSolverNode::publishMarkers(
    const rm_interfaces::msg::Target& target_msg,
    const rm_interfaces::msg::GimbalCmd& gimbal_cmd
) noexcept {
    position_marker_.header = target_msg.header;
    linear_v_marker_.header = target_msg.header;
    angular_v_marker_.header = target_msg.header;
    armors_marker_.header = target_msg.header;
    selection_marker_.header = target_msg.header;
    trajectory_marker_.header = target_msg.header;
    armor_position_marker_.header = target_msg.header;
    armor_linear_v_marker_.header = target_msg.header;

    visualization_msgs::msg::MarkerArray marker_array;

    if (target_msg.tracking) {
        // double yaw = target_msg.yaw;
        double xc = target_msg.position.x, yc = target_msg.position.y, zc = target_msg.position.z;
        double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;

        // 整车中心位置
        position_marker_.action = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = xc;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = zc;

        // 整车中心速度
        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += vx;
        arrow_end.y += vy;
        arrow_end.z += vz;
        linear_v_marker_.points.emplace_back(arrow_end);

        // 角度速度方向
        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += target_msg.v_yaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        // 装甲板位置和朝向
        armors_marker_.action = visualization_msgs::msg::Marker::ADD;
        armors_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
        // Draw armors（由 rm_utils 根据状态量统一推导所有装甲板 xyz + yaw）
        const auto armor_poses = qd::utils::getArmorPosesFromState(
            tracker_->target_state,
            target_msg.armors_num,
            target_msg.radius_list,
            target_msg.dz_list
        );
        for (size_t i = 0; i < armor_poses.size(); i++) {
            const auto& pose = armor_poses[i];
            armors_marker_.id = i;
            armors_marker_.pose.position.x = pose.position.x();
            armors_marker_.pose.position.y = pose.position.y();
            armors_marker_.pose.position.z = pose.position.z();
            tf2::Quaternion q;
            q.setRPY(0, target_msg.id == "outpost" ? -0.2618 : 0.2618, pose.yaw);
            armors_marker_.pose.orientation = tf2::toMsg(q);
            marker_array.markers.emplace_back(armors_marker_);
        }

        // 瞄准点位置
        selection_marker_.action = visualization_msgs::msg::Marker::ADD;
        selection_marker_.points.clear();
        selection_marker_.pose.position.y = gimbal_cmd.distance * sin(gimbal_cmd.yaw * M_PI / 180);
        selection_marker_.pose.position.x = gimbal_cmd.distance * cos(gimbal_cmd.yaw * M_PI / 180);
        selection_marker_.pose.position.z =
            gimbal_cmd.distance * sin(gimbal_cmd.pitch * M_PI / 180);

        // 预测弹道位置
        trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
        trajectory_marker_.points.clear();
        trajectory_marker_.header.frame_id = gimble_frame_;
        for (const auto& point: solver_->getTrajectory()) {
            geometry_msgs::msg::Point p;
            p.x = point.first;
            p.z = point.second;
            trajectory_marker_.points.emplace_back(p);
        }
        if (gimbal_cmd.fire_advice) {
            trajectory_marker_.color.r = 0;
            trajectory_marker_.color.g = 1;
            trajectory_marker_.color.b = 0;
        } else {
            trajectory_marker_.color.r = 1;
            trajectory_marker_.color.g = 1;
            trajectory_marker_.color.b = 1;
        }

        //平移运动模型可视化
/*         {
            double xa = target_msg.position_armor.x, ya = target_msg.position_armor.y,
                   za = target_msg.position_armor.z;
            double vxa = target_msg.velocity_armor.x, vya = target_msg.velocity_armor.y,
                   vza = target_msg.velocity_armor.z;
            armor_position_marker_.action = visualization_msgs::msg::Marker::ADD;
            armor_position_marker_.pose.position.x = xa;
            armor_position_marker_.pose.position.y = ya;
            armor_position_marker_.pose.position.z = za;

            armor_linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
            armor_linear_v_marker_.points.clear();
            armor_linear_v_marker_.points.emplace_back(armor_position_marker_.pose.position);
            geometry_msgs::msg::Point armor_arrow_end = armor_position_marker_.pose.position;
            armor_arrow_end.x += vxa;
            armor_arrow_end.y += vya;
            armor_arrow_end.z += vza;
            armor_linear_v_marker_.points.emplace_back(armor_arrow_end);
        } */
    } else {
        position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
        trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
        selection_marker_.action = visualization_msgs::msg::Marker::DELETE;
        armor_position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        armor_linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    }

    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(trajectory_marker_);
    marker_array.markers.emplace_back(linear_v_marker_);
    marker_array.markers.emplace_back(angular_v_marker_);
    marker_array.markers.emplace_back(armors_marker_);
    marker_array.markers.emplace_back(selection_marker_);
    marker_array.markers.emplace_back(armor_position_marker_);
    marker_array.markers.emplace_back(armor_linear_v_marker_);
    marker_pub_->publish(
        std::make_unique<visualization_msgs::msg::MarkerArray>(std::move(marker_array))
    );
}

void ArmorSolverNode::updateSolveYawPnPCameraIfChanged(
    const sensor_msgs::msg::CameraInfo& camera_info
) {
    if (solveYawPnP_ == nullptr) {
        return;
    }

    const bool k_changed = !camera_params_cached_
        || !std::equal(camera_info.k.begin(), camera_info.k.end(), last_camera_k_.begin());
    const bool d_changed = !camera_params_cached_ || camera_info.d.size() != last_camera_d_.size()
        || !std::equal(camera_info.d.begin(), camera_info.d.end(), last_camera_d_.begin());

    if (!k_changed && !d_changed) {
        return;
    }

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    getCameraParams(camera_info, camera_matrix, distortion_coefficients);
    if (camera_matrix.empty() || distortion_coefficients.empty()) {
        return;
    }

    solveYawPnP_->set_camera_params(camera_matrix, distortion_coefficients);
    std::copy(camera_info.k.begin(), camera_info.k.end(), last_camera_k_.begin());
    last_camera_d_ = camera_info.d;
    camera_params_cached_ = true;
}

/**
 * @brief 服务通信回调，设置是否进行自瞄跟踪，有状态机后废弃
 * 
 * @param request 
 * @param response 
 */
void ArmorSolverNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response
) {
    response->success = true;

    VisionMode mode = static_cast<VisionMode>(request->mode);
    std::string mode_name = visionModeToString(mode);
    if (mode_name == "UNKNOWN") {
        FYT_ERROR("armor_solver", "Invalid mode: {}", request->mode);
        return;
    }

    switch (mode) {
        case VisionMode::AUTO_AIM_RED:
        case VisionMode::AUTO_AIM_BLUE: {
            enable_ = true;
            break;
        }
        default: {
            enable_ = false;
            break;
        }
    }

    FYT_WARN("armor_solver", "Set Mode to {}", visionModeToString(mode));
}

void ArmorSolverNode::serialCallback(
    const rm_interfaces::msg::SerialReceiveData::ConstSharedPtr serial_data
) {
    const double raw_bullet_speed = serial_data->bullet_speed;
    if (!std::isfinite(raw_bullet_speed) || raw_bullet_speed <= 10.0) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(solver_state_mutex_);
        if (solver_ == nullptr) {
            return;
        }
        // 每条新反馈仅更新一次，避免解算循环重复滤波同一份数据。
        solver_->update_bullet_speed(raw_bullet_speed);
    }

    auto result = this->set_parameter(rclcpp::Parameter("solver.bullet_speed", raw_bullet_speed));

    if (!result.successful) {
        RCLCPP_WARN(this->get_logger(), "Failed to set bullet_speed parameter");
    }
}

} // namespace qd::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(qd::auto_aim::ArmorSolverNode)
