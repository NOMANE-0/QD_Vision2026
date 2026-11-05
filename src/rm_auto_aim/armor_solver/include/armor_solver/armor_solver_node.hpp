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

#ifndef ARMOR_SOLVER_SOLVER_NODE_HPP_
#define ARMOR_SOLVER_SOLVER_NODE_HPP_

// ros2
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <image_transport/publisher.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rm_interfaces/msg/detail/serial_receive_data__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
// project
#include "armor_solver/armor_solver.hpp"
#include "armor_solver/armor_tracker.hpp"
// #include "armor_solver/solveYawPnP.hpp"
#include "armor_solver/armor_yaw_solver.hpp"
#include "armor_solver/yaw_command_filter.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/measurement.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/vision_state_machine.hpp"

namespace qd::auto_aim {
class ArmorSolverNode: public rclcpp::Node {
public:
    explicit ArmorSolverNode(const rclcpp::NodeOptions& options);
    ~ArmorSolverNode() override;

private:
    struct ResultImageTask {
        std_msgs::msg::Header header;
        cv::Mat image;
        std::vector<rm_interfaces::msg::Armor> reprojected_armors;
    };

    void armorFrameWorkerLoop();
    /**
     * @brief 处理本帧 PnP 观测，并按配置更新 EKF 或发布未滤波观测。
     * @param armors_ptr 输入相机坐标系位姿，输出 target_frame 坐标系位姿。
     * @param camera_info 本帧相机内参，供装甲板 yaw 解算使用。
     * @param reprojected_armors 输出绘图数据，关闭 EKF 时使用原始检测角点。
     * @return 坐标变换和本帧处理成功时返回 true。
     * @note enable_ekf 在启动时读取；关闭时不生成跟踪预测或云台指令。
     */
    bool processArmors(
        const rm_interfaces::msg::Armors::SharedPtr& armors_ptr,
        const sensor_msgs::msg::CameraInfo& camera_info,
        std::vector<rm_interfaces::msg::Armor>& reprojected_armors
    );
    /**
     * @brief 根据新的有效串口反馈更新解算弹速并记录原始弹速参数。
     * @param serial_data 电控反馈；弹速单位米每秒，仅接受有限且大于 10 的值。
     * @note 持有解算状态互斥锁时更新滤波，参数写入在锁外进行。
     */
    void serialCallback(const rm_interfaces::msg::SerialReceiveData::ConstSharedPtr serial_data);
    void publishArmors(rm_interfaces::msg::Armors&& armors_msg);
    std::vector<rm_interfaces::msg::Armor> getReprojectedArmors();
    void drawArmors(
        cv::Mat& img,
        const std::vector<rm_interfaces::msg::Armor>& armors,
        const std::vector<rm_interfaces::msg::Armor>& reprojected_armors,
        bool is_rgb
    );
    void resultImagePublishLoop();
    void publishResultImage(ResultImageTask result_image_task);

    void initMarkers() noexcept;
    void init_ArmorStateEKF();

    void publishMarkers(
        const rm_interfaces::msg::Target& target_msg,
        const rm_interfaces::msg::GimbalCmd& gimbal_cmd
    ) noexcept;

    void setModeCallback(
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response
    );

    bool debug_mode_;
    // 启动时读取；关闭后仅发布未滤波的装甲板位姿和结果图。
    bool enable_ekf_;

    // Heartbeat
    HeartBeatPublisher::SharedPtr heartbeat_;

    // The time when the last message was received
    rclcpp::Time last_time_;
    double dt_;
    double filter_reset_dt_thres_;

    // Armor tracker
    double q_pos_v_, q_orientation_yaw_v_;

    double q_armor_x_, q_armor_v_;
    double r_armor_yaw_, r_armor_pitch_, r_armor_dis_;

    double lost_time_thres_;
    std::unique_ptr<Tracker> tracker_;

    // Armor Solver
    std::unique_ptr<Solver> solver_;
    YawCommandFilter yaw_command_filter_;
    double yaw_filter_fire_tolerance_deg_ { 0.5 };

    // Armor Yaw Solver
    std::unique_ptr<SolveYawPnP> solveYawPnP_;

    std::string target_frame_;
    std::string gimble_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    rm_interfaces::msg::Target armor_target_;

    // Measurement publisher
    rclcpp::Publisher<rm_interfaces::msg::Measurement>::SharedPtr measure_pub_;

    // Publisher
    rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;
    rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
    image_transport::Publisher result_img_pub_;

    // for pub img thread
    std::thread armor_frame_worker_thread_;
    std::thread result_image_publish_thread_;
    std::mutex result_image_mutex_;
    std::condition_variable result_image_cv_;
    std::optional<ResultImageTask> pending_result_image_task_;
    std::atomic<bool> should_stop_armor_frame_worker_ { false };
    std::atomic<bool> should_stop_result_image_publish_ { false };

    rclcpp::TimerBase::SharedPtr pub_timer_;
    void timerCallback();

    // Subscriber
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_data_sub_;

    // Enable/Disable Armor Solver
    bool enable_;
    bool enable_YawPnP;

    rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

    // Visualization marker publisher
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker trajectory_marker_;
    visualization_msgs::msg::Marker armors_marker_;
    visualization_msgs::msg::Marker selection_marker_;
    visualization_msgs::msg::Marker armor_position_marker_;
    visualization_msgs::msg::Marker armor_linear_v_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    double max_armor_distance_ = 10.0;

    // 状态机
    qd::utils::VisionStateMachine& state_machine_;
    std::mutex solver_state_mutex_;

    // 缓存上一次刷给 SolveYawPnP 的相机内参，避免每帧都做 cv::Mat::clone() 与 set_camera_params。
    std::array<double, 9> last_camera_k_ {};
    std::vector<double> last_camera_d_;
    bool camera_params_cached_ { false };

    // 当 CameraInfo 的 k/d 相对缓存变化时，刷新缓存并把内参喂给 SolveYawPnP。
    void updateSolveYawPnPCameraIfChanged(const sensor_msgs::msg::CameraInfo& camera_info);
};

} // namespace qd::auto_aim

#endif // ARMOR_SOLVER_SOLVER_NODE_HPP_
