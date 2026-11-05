// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <cmath>

namespace qd::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n): node_(n) {
    auto node = node_.lock();
    // 装甲板大小
    shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
    shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
    // 延迟时间
    predict2send_delay_ = node->declare_parameter("solver.predict2send_delay", 0.0);
    controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
    additional_prediction_time = node->declare_parameter("solver.additional_prediction_time", 0.0);
    // 弹速
    bullet_speed = node->declare_parameter("solver.bullet_speed", 16.0);
    bullet_speed_filter_alpha_ =
        std::clamp(node->declare_parameter("solver.bullet_speed_filter_alpha", 0.2), 0.0, 1.0);
    // 配置弹速仅用于兜底，首次有效反馈直接初始化滤波状态。
    bullet_speed_filter_initialized_ = false;
    gravity = node->declare_parameter("solver.gravity", 9.8);
    // 空气阻力
    resistance = node->declare_parameter("solver.resistance", 0.001);

    use_armor_top_ = node->declare_parameter("solver.use_armor_top", false);
    enable_motion_prediction_ = node->declare_parameter("solver.enable_motion_prediction", true);

    // 补偿器
    std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
    trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
    trajectory_compensator_->iteration_times =
        node->declare_parameter("solver.iteration_times", 20);
    trajectory_compensator_->velocity = bullet_speed;
    trajectory_compensator_->gravity = gravity;
    trajectory_compensator_->resistance = resistance;
    trajectory_compensator_->rk4_time_step =
        node->declare_parameter("solver.rk4.time_step", 0.003);
    trajectory_compensator_->rk4_max_steps =
        static_cast<int>(node->declare_parameter("solver.rk4.max_steps", 8000));
    trajectory_compensator_->rk4_hit_distance_threshold =
        node->declare_parameter("solver.rk4.hit_distance_threshold", 0.02);

    // 角度补偿器
    manual_compensator_ = std::make_unique<ManualCompensator>();
    auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string> {});
    if (!manual_compensator_->updateMapFlow(angle_offset)) {
        FYT_WARN("armor_solver", "Manual compensator update failed!");
    }

    // 反陀螺参数
    activate_threshold[0] = 0.0;
    activate_threshold[1] = node->declare_parameter("solver.top1.activate_threshold", 90.0);
    activate_threshold[2] = node->declare_parameter("solver.top2.activate_threshold", 500.0);
    top_activation_frames_ = static_cast<int>(std::clamp<std::int64_t>(
        node->declare_parameter("solver.top_activation_frames", 4), 1, 16
    ));
    for (auto& angle: activate_threshold)
        angle = angle * M_PI / 180.; // 转弧度制
    deactivate_threshold[0] = 0.0;
    deactivate_threshold[1] = node->declare_parameter("solver.top1.deactivate_threshold", 60.0);
    deactivate_threshold[2] = node->declare_parameter("solver.top2.deactivate_threshold", 450.0);
    for (auto& angle: deactivate_threshold)
        angle = angle * M_PI / 180.; // 转弧度制
    max_out_error[0] = node->declare_parameter("solver.top0.max_out_error", 1.8);
    max_out_error[1] = node->declare_parameter("solver.top1.max_out_error", 0.6);
    max_out_error[2] = node->declare_parameter("solver.top2.max_out_error", 1.8);
    max_orientation_angle[0] = node->declare_parameter("solver.top0.max_orientation_angle", 58.88);
    max_orientation_angle[1] = node->declare_parameter("solver.top1.max_orientation_angle", 0.0);
    max_orientation_angle[2] = node->declare_parameter("solver.top2.max_orientation_angle", 0.0);
    for (auto& angle: max_orientation_angle)
        angle = angle * M_PI / 180.; // 转弧度制

    node.reset();

    this->top_level = 0;
}

/**
 * @brief 获得瞄准信息
 * 
 * @param target 目标状态信息
 * @param current_time 当前时间
 * @param tf2_buffer_ ros的坐标监听，用于获取云台姿态
 * @return rm_interfaces::msg::GimbalCmd 
 */
rm_interfaces::msg::GimbalCmd Solver::solve(
    rm_interfaces::msg::Target& target,
    const rclcpp::Time& current_time,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_
) {
    // Get newest parameters
    // 更新参数
    try {
        auto node = node_.lock();
        predict2send_delay_ = node->get_parameter("solver.predict2send_delay").as_double();
        controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
        additional_prediction_time =
            node->get_parameter("solver.additional_prediction_time").as_double();
        bullet_speed_filter_alpha_ = std::clamp(
            node->get_parameter("solver.bullet_speed_filter_alpha").as_double(),
            0.0,
            1.0
        );
        use_armor_top_ = node->get_parameter("solver.use_armor_top").as_bool();
        enable_motion_prediction_ = node->get_parameter("solver.enable_motion_prediction").as_bool();
        gimble_frame_ = node->get_parameter("gimble_frame").as_string();
        node.reset();

        prediction_delay_ = (current_time - target.header.stamp).seconds();
    } catch (const std::runtime_error& e) {
        FYT_ERROR("armor_solver", "{}", e.what());
    }

    // Get current roll, yaw and pitch of gimbal
    // 获得云台当前姿态
    try {
        auto gimbal_tf =
            tf2_buffer_->lookupTransform(target.header.frame_id, gimble_frame_, tf2::TimePointZero);
        auto msg_q = gimbal_tf.transform.rotation;

        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3(tf_q).getRPY(gimbal_rpy_[0], gimbal_rpy_[1], gimbal_rpy_[2]);
        gimbal_rpy_[1] = -gimbal_rpy_[1]; // 转成向上为正
    } catch (tf2::TransformException& ex) {
        FYT_ERROR("armor_solver", "{}", ex.what());
        throw ex;
    }

    // 更新旋转状态
    update_top_level(
        target.v_yaw,
        rclcpp::Time(target.header.stamp).nanoseconds(),
        target.id
    );
    target.top_level = get_top_level();

    if (target.id == "outpost") {
        pitch_inclined_ = -15.0 * M_PI / 180.0; // 前哨战固定15度
    } else {
        pitch_inclined_ = 15.0 * M_PI / 180.0;
    }

    // 根据top_level选择决策
    RosTarget target_msg(target);
    ShootState target_aim;
    if (get_top_level() > 0) {
        target_aim = get_aim(target_msg, gimbal_rpy_);
    } else if (use_armor_top_) {
        target_aim = get_aim_positon(target_msg, gimbal_rpy_);
    } else {
        target_aim = get_aim(target_msg, gimbal_rpy_);
    }

    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header = target.header;
    gimbal_cmd.id = target.id;
    auto send_ypd = target_aim.aim_ypd;
    auto send_ypd_v = target_aim.aim_ypd_v;
    // 关闭运动预测时也跳过额外角速度外推，保留速度输出供对照诊断。
    const double extra_prediction_time =
        enable_motion_prediction_ ? additional_prediction_time : 0.0;
    gimbal_cmd.yaw =
        reduced_angle(send_ypd.yaw + (send_ypd_v.yaw * extra_prediction_time)) * 180 / M_PI;
    gimbal_cmd.pitch =
        (send_ypd.pitch + (send_ypd_v.pitch * extra_prediction_time)) * 180 / M_PI;
    gimbal_cmd.yaw_diff = reduced_angle(send_ypd.yaw - gimbal_rpy_[2]) * 180 / M_PI;
    gimbal_cmd.pitch_diff = reduced_angle(send_ypd.pitch - gimbal_rpy_[1]) * 180 / M_PI;
    gimbal_cmd.distance = send_ypd.distance;
    gimbal_cmd.fire_advice = target_aim.shoot == ShootMode::SHOOT_NOW ? true : false;
    // 以下信息主要用于调试和可视化
    gimbal_cmd.bullet_speed = get_bullet_speed();
    gimbal_cmd.exp_yaw = send_ypd.yaw * 180 / M_PI;
    gimbal_cmd.exp_pitch = send_ypd.pitch * 180 / M_PI;
    gimbal_cmd.yaw_vel = send_ypd_v.yaw * 180 / M_PI;
    gimbal_cmd.pitch_vel = send_ypd_v.pitch * 180 / M_PI;
    gimbal_cmd.target_v_yaw = target_aim.state(7);

    return gimbal_cmd;
}

/**
 * @brief 根据 pitch 模拟弹道轨迹
 * 
 * @return std::vector<std::pair<double, double>> 
 */
std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
    auto trajectory = trajectory_compensator_->getTrajectory(15, gimbal_rpy_[1]);
    // Rotate
    for (auto& p: trajectory) {
        double x = p.first;
        double y = p.second;
        p.first = x * cos(gimbal_rpy_[1]) + y * sin(gimbal_rpy_[1]);
        p.second = -x * sin(gimbal_rpy_[1]) + y * cos(gimbal_rpy_[1]);
    }
    return trajectory;
}

void Solver::reset_top_level() noexcept {
    top_level = 0;
    top_activation_count_ = 0;
    top_candidate_sign_ = 0;
    last_top_frame_stamp_ns_ = 0;
    top_target_id_.clear();
}

void Solver::update_top_level(
    double angular_velocity,
    std::int64_t frame_stamp_ns,
    const std::string& target_id
) {
    constexpr std::int64_t kMaximumFrameGapNs = 150000000;
    if (target_id != top_target_id_
        || (last_top_frame_stamp_ns_ > 0
            && frame_stamp_ns - last_top_frame_stamp_ns_ > kMaximumFrameGapNs))
    {
        reset_top_level();
    }
    if (last_top_frame_stamp_ns_ > 0 && frame_stamp_ns <= last_top_frame_stamp_ns_) {
        return;
    }
    last_top_frame_stamp_ns_ = frame_stamp_ns;
    top_target_id_ = target_id;

    if (!std::isfinite(angular_velocity)) {
        reset_top_level();
        return;
    }

    const double credible_abs_w = std::abs(angular_velocity);
    // 最大 top 等级是 2 写死了
    // 是这样的
    // 0 级：基本上直接跟随就行
    // 1 级：引入 indirect
    // 2 级：转得太快跟不上，改成往中间打
    const double next_level_activate_w =
        this->top_level == 2 ? DBL_MAX : activate_threshold[this->top_level + 1];
    const double deactivate_w =
        this->top_level == 0 ? 0.0 : deactivate_threshold[this->top_level];

    // 低于退出阈值时降级；进入高等级须由连续新图像确认。
    if (this->top_level > 0 && credible_abs_w < deactivate_w) {
        this->top_level -= 1;
        top_activation_count_ = 0;
        top_candidate_sign_ = 0;
        return;
    }

    if (this->top_level < 2 && credible_abs_w >= next_level_activate_w) {
        const int sign = angular_velocity > 0.0 ? 1 : -1;
        top_activation_count_ = sign == top_candidate_sign_ ? top_activation_count_ + 1 : 1;
        top_candidate_sign_ = sign;
        if (top_activation_count_ >= top_activation_frames_) {
            this->top_level += 1;
            top_activation_count_ = 0;
        }
    } else {
        top_activation_count_ = 0;
        top_candidate_sign_ = 0;
    }
}

/**
  @brief 得到当前旋转状态
  @return 返回一个 int 类型数据，0 没有旋转，1 在旋转，2 在高速旋转
*/
int Solver::get_top_level() const {
    return this->top_level;
}

/**
  @brief 基于当前弹道模型计算飞行时间
  @param target_pos [IN] 目标三维坐标 xyz
  @return 飞行所需时间，单位秒
*/
double Solver::get_ballistic_flying_time(const Eigen::Vector3d& target_pos) {
    const double safe_bullet_speed = std::max(get_bullet_speed(), 1e-3);
    const double distance_xy = target_pos.head(2).norm();

    double pitch = 0.0;
    if (!trajectory_compensator_ || !trajectory_compensator_->compensate(target_pos, pitch)) {
        return target_pos.norm() / safe_bullet_speed;
    }

    Eigen::Vector3d aim_pos = target_pos;
    aim_pos.z() = distance_xy * std::tan(pitch);
    const double flying_time = trajectory_compensator_->getFlyingTime(aim_pos);

    if (!std::isfinite(flying_time) || flying_time < 0.0) {
        return target_pos.norm() / safe_bullet_speed;
    }

    return flying_time;
}

/**
  @brief 迭代求解命中时刻对应的装甲板状态
  @param state [IN] 当前装甲板状态
  @param base_delay [IN] 开火前固定延迟
  @return 命中时刻的装甲板状态
*/
State Solver::solve_impact_state(const State& state, double base_delay) {
    if (!enable_motion_prediction_) {
        return state;
    }

    constexpr int kMaxIterations = 8;
    constexpr double kTimeTolerance = 1e-3;
    constexpr double kRelaxation = 0.5;

    double dt = std::max(0.0, base_delay);
    State predicted_state = state_predict(state, dt);
    dt += get_ballistic_flying_time(state_to_armor_pos(predicted_state));
    predicted_state = state_predict(state, dt);

    for (int i = 0; i < kMaxIterations; ++i) {
        const double solved_dt = std::max(
            0.0,
            base_delay + get_ballistic_flying_time(state_to_armor_pos(predicted_state))
        );
        if (std::abs(solved_dt - dt) < kTimeTolerance) {
            dt = solved_dt;
            break;
        }

        dt = (1.0 - kRelaxation) * dt + kRelaxation * solved_dt;
        predicted_state = state_predict(state, dt);
    }

    return state_predict(state, dt);
}

/**
  @brief 获得各个装甲板的向量，xyz是中心点，便于后面想要直接算出当前板子的位置
*/
std::vector<State> Solver::get_armors_state(const RosTarget& target_msg) {
    auto target_yaw = target_msg.yaw_;
    auto v_yaw = target_msg.v_yaw_;
    auto c_pos = target_msg.center_position_;
    auto c_vel = target_msg.center_velocity_;
    // auto zc = c_pos.z();

    auto armors_num = target_msg.armors_num_;
    auto r_list = target_msg.radius_list_;
    auto dz_list = target_msg.dz_list_;
    auto armors_state = std::vector<State>(
        armors_num,
        { c_pos.x(), c_vel.x(), c_pos.y(), c_vel.y(), c_pos.z(), c_vel.z(), 0.0, v_yaw, 0.0 }
    );
    // Calculate the position of each armor
    double r = 0., target_dz = 0.;
    for (int i = 0; i < armors_num; i++) {
        double temp_yaw = reduced_angle(target_yaw + i * (2 * M_PI / armors_num));

        r = r_list[i]; // 使用每块装甲板的半径
        target_dz = dz_list[i];
        armors_state[i](4) = target_dz;
        armors_state[i](6) = temp_yaw;
        armors_state[i](8) = r;
    }
    return armors_state;
}

/**
  @brief 根据目标状态获得瞄准信息，考虑旋转
*/
ShootState Solver::get_aim(const RosTarget& target_msg, const Eigen::Vector3d& gimbal_rpy) {
    // 预测dt后装甲板state
    auto filters = get_armors_state(target_msg);
    std::vector<State> hit_state_vec {};
    const double base_delay = prediction_delay_ + predict2send_delay_;
    for (const auto& filter: filters) {
        hit_state_vec.emplace_back(solve_impact_state(filter, base_delay));
    }

    // 计算弹道
    const bool allow_indirect = this->top_level > 0; // 是否允许切板，平移非陀螺为 false
    auto target_aim = state_vec_to_aim(
        hit_state_vec,
        max_orientation_angle[this->top_level],
        max_out_error[this->top_level],
        allow_indirect,
        gimbal_rpy
    );

    // 这里比上面的多加了个开火延迟，其他没区别
    std::vector<State> command_hit_state_vec {};
    const double command_delay = base_delay + controller_delay_;
    for (const auto& filter: filters) {
        command_hit_state_vec.emplace_back(solve_impact_state(filter, command_delay));
    }
    auto command_target_aim = state_vec_to_aim(
        command_hit_state_vec,
        max_orientation_angle[this->top_level],
        max_out_error[this->top_level],
        true,
        gimbal_rpy
    );

    // 火控
    target_aim.shoot = isSuggestFire(
        target_aim.aim_ypd,
        command_target_aim.aim_ypd,
        gimbal_rpy,
        max_out_error[this->top_level],
        reduced_angle(target_aim.state(6) - gimbal_rpy[2]),
        pitch_inclined_ // 角度不知道有没有反
    );

    return target_aim;
}

/**
 * @brief 根据目标状态获得瞄准信息，直接使用正在跟踪的那块
 * 
 * @param target_msg 
 * @param gimbal_rpy 
 * @return ShootState 
 */
ShootState Solver::get_aim_positon(const RosTarget& target_msg, const Eigen::Vector3d& gimbal_rpy) {
    // 预测dt后装甲板state
    Eigen::Vector3d p = target_msg.armor_position_;
    auto p_v = target_msg.armor_velocity_;
    const State armor_state { p.x(), p_v.x(), p.y(), p_v.y(), p.z(), p_v.z(), 0, 0, 0 };
    const State pre_state =
        solve_impact_state(armor_state, prediction_delay_ + predict2send_delay_);

    //弹道解算和火控
    const Eigen::Vector3d target_pos { pre_state(0), pre_state(2), pre_state(4) };
    const auto aim_ypd = target_pos_to_aim_ypd(target_pos);

    auto isFire = isSuggestFire(
        aim_ypd,
        aim_ypd,
        gimbal_rpy,
        this->max_out_error[this->top_level],
        0.0,
        pitch_inclined_
    );

    auto target_aim = YpdCoord_to_ShootState(aim_ypd, isFire);
    // 普通跟随的状态也须明确赋值，避免串口读取未初始化的目标自转速度。
    target_aim.state = pre_state;
    target_aim.aim_ypd = aim_ypd;
    target_aim.aim_ypd_v = get_gimbal_ypd_v(p, p_v, gimbal_rpy);

    return target_aim;
}

/**
  @brief 弹道求解，在于计算 pitch
*/
ShootParam Solver::target_pos_to_shoot_param(const Eigen::Vector3d& target_pos) {
    Eigen::Vector3d target_xyz_i_barrel = target_pos;
    const double bs = this->get_bullet_speed();
    const double target_xy = std::hypot(target_xyz_i_barrel.x(), target_xyz_i_barrel.y());
    double shoot_angle = std::atan2(target_xyz_i_barrel.z(), target_xy);
    Eigen::Vector3d aim_xyz_i_barrel = target_xyz_i_barrel;

    double compensated_pitch = shoot_angle;
    if (trajectory_compensator_->compensate(target_xyz_i_barrel, compensated_pitch)
        && std::isfinite(compensated_pitch))
    {
        shoot_angle = compensated_pitch;
        aim_xyz_i_barrel.z() = target_xy * std::tan(shoot_angle);
    }

    return ShootParam { bs, shoot_angle, aim_xyz_i_barrel, target_pos };
}

void Solver::update_bullet_speed(double raw_bullet_speed) {
    if (!std::isfinite(raw_bullet_speed) || raw_bullet_speed <= 0.0) {
        return;
    }

    if (!bullet_speed_filter_initialized_) {
        bullet_speed = raw_bullet_speed;
        bullet_speed_filter_initialized_ = true;
    } else {
        bullet_speed += bullet_speed_filter_alpha_ * (raw_bullet_speed - bullet_speed);
    }

    if (trajectory_compensator_) {
        trajectory_compensator_->velocity = bullet_speed;
    }
}

/**
  @brief 获得子弹速度
*/
double Solver::get_bullet_speed() {
    return bullet_speed;
}

/**
  @brief 预测装甲板dt后的向量
*/
State Solver::state_predict(const State& state, double dt) {
    if (!enable_motion_prediction_) {
        return state;
    }

    State state_predict = state;
    state_predict(0) += state_predict(1) * dt;
    state_predict(2) += state_predict(3) * dt;
    state_predict(4) += state_predict(5) * dt;
    state_predict(6) = reduced_angle(state_predict(6) + state_predict(7) * dt);

    return state_predict;
}

/**
  @brief 获得一定角度内的装甲板
*/
std::vector<State> Solver::state_vec_to_direct_state_vec(
    const std::vector<State>& state_vec,
    const double& max_orientation_angle,
    const Eigen::Vector3d& gimbal_rpy
) {
    std::vector<State> direct_vec;
    for (const auto& state: state_vec) {
        double current_max_orientation_angle = max_orientation_angle;
        // 若当前装甲板与上一帧锁定装甲板足够接近，则放宽直接击打角度阈值，减少切板抖动
        if (std::abs(reduced_angle(state(6) - last_aim_yaw_)) < M_PI / 4.0) {
            current_max_orientation_angle += 15.0 * M_PI / 180.0;
        }
        // 如果装甲板相对于相机的角度在允许范围内
        if (std::abs(reduced_angle(state(6) - gimbal_rpy[2])) <= current_max_orientation_angle) {
            direct_vec.push_back(state);
        }
    }
    return direct_vec;
}

/**
  @brief 获得装甲板的xyz
*/
Eigen::Vector3d Solver::state_to_armor_pos(const State& state) {
    return Eigen::Vector3d { state[0] - (std::cos(state[6]) * state[8]),
                             state[2] - (std::sin(state[6]) * state[8]),
                             state[4] };
}

/**
  @brief 将xyz坐标转化成ypd
*/
YpdCoord Solver::xyz_to_ypd(const Eigen::Vector3d& xyz) {
    YpdCoord ypd;
    ypd.yaw = std::atan2(xyz(1, 0), xyz(0, 0));
    ypd.pitch = std::atan2(xyz(2, 0), std::sqrt(xyz(0, 0) * xyz(0, 0) + xyz(1, 0) * xyz(1, 0)));
    ypd.distance = xyz.norm();

    return ypd;
}

/**
  @brief 将xyz转成ypd，带手动角度补偿
*/
YpdCoord Solver::aim_xyz_to_aim_ypd(const Eigen::Vector3d& xyz) {
    YpdCoord aim_ypd = xyz_to_ypd(xyz);

    auto angle_offset = manual_compensator_->angleHardCorrect(xyz.head(2).norm(), xyz.z());
    double pitch_offset = angle_offset[0] * M_PI / 180;
    double yaw_offset = angle_offset[1] * M_PI / 180;

    // NEED DEBUG
    return { reduced_angle(aim_ypd.yaw + yaw_offset),
             aim_ypd.pitch + pitch_offset,
             aim_ypd.distance };
}

/**
  @brief 获得目标yaw pitch 与当前云台的差值代价cost
*/
double Solver::aim_swing_cost(const YpdCoord& aim_ypd, const Eigen::Vector3d& gimbal_rpy) {
    // NEED DEBUG
    return Eigen::Vector2d(
               reduced_angle(aim_ypd.yaw - gimbal_rpy[2]),
               aim_ypd.pitch - gimbal_rpy[1]
    )
        .norm();
}

/**
  @brief 得到最小代价击打的装甲板的ypd，最终击打位置
*/
std::pair<YpdCoord, State> Solver::choose_direct_aim(
    const std::vector<State>& direct_state_vec,
    const Eigen::Vector3d& gimbal_rpy
) {
    // 获得最小摆动代价的装甲板
    State chosen_state = direct_state_vec[0];
    YpdCoord chosen_aim_ypd;
    double min_swing_cost = DBL_MAX;
    for (const auto& direct: direct_state_vec) {
        const Eigen::Vector3d target_pos = state_to_armor_pos(direct);
        const auto aim_ypd = target_pos_to_aim_ypd(target_pos);
        const double swing_cost = aim_swing_cost(aim_ypd, gimbal_rpy);
        if (swing_cost < min_swing_cost) {
            chosen_state = direct;
            min_swing_cost = swing_cost;
            chosen_aim_ypd = aim_ypd;
        }
    }

    return { chosen_aim_ypd, chosen_state };
}

/**
  @brief 得到经过弹道解算后的ypd,有手动补偿角度
*/
YpdCoord Solver::target_pos_to_aim_ypd(const Eigen::Vector3d& xyz) {
    Eigen::Vector3d aim_xyz = this->target_pos_to_shoot_param(xyz).aim_xyz;
    return this->aim_xyz_to_aim_ypd(aim_xyz);
}

/**
  @brief 角速度较大时调用，到指定位置等待
*/
std::pair<YpdCoord, State> Solver::choose_indirect_aim(
    const std::vector<State>& indirect_state_vec,
    const Eigen::Vector3d& gimbal_rpy,
    double max_orientation_angle_,
    double max_out_error_
) {
    // 角速度 v-yaw
    const double w = indirect_state_vec[0][7];
    // 等待装甲板的角度yaw，向哪个方向等待
    const double zn_to_wait = w > 0.0 ? -max_orientation_angle_ : +max_orientation_angle_;
    // 目标装甲板进入 max_orientation_angle 范围所需角度
    double min_score = DBL_MAX;
    double actual_min_wait = DBL_MAX;
    State indirect_aim_state = indirect_state_vec[0];

    // 找到等待击打的装甲板
    for (const auto& state: indirect_state_vec) {
        const double max_out_angle =
            this->shooting_range_w_ / 2.0 * max_out_error_ / state[8]; // 近似角度误差
        const double zn_to_armor =
            reduced_angle(state(6) - gimbal_rpy[2]); // 装甲板当前yaw,相对于相机
        // 值域 (-max_out_angle, M_PI - max_out_angle)
        const double armor_to_wait =
            reduced_angle(
                (w > 0.0 ? zn_to_wait - zn_to_armor : zn_to_armor - zn_to_wait) - M_PI
                + max_out_angle
            )
            + M_PI - max_out_angle;
        double score = armor_to_wait;
        // 对上一帧已锁定附近的装甲板给予粘滞优待，降低临界切板频率
        if (std::abs(reduced_angle(state(6) - last_aim_yaw_)) < M_PI / 4.0) {
            score -= 0.5;
        }
        if (score < min_score) {
            min_score = score;
            actual_min_wait = armor_to_wait;
            indirect_aim_state = state;
        }
    }

    // 装甲板进入 max_orientation_angle 所需时间 dt
    const double time_to_emerge = actual_min_wait / std::abs(w);
    auto aim_state = state_predict(indirect_aim_state, time_to_emerge);
    const Eigen::Vector3d target_pos = state_to_armor_pos(aim_state);
    const auto aim_ypd = target_pos_to_aim_ypd(target_pos);

    return { aim_ypd, indirect_aim_state };
}

/**
  @brief 数据类型转化
*/
ShootState Solver::YpdCoord_to_ShootState(const YpdCoord& ypd, const ShootMode& shoot) {
    return ShootState { ypd.yaw, ypd.pitch, ypd.distance, shoot };
}
YpdCoord Solver::ShootState_to_YpdCoord(const ShootState& shoot_state) {
    return YpdCoord { shoot_state.yaw, shoot_state.pitch, shoot_state.distance };
}

/**
  @brief 获得最终击打信息
  @param state_vec [IN] 预测后的装甲板状态
  @param max_orientation_angle [IN] 允许直接击打的最大装甲板yaw角度
  @param max_out_error [IN] 允许的最大误差率
  @param allow_indirect [IN] 是否允许间接击打
    @param gimbal_rpy [IN] 当前云台姿态
  @return 最终击打信息
*/
ShootState Solver::state_vec_to_aim(
    const std::vector<State>& state_vec,
    const double& max_orientation_angle,
    const double& max_out_error,
    const bool& allow_indirect,
    const Eigen::Vector3d& gimbal_rpy
) {
    const std::vector<State> direct_vec =
        state_vec_to_direct_state_vec(state_vec, max_orientation_angle, gimbal_rpy);
    std::pair<YpdCoord, State> target_ypd_and_state;
    if (direct_vec.empty()) {
        if (!allow_indirect) {
            //debug
            FYT_DEBUG("armor_solver", "no direct armor, no indirect allowed, idle");
            return ShootState { 0.0, 0.0, -1.0, ShootMode::IDLE, State {} };
        }

        target_ypd_and_state =
            choose_indirect_aim(state_vec, gimbal_rpy, max_orientation_angle, max_out_error);
    } else {
        target_ypd_and_state = choose_direct_aim(direct_vec, gimbal_rpy);
    }

    // 赋值
    auto target_aim = ShootState {};
    target_aim.shoot = ShootMode::TRACKING;
    target_aim.state = target_ypd_and_state.second;
    target_aim.aim_ypd = target_ypd_and_state.first;
    auto target_pos = state_to_armor_pos(target_ypd_and_state.second);
    auto armor_v = state_to_armor_v(target_ypd_and_state.second);
    target_aim.aim_ypd_v = get_gimbal_ypd_v(target_pos, armor_v, gimbal_rpy); // 计算ypd_v
    // 记录本帧最终锁定的装甲板世界 yaw，供下一帧切板滞回使用
    last_aim_yaw_ = target_aim.state(6);
    return target_aim;
}

/**
    @brief 判断是否超过允许误差率
    @return 在误差以内返回 true，否则返回 false
*/
bool Solver::aim_error_exceeded(
    const YpdCoord& ypd,
    const Eigen::Vector3d& gimbal_rpy,
    const double& error_rate,
    const double& yaw_inclined,
    const double& pitch_inclined
) {
    double yaw_diff = std::abs(reduced_angle(ypd.yaw - gimbal_rpy[2]));
    double pitch_diff = std::abs(ypd.pitch - gimbal_rpy[1]);

    // 当前距离目标的diff 比 允许开火diff
    if (get_termination_dis(ypd.distance, yaw_diff)
        > shooting_range_w_ / 2. * std::cos(yaw_inclined) * error_rate)
    {
        return false;
    }
    if (get_termination_dis(ypd.distance, pitch_diff)
        > shooting_range_h_ / 2. * std::cos(pitch_inclined) * error_rate)
    {
        // 对方朝上 15 度，我朝下 15 度，这是刚好对准的
        return false;
    }

    return true;
}

/**
    @brief 判断是否超过允许误差率
    @return 在误差以内返回 true，否则返回 false
*/
bool Solver::aim_error_exceeded(
    const YpdCoord& real_ypd,
    const YpdCoord& ideal_ypd,
    const Eigen::Vector3d& gimbal_rpy,
    const double& error_rate,
    const double& yaw_inclined,
    const double& pitch_inclined
) {
    double yaw_diff = std::abs(reduced_angle(real_ypd.yaw - ideal_ypd.yaw));
    double pitch_diff = std::abs(real_ypd.pitch - ideal_ypd.pitch);

    // 当前距离目标的diff 比 允许开火diff
    if (get_termination_dis(real_ypd.distance, yaw_diff)
        > shooting_range_w_ / 2. * std::cos(yaw_inclined) * error_rate)
    {
        return false;
    }
    if (get_termination_dis(real_ypd.distance, pitch_diff)
        > shooting_range_h_ / 2. * std::cos(pitch_inclined) * error_rate)
    {
        // 对方朝上 15 度，我朝下 15 度，这是刚好对准的
        return false;
    }

    return true;
}

/**
 * @brief 计算是否建议开火
 * 
 * @param real_ypd 实际要瞄准的位置
 * @param ideal_ypd 加上开火延迟后实际打到的地方
 * @param gimbal_rpy 当前云台姿态
 * @param error_rate 开火误差
 * @param yaw_inclined 装甲板相对于云台yaw
 * @param pitch_inclined 装甲板pitch
 * @return ShootMode 开火建议
 */
ShootMode Solver::isSuggestFire(
    const YpdCoord& real_ypd,
    const YpdCoord& ideal_ypd,
    const Eigen::Vector3d& gimbal_rpy,
    const double& error_rate,
    const double& yaw_inclined,
    const double& pitch_inclined
) {
    if (aim_error_exceeded(real_ypd, gimbal_rpy, error_rate, yaw_inclined, pitch_inclined)
        && aim_error_exceeded(
            real_ypd,
            ideal_ypd,
            gimbal_rpy,
            error_rate,
            yaw_inclined,
            pitch_inclined
        ))
    {
        return ShootMode::SHOOT_NOW;
    } else {
        return ShootMode::TRACKING;
    }
}

bool Solver::aim_cmp(const ShootState& aim_a, const ShootState& aim_b) {
    if ((aim_a.shoot == ShootMode::IDLE) != (aim_b.shoot == ShootMode::IDLE)) {
        return static_cast<int>(aim_a.shoot == ShootMode::IDLE)
            < static_cast<int>(aim_b.shoot == ShootMode::IDLE); // false 优先
    }
    return this->aim_swing_cost(ShootState_to_YpdCoord(aim_a), this->gimbal_rpy_)
        < this->aim_swing_cost(ShootState_to_YpdCoord(aim_b), this->gimbal_rpy_);
}

// 这个用来计算 ypd_v 的
Eigen::Vector3d Solver::state_to_armor_v(const State& state) {
    const Eigen::Vector3d center_v = { state[1], state[3], state[5] };
    const Eigen::Vector2d radius_norm = rotate({ 1.0, 0.0 }, state[6]);
    const Eigen::Vector2d theta_norm = rotate(radius_norm, M_PI / 2.0);
    const Eigen::Vector2d theta_v = state[7] * theta_norm * state[8];
    return { center_v[0] - theta_v[0], center_v[1] - theta_v[1], center_v[2] };
}

/**
 * @brief 将 ROS 坐标系下的速度向量转换为球坐标系/YPD坐标系下的变化率
 *  输入的是基于云台坐标系的位置、速度，由于 odom 和 gimbal 重合，这里直接用 odom 系下的了
 * @param xyz_i 装甲板xyz
 * @param xyz_v_i 装甲板xyz_v
 * @param gimbal_rpy 云台姿态
 * @return YpdCoord 
 */
YpdCoord Solver::get_gimbal_ypd_v(
    const Eigen::Vector3d& xyz_i,
    const Eigen::Vector3d& xyz_v_i,
    const Eigen::Vector3d& gimbal_rpy
) {
    // 初始化 1维 Jet。实部为坐标值，导数部（.v[0]）为速度值
    // 相当于定义了变量随时间的一阶泰勒展开： x(t) = x + vx * dt
    ceres::Jet<double, 1> x(xyz_i.x());
    x.v[0] = xyz_v_i.x();
    ceres::Jet<double, 1> y(xyz_i.y());
    y.v[0] = xyz_v_i.y();
    ceres::Jet<double, 1> z(xyz_i.z());
    z.v[0] = xyz_v_i.z();

    YpdCoord ypd_v;

    // --- Yaw 的自动求导 ---
    ceres::Jet<double, 1> yaw = ceres::atan2(y, x);
    ypd_v.yaw = yaw.v[0];

    // --- Pitch 的自动求导 ---
    ceres::Jet<double, 1> xy_norm = ceres::sqrt(x * x + y * y);
    ceres::Jet<double, 1> pitch = ceres::atan2(z, xy_norm);
    ypd_v.pitch = pitch.v[0];

    // --- Distance 的自动求导 ---
    ceres::Jet<double, 1> dist = ceres::sqrt(x * x + y * y + z * z);
    ypd_v.distance = dist.v[0]; // 直接提取导数部分

    return ypd_v;
}

} // namespace qd::auto_aim
