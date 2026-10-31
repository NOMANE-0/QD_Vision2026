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

#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_

// std
#include <memory>
#include <optional>
// ros2
#include <angles/angles.h>
#include <tf2_ros/buffer.h>

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>
#include <utility>
// project
#include "armor_solver/bullet_speed_state.hpp"
#include "armor_solver/axis_mpc.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/manual_compensator.hpp"
#include "rm_utils/math/math.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"

namespace qd::auto_aim {

constexpr int N_X = 9;
using State = Eigen::Matrix<double, N_X, 1>; // 用于存放装甲板信息
// state: xc, v_xc, yc, v_yc, za, v_za, orient, v_yaw, r,

/** @brief 保存同一次弹道解算的角度，内部单位为 rad。 */
struct BallisticDebugData {
    double geometric_pitch_rad = 0.0;
    double compensated_pitch_rad = 0.0;
    bool ballistic_enabled = true;
    bool ballistic_success = false;
};

struct ShootState {
    double yaw;
    double pitch;
    double distance;
    // 上面几个不起主要作用

    ShootMode shoot; // 开火建议
    State state = State::Zero(); // 目标装甲板，整车模型才有
    YpdCoord aim_ypd; // 电控要瞄准的位置
    YpdCoord aim_ypd_v; // 目标在云台下的球坐标变化率
    std::optional<BallisticDebugData> ballistic_debug;
    State planning_state = State::Zero(); // 图像参考时刻选中装甲板的状态
    bool planning_state_valid = false;
    double wait_duration = 0.0; // 间接击打等待时间，s
};

struct ShootParam {
    double bullet_speed = 0.;
    double pitch_angle = 0.;
    Eigen::Vector3d aim_xyz =
        Eigen::Vector3d::Zero(); // z 改成 pitch_angle 算出的，其他同 target_xyz
    Eigen::Vector3d target_xyz = Eigen::Vector3d::Zero();
    double geometric_pitch_rad = 0.;
    bool ballistic_success = false;
};

struct AimResult {
    YpdCoord aim_ypd;
    std::optional<BallisticDebugData> ballistic_debug;
    double wait_duration = 0.0;
};

// ros消息转换类
struct RosTarget {
public:
    explicit RosTarget(rm_interfaces::msg::Target& target_msg) {
        tracking_ = target_msg.tracking;
        id_ = target_msg.id;
        armors_num_ = target_msg.armors_num;
        // 整车模型
        center_position_.x() = target_msg.position.x;
        center_position_.y() = target_msg.position.y;
        center_position_.z() = target_msg.position.z;
        center_velocity_.x() = target_msg.velocity.x;
        center_velocity_.y() = target_msg.velocity.y;
        center_velocity_.z() = target_msg.velocity.z;
        // 平移运动
        armor_position_.x() = target_msg.position_armor.x;
        armor_position_.y() = target_msg.position_armor.y;
        armor_position_.z() = target_msg.position_armor.z;
        armor_velocity_.x() = target_msg.velocity_armor.x;
        armor_velocity_.y() = target_msg.velocity_armor.y;
        armor_velocity_.z() = target_msg.velocity_armor.z;
        yaw_ = target_msg.yaw;
        v_yaw_ = target_msg.v_yaw;
        dz_list_.clear();
        for (const auto& dz: target_msg.dz_list) {
            dz_list_.emplace_back(dz);
        }
        radius_list_.clear();
        for (const auto& r: target_msg.radius_list) {
            radius_list_.emplace_back(r);
        }
    }

    bool tracking_;
    std::string id_;
    int armors_num_;
    // 整车模型
    Eigen::Vector3d center_position_;
    Eigen::Vector3d center_velocity_;
    // 平移运动
    Eigen::Vector3d armor_position_;
    Eigen::Vector3d armor_velocity_;
    double yaw_, v_yaw_;
    std::vector<double> dz_list_;
    std::vector<double> radius_list_;
};

// Solver class used to solve the gimbal command from tracked target
class Solver {
public:
    explicit Solver(std::weak_ptr<rclcpp::Node> node);
    // explicit Solver(std::string trajectory_compensator_type, float max_tracking_v_yaw);
    ~Solver() = default;

    // Solve the gimbal command from tracked target
    // Throw: tf2::TransformException if the transform from "odom" to "gimbal_link" is not available
    rm_interfaces::msg::GimbalCmd solve(
        rm_interfaces::msg::Target& target_msg,
        const rclcpp::Time& current_time,
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_
    );

    /**
     * @brief 返回本帧用于生成云台命令的弹道角度。
     * @return 有效瞄准结果的角度数据；无有效结果时为空。
     */
    std::optional<BallisticDebugData> get_ballistic_debug() const noexcept;

    /**
     * @brief 接收一条新的弹速样本并更新弹道模型。
     * @param raw_bullet_speed 电控上报的弹速，单位米每秒。
     * @note 调用方须与 solve() 共用 solver_state_mutex_ 保护求解器状态。
     */
    void update_bullet_speed(double raw_bullet_speed);

    /**
     * @brief 缓存 V2 姿态/角速度及其采样时间，供 MPC 初始化。
     * @param feedback odom 中的绝对姿态及姿态角速度，degree、degree/s，Pitch 向下为正。
     * @note 与 solve() 共用 solver_state_mutex_。
     */
    void update_feedback(const rm_interfaces::msg::SerialReceiveData& feedback);


    // 可视化用的
    std::vector<std::pair<double, double>> getTrajectory() const noexcept;

    ShootState get_aim(const RosTarget& target_msg, const Eigen::Vector3d& gimbal_rpy);
    ShootState get_aim_positon(const RosTarget& target_msg, const Eigen::Vector3d& gimbal_rpy);

    int get_top_level() const;

private:
    /**
     * @brief 从同一块装甲板的未来弹道角生成有限时域参考并求解两轴 MPC。
     * @param aim 当前选板结果及图像时刻的运动状态。
     * @param target 当前跟踪目标，包含坐标系和采样时间。
     * @param now 本轮求解的 ROS 时间。
     * @param command 写入同一规划时刻的角度、速度、加速度及有效位。
     * @return 反馈新鲜、弹道及两轴优化均成功时返回 true。
     */
    bool plan_mpc(const ShootState& aim, const rm_interfaces::msg::Target& target,
                  const rclcpp::Time& now, rm_interfaces::msg::GimbalCmd& command);
    std::unique_ptr<AxisMpc> yaw_mpc_;
    std::unique_ptr<AxisMpc> pitch_mpc_;
    AxisMpc::Config mpc_config_;
    std::optional<rm_interfaces::msg::SerialReceiveData> feedback_;
    bool mpc_enabled_ = true;
    bool direct_angle_command_ = false;
    double feedback_timeout_ = 0.1;
    double target_timeout_ = 0.3;

    std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
    std::unique_ptr<ManualCompensator> manual_compensator_;
    std::weak_ptr<rclcpp::Node> node_;
    std::string gimble_frame_;

private:
    void update_top_level(const double angular_velocity);

    double get_ballistic_flying_time(const Eigen::Vector3d& target_pos);
    State solve_impact_state(const State& state, double base_delay);
    double get_bullet_speed();

    std::vector<State> get_armors_state(const RosTarget& target_msg);

    ShootParam target_pos_to_shoot_param(const Eigen::Vector3d& target_pos);

    State state_predict(const State& state, double dt);

    std::vector<State> state_vec_to_direct_state_vec(
        const std::vector<State>& state_vec,
        const double& max_orientation_angle,
        const Eigen::Vector3d& gimbal_rpy
    );

    Eigen::Vector3d state_to_armor_pos(const State& state);

    YpdCoord xyz_to_ypd(const Eigen::Vector3d& xyz);
    YpdCoord aim_xyz_to_aim_ypd(const Eigen::Vector3d& xyz);
    AimResult target_pos_to_aim_ypd(const Eigen::Vector3d& xyz);

    double aim_swing_cost(const YpdCoord& aim_ypd, const Eigen::Vector3d& gimbal_rpy);

    std::pair<AimResult, State> choose_direct_aim(
        const std::vector<State>& direct_state_vec,
        const Eigen::Vector3d& gimbal_rpy
    );
    std::pair<AimResult, State> choose_indirect_aim(
        const std::vector<State>& indirect_state_vec,
        const Eigen::Vector3d& gimbal_rpy,
        double max_orientation_angle_,
        double max_out_error_
    );

    ShootState YpdCoord_to_ShootState(const YpdCoord& ypd, const ShootMode& shoot);
    YpdCoord ShootState_to_YpdCoord(const ShootState& shoot_state);

    Eigen::Vector3d state_to_armor_v(const State& state);
    YpdCoord get_gimbal_ypd_v(
        const Eigen::Vector3d& xyz_i,
        const Eigen::Vector3d& xyz_v_i,
        const Eigen::Vector3d& gimbal_rpy
    );
    ShootState state_vec_to_aim(
        const std::vector<State>& state_vec,
        const double& max_orientation_angle,
        const double& max_out_error,
        const bool& allow_indirect,
        const Eigen::Vector3d& gimbal_rpy
    );

    bool aim_error_exceeded(
        const YpdCoord& ypd,
        const Eigen::Vector3d& gimbal_rpy,
        const double& error_rate,
        const double& yaw_inclined,
        const double& pitch_inclined
    );
    bool aim_error_exceeded(
        const YpdCoord& real_ypd,
        const YpdCoord& ideal_ypd,
        const Eigen::Vector3d& gimbal_rpy,
        const double& error_rate,
        const double& yaw_inclined,
        const double& pitch_inclined
    );
    ShootMode isSuggestFire(
        const YpdCoord& real_ypd,
        const YpdCoord& ideal_ypd,
        const Eigen::Vector3d& gimbal_rpy,
        const double& error_rate,
        const double& yaw_inclined,
        const double& pitch_inclined
    );

    bool aim_cmp(const ShootState& aim_a, const ShootState& aim_b);

private:
    Eigen::Vector3d gimbal_rpy_; // 当前云台姿态

    int top_level; // 运动状态

    double gravity = 9.8;
    double resistance = 0.001;
    BulletSpeedState bullet_speed_state_;

    double max_orientation_angle[3]; // 允许直接击打范围
    double max_out_error[3]; // 允许的误差
    double activate_threshold[3]; // 进入更高等级的阈值
    double deactivate_threshold[3]; // 进入更低等级的阈值

    double pitch_inclined_; // 装甲板pitch，相对于地面

    double prediction_delay_;
    double predict2send_delay_;
    double controller_delay_;
    double additional_prediction_time;

    double shooting_range_w_;
    double shooting_range_h_;

    double last_aim_yaw_ = 0.0; // 记录上一帧锁定的装甲板世界 yaw，用于切板滞回

    bool use_armor_top_ = false;
    bool use_ballistic_compensation_ = true;
    std::optional<BallisticDebugData> ballistic_debug_;
};

} // namespace qd::auto_aim
#endif // ARMOR_SOLVER_SOLVER_HPP_
