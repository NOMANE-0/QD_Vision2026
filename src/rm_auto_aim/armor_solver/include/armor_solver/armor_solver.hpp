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
#include <cstdint>
#include <memory>
#include <string>
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

struct ShootState {
    double yaw;
    double pitch;
    double distance;
    // 上面几个不起主要作用

    ShootMode shoot; // 开火建议
    State state; // 目标装甲板，整车模型才有
    YpdCoord aim_ypd; // 电控要瞄准的位置
    YpdCoord aim_ypd_v; // 目标在云台下的球坐标变化率
};

struct ShootParam {
    double bullet_speed = 0.;
    double pitch_angle = 0.;
    Eigen::Vector3d aim_xyz =
        Eigen::Vector3d::Zero(); // z 改成 pitch_angle 算出的，其他同 target_xyz
    Eigen::Vector3d target_xyz = Eigen::Vector3d::Zero();
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

    // 可视化用的
    std::vector<std::pair<double, double>> getTrajectory() const noexcept;

    ShootState get_aim(const RosTarget& target_msg, const Eigen::Vector3d& gimbal_rpy);
    ShootState get_aim_positon(const RosTarget& target_msg, const Eigen::Vector3d& gimbal_rpy);

    int get_top_level() const;

    /** @brief 丢失目标时清除旋转模式及连续观测计数。 */
    void reset_top_level() noexcept;

    /**
     * @brief 使用新收到的有效弹速更新滤波状态和弹道模型。
     * @param raw_bullet_speed 原始反馈弹速，单位米每秒，须为有限正数。
     * @note 每条新反馈调用一次；调用方须与解算线程互斥访问。
     */
    void update_bullet_speed(double raw_bullet_speed);

private:
    std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
    std::unique_ptr<ManualCompensator> manual_compensator_;
    std::weak_ptr<rclcpp::Node> node_;
    std::string gimble_frame_;

private:
    /**
     * @brief 用一张新图像的自转速度更新旋转目标模式。
     * @param angular_velocity EKF 自转速度，单位弧度每秒。
     * @param frame_stamp_ns 图像时间戳，单位纳秒；同帧重复解算不累计确认次数。
     * @param target_id 当前跟踪的装甲板编号。
     */
    void update_top_level(
        double angular_velocity,
        std::int64_t frame_stamp_ns,
        const std::string& target_id
    );

    double get_ballistic_flying_time(const Eigen::Vector3d& target_pos);
    /**
     * @brief 根据运动预测开关返回当前状态或命中时刻的预测状态。
     * @param state 当前目标状态，位置为目标参考系中的米，速度为米每秒或弧度每秒。
     * @param base_delay 图像到发送及控制的固定延迟，单位秒。
     * @return 预测开启时返回命中状态；关闭时原样返回当前状态。
     */
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
    YpdCoord target_pos_to_aim_ypd(const Eigen::Vector3d& xyz);

    double aim_swing_cost(const YpdCoord& aim_ypd, const Eigen::Vector3d& gimbal_rpy);

    std::pair<YpdCoord, State> choose_direct_aim(
        const std::vector<State>& direct_state_vec,
        const Eigen::Vector3d& gimbal_rpy
    );
    std::pair<YpdCoord, State> choose_indirect_aim(
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
    int top_activation_frames_ = 4;
    int top_activation_count_ = 0;
    int top_candidate_sign_ = 0;
    std::int64_t last_top_frame_stamp_ns_ = 0;
    std::string top_target_id_;

    double gravity = 9.8;
    double resistance = 0.001;
    double bullet_speed = 16.0;
    double bullet_speed_filter_alpha_ = 0.2;
    bool bullet_speed_filter_initialized_ = false;

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
    bool enable_motion_prediction_ = true; // 仅控制解算外推，不修改 EKF 位置和速度估计
};

} // namespace qd::auto_aim
#endif // ARMOR_SOLVER_SOLVER_HPP_
