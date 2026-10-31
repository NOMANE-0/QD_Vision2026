#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <cmath>

namespace qd::auto_aim {
void Solver::update_feedback(const rm_interfaces::msg::SerialReceiveData& feedback) {
    if (!std::isfinite(feedback.yaw) || !std::isfinite(feedback.pitch)
        || !std::isfinite(feedback.yaw_actual_vel_deg_s)
        || !std::isfinite(feedback.pitch_actual_vel_deg_s) || feedback.mode > 5) {
        feedback_.reset();
        return;
    }
    if (feedback_ && rclcpp::Time(feedback.header.stamp)
                         < rclcpp::Time(feedback_->header.stamp)) {
        return;
    }
    feedback_ = feedback;
}

bool Solver::plan_mpc(const ShootState& aim, const rm_interfaces::msg::Target& target,
                      const rclcpp::Time& now, rm_interfaces::msg::GimbalCmd& command) {
    command.reference_valid = false;
    command.yaw_vel_deg_s = 0.0f;
    command.pitch_vel_deg_s = 0.0f;
    command.yaw_acc_deg_s2 = 0.0f;
    command.pitch_acc_deg_s2 = 0.0f;
    const auto reject = [&](const char* reason) {
        command.reference_status = reason;
        return false;
    };
    if (!mpc_enabled_) {
        return reject("mpc_disabled");
    }
    if (!feedback_) {
        return reject("no_feedback");
    }
    if (feedback_->mode > 1) {
        return reject("not_armor_mode");
    }
    if (feedback_->header.frame_id != target.header.frame_id) {
        return reject("frame_mismatch");
    }
    if (mpc_config_.horizon < 2 || mpc_config_.horizon > 100
        || !std::isfinite(mpc_config_.dt) || mpc_config_.dt <= 0.0
        || !std::isfinite(feedback_timeout_)
        || feedback_timeout_ <= 0.0 || !std::isfinite(target_timeout_)
        || target_timeout_ <= 0.0 || !std::isfinite(predict2send_delay_)
        || predict2send_delay_ < 0.0 || !std::isfinite(additional_prediction_time)
        || additional_prediction_time < 0.0) {
        return reject("invalid_mpc_config");
    }
    const double feedback_age = (now - rclcpp::Time(feedback_->header.stamp,
                                                    now.get_clock_type())).seconds();
    const double target_age = (now - rclcpp::Time(target.header.stamp,
                                                 now.get_clock_type())).seconds();
    if (feedback_age < 0.0 || feedback_age > feedback_timeout_) {
        return reject("stale_feedback");
    }
    if (target_age < 0.0 || target_age > target_timeout_) {
        return reject("stale_target");
    }
    if (!target.tracking || !std::isfinite(target.v_yaw) || !aim.planning_state_valid
        || !aim.planning_state.allFinite()
        || !std::isfinite(aim.aim_ypd.distance) || aim.aim_ypd.distance <= 0.0
        || !std::isfinite(aim.wait_duration) || aim.wait_duration < 0.0) {
        return reject("invalid_target");
    }

    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    // 用反馈角速度将采样姿态外推到当前时刻。
    const double yaw_initial = feedback_->yaw + feedback_->yaw_actual_vel_deg_s
        * feedback_age;
    const double pitch_initial = feedback_->pitch
        + feedback_->pitch_actual_vel_deg_s
            * feedback_age;
    std::vector<double> yaw_reference;
    std::vector<double> pitch_reference;
    yaw_reference.reserve(mpc_config_.horizon + 1);
    pitch_reference.reserve(mpc_config_.horizon + 1);
    double yaw_anchor = yaw_initial;
    double command_distance = aim.aim_ypd.distance;
    // 全时域固定当前选中的装甲板，不在各预测点重新选板或对切板角度差分。
    for (int k = 0; k <= mpc_config_.horizon; ++k) {
        const double lead = k * mpc_config_.dt + additional_prediction_time;
        auto hit_state = solve_impact_state(aim.planning_state,
                                             target_age + predict2send_delay_ + lead);
        hit_state = state_predict(hit_state, std::max(0.0, aim.wait_duration - lead));
        const auto point = state_to_armor_pos(hit_state);
        if (!point.allFinite() || point.norm() <= 0.0) {
            return reject("invalid_prediction");
        }
        const auto result = target_pos_to_aim_ypd(point);
        if (use_ballistic_compensation_
            && (!result.ballistic_debug || !result.ballistic_debug->ballistic_success)) {
            return reject("ballistic_failed");
        }
        const double yaw = result.aim_ypd.yaw * RAD_TO_DEG;
        const double pitch = -result.aim_ypd.pitch * RAD_TO_DEG;
        if (!std::isfinite(yaw) || !std::isfinite(pitch) || std::abs(pitch) > 327.67) {
            return reject("invalid_reference");
        }
        // 相邻角按最短角差展开，跨 ±180° 不产生 360° 速度尖峰。
        yaw_anchor = unwrap_degrees(yaw, yaw_anchor);
        yaw_reference.push_back(yaw_anchor);
        pitch_reference.push_back(pitch);
        if (k == 0) {
            command_distance = point.norm();
        }
    }
    if (!std::isfinite(command_distance) || command_distance < 0.01) {
        return reject("invalid_target");
    }
    if (direct_angle_command_) {
        command.yaw = std::remainder(yaw_reference[0], 360.0);
        command.pitch = pitch_reference[0];
        command.distance = command_distance;
        command.yaw_diff = std::remainder(command.yaw - feedback_->yaw, 360.0);
        command.pitch_diff = command.pitch - feedback_->pitch;
        command.header.stamp = now + rclcpp::Duration::from_seconds(predict2send_delay_);
        command.reference_valid = true;
        command.reference_status = "valid_direct";
        const double yaw_tolerance = std::atan2(shooting_range_w_ * 0.5
            * max_out_error[top_level], command_distance) * RAD_TO_DEG;
        const double pitch_tolerance = std::atan2(shooting_range_h_ * 0.5
            * max_out_error[top_level], command_distance) * RAD_TO_DEG;
        command.fire_advice = command.fire_advice
            && std::abs(command.yaw_diff) <= yaw_tolerance
            && std::abs(command.pitch_diff) <= pitch_tolerance;
        return true;
    }
    const auto yaw_plan = yaw_mpc_->plan(yaw_initial, yaw_reference);
    if (!yaw_plan) {
        return reject("yaw_mpc_failed");
    }
    const auto pitch_plan = pitch_mpc_->plan(pitch_initial, pitch_reference);
    if (!pitch_plan) {
        return reject("pitch_mpc_failed");
    }
    if (std::abs(pitch_plan->position) > 327.67) {
        return reject("pitch_out_of_wire_range");
    }

    command.yaw = std::remainder(yaw_plan->position, 360.0);
    command.pitch = pitch_plan->position;
    command.yaw_vel_deg_s = static_cast<float>(yaw_plan->velocity);
    command.pitch_vel_deg_s = static_cast<float>(pitch_plan->velocity);
    command.yaw_acc_deg_s2 = static_cast<float>(yaw_plan->acceleration);
    command.pitch_acc_deg_s2 = static_cast<float>(pitch_plan->acceleration);
    if (!std::isfinite(command.yaw_vel_deg_s) || !std::isfinite(command.pitch_vel_deg_s)
        || !std::isfinite(command.yaw_acc_deg_s2) || !std::isfinite(command.pitch_acc_deg_s2)) {
        return reject("wire_float_overflow");
    }
    command.distance = command_distance;
    command.yaw_diff = std::remainder(command.yaw - feedback_->yaw, 360.0);
    command.pitch_diff = command.pitch - feedback_->pitch;
    command.header.stamp = now + rclcpp::Duration::from_seconds(
        predict2send_delay_);
    command.reference_valid = true;
    command.reference_status = "valid";
    const double yaw_goal = yaw_reference[0];
    const double pitch_goal = pitch_reference[0];
    const double yaw_tolerance = std::atan2(shooting_range_w_ * 0.5
        * max_out_error[top_level], command_distance) * RAD_TO_DEG;
    const double pitch_tolerance = std::atan2(shooting_range_h_ * 0.5
        * max_out_error[top_level], command_distance) * RAD_TO_DEG;
    // 角度指令允许领先，开火必须检查实际反馈而不是发送指令是否对准。
    const double fire_yaw = feedback_->yaw;
    const double fire_pitch = feedback_->pitch;
    command.fire_advice = command.fire_advice
        && std::abs(std::remainder(fire_yaw - yaw_goal, 360.0)) <= yaw_tolerance
        && std::abs(fire_pitch - pitch_goal) <= pitch_tolerance;
    return true;
}
} // namespace qd::auto_aim
