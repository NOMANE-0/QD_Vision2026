#ifndef ARMOR_SOLVER_ARMOR_MEASUREMENT_GATE_HPP_
#define ARMOR_SOLVER_ARMOR_MEASUREMENT_GATE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include "rm_utils/math/math.hpp"

namespace qd::auto_aim {

/**
 * @brief 判断观测高度是否接近当前模型预测的任一装甲板高度。
 * @param observed_z_m 装甲板在 odom 坐标系中的观测高度，单位米。
 * @param predicted_z_m 各装甲板在 odom 坐标系中的预测高度，单位米。
 * @param max_residual_m 允许的最大高度残差，单位米。
 * @return 至少有一个板位的高度残差未超过阈值时返回 true。
 */
inline bool is_armor_height_consistent(
    double observed_z_m,
    const std::vector<double>& predicted_z_m,
    double max_residual_m
) {
    if (!std::isfinite(observed_z_m) || max_residual_m < 0.0) {
        return false;
    }
    return std::any_of(predicted_z_m.begin(), predicted_z_m.end(), [&](double predicted_z) {
        return std::isfinite(predicted_z) && std::abs(observed_z_m - predicted_z) <= max_residual_m;
    });
}

/**
 * @brief 判断同一位置附近的装甲板朝向是否发生不可信的单帧跳变。
 * @param previous_position 上一帧已匹配装甲板在 odom 下的位置，单位米。
 * @param previous_yaw 上一帧装甲板朝向，单位弧度。
 * @param previous_stamp_ns 上一帧图像时间戳，单位纳秒。
 * @param position 当前装甲板在 odom 下的位置，单位米。
 * @param yaw 当前装甲板朝向，单位弧度。
 * @param stamp_ns 当前图像时间戳，单位纳秒。
 * @return 同位置装甲板的朝向变化超过物理上限时返回 true。
 * @note 位置明显变化或图像间隔过长时允许切板和重新捕获。
 */
inline bool is_implausible_same_armor_orientation(
    const geometry_msgs::msg::Point& previous_position,
    double previous_yaw,
    std::int64_t previous_stamp_ns,
    const geometry_msgs::msg::Point& position,
    double yaw,
    std::int64_t stamp_ns
) {
    constexpr double kSameArmorDistanceM = 0.12;
    constexpr double kMinimumYawChangeRad = 35.0 * M_PI / 180.0;
    constexpr double kMaximumYawRateRadPerSec = 800.0 * M_PI / 180.0;
    constexpr double kMaximumFrameGapSec = 0.15;

    const double dt = static_cast<double>(stamp_ns - previous_stamp_ns) * 1e-9;
    if (dt <= 0.0 || dt > kMaximumFrameGapSec) {
        return false;
    }

    const double dx = position.x - previous_position.x;
    const double dy = position.y - previous_position.y;
    const double dz = position.z - previous_position.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > kSameArmorDistanceM) {
        return false;
    }

    const double yaw_change = std::abs(reduced_angle(yaw - previous_yaw));
    return yaw_change > std::max(kMinimumYawChangeRad, kMaximumYawRateRadPerSec * dt);
}

} // namespace qd::auto_aim

#endif // ARMOR_SOLVER_ARMOR_MEASUREMENT_GATE_HPP_
