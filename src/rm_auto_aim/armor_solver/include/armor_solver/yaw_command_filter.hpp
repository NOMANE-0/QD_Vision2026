#ifndef ARMOR_SOLVER_YAW_COMMAND_FILTER_HPP_
#define ARMOR_SOLVER_YAW_COMMAND_FILTER_HPP_

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace qd::auto_aim {

/**
 * @brief 按图像帧平滑同一目标的云台 yaw 指令，并抑制微小角度抖动。
 * @note 首帧和目标切换时直接采用新角度；调用方负责串行访问。
 */
class YawCommandFilter {
public:
    /**
     * @brief 设置每帧平滑系数和保持角度的死区。
     * @param alpha 每张新图像的更新比例，范围 (0, 1]。
     * @param deadband_deg 保持上次指令的最大角度差，单位度。
     */
    explicit YawCommandFilter(double alpha = 0.25, double deadband_deg = 0.15):
        alpha_(alpha > 0.0 && alpha <= 1.0 ? alpha : 0.25),
        deadband_deg_(deadband_deg >= 0.0 ? deadband_deg : 0.15) {}

    /**
     * @brief 返回本次可下发的 yaw 角度。
     * @param yaw_deg 原始指令角度，单位度，必须是有限值。
     * @param frame_stamp_ns 目标观测的图像时间戳，单位纳秒。
     * @param target_id 跟踪目标编号，用于目标切换时重置滤波。
     * @return 经过平滑和死区处理的 yaw 角度，单位度。
     */
    double update(double yaw_deg, std::int64_t frame_stamp_ns, const std::string& target_id) {
        if (!filtered_yaw_deg_ || target_id != target_id_) {
            filtered_yaw_deg_ = yaw_deg;
            last_frame_stamp_ns_ = frame_stamp_ns;
            target_id_ = target_id;
            return yaw_deg;
        }

        if (last_frame_stamp_ns_ == frame_stamp_ns) {
            return *filtered_yaw_deg_;
        }
        last_frame_stamp_ns_ = frame_stamp_ns;

        const double delta_deg = std::remainder(yaw_deg - *filtered_yaw_deg_, 360.0);
        if (std::abs(delta_deg) > deadband_deg_) {
            filtered_yaw_deg_ = std::remainder(*filtered_yaw_deg_ + alpha_ * delta_deg, 360.0);
        }
        return *filtered_yaw_deg_;
    }

    /** @brief 跟踪丢失或停止检测时清除上一目标的指令。 */
    void reset() {
        filtered_yaw_deg_.reset();
        last_frame_stamp_ns_.reset();
        target_id_.clear();
    }

private:
    double alpha_;
    double deadband_deg_;
    std::optional<double> filtered_yaw_deg_;
    std::optional<std::int64_t> last_frame_stamp_ns_;
    std::string target_id_;
};

} // namespace qd::auto_aim

#endif // ARMOR_SOLVER_YAW_COMMAND_FILTER_HPP_
