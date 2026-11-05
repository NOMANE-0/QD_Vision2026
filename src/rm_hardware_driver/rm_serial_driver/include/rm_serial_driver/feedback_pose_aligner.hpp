#ifndef RM_SERIAL_DRIVER_FEEDBACK_POSE_ALIGNER_HPP_
#define RM_SERIAL_DRIVER_FEEDBACK_POSE_ALIGNER_HPP_

#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>

namespace qd::serial_driver {

struct YawDataAge {
    bool supported = false;
    bool valid = true;
    uint8_t milliseconds = 0;
};

/**
 * @brief 解析 QD 反馈预留字节中的顶板 yaw 数据年龄。
 * @param value 第 14 字节；最高位标记新协议，低 7 位为毫秒年龄，127 表示无效。
 * @return 年龄是否受支持、是否有效以及毫秒值；旧协议保持原有时间处理。
 */
inline YawDataAge decode_yaw_data_age(uint8_t value) {
    const bool supported = (value & 0x80U) != 0;
    const auto milliseconds = static_cast<uint8_t>(value & 0x7fU);
    return { supported, !supported || milliseconds < 127U, milliseconds };
}

struct FeedbackPose {
    int64_t stamp_ns;
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
};

class FeedbackPoseAligner {
public:
    /**
     * @brief 清除姿态历史及跨包时间状态。
     */
    void reset() {
        history_.clear();
        last_send_mcu_ms_.reset();
        last_yaw_mcu_ms_.reset();
        last_pose_stamp_ns_.reset();
    }

    /**
     * @brief 用 yaw 的接收时刻和该时刻的历史 roll/pitch 生成同一时刻的姿态。
     * @param packet_stamp_ns 已同步且加上固定补偿的发包时间，单位纳秒。
     * @param send_mcu_ms 电控发包时的本机毫秒计数，允许 uint32 回绕。
     * @param roll_deg 当前反馈包的 roll，单位度。
     * @param pitch_deg 当前反馈包的 pitch，抬头为正，单位度。
     * @param yaw_deg 当前反馈包缓存的 yaw，单位度。
     * @param timing_byte 反馈第 14 字节的数据年龄编码。
     * @return 可用于 TF 的姿态；年龄无效、历史不足、重复或过期 yaw 时返回空值。
     * @note 调用者串行访问；只能补偿顶板接收后的等待，未修正传感器内部采样延迟。
     */
    std::optional<FeedbackPose> update(
        int64_t packet_stamp_ns,
        uint32_t send_mcu_ms,
        double roll_deg,
        double pitch_deg,
        double yaw_deg,
        uint8_t timing_byte
    ) {
        if ((last_send_mcu_ms_ && static_cast<int32_t>(send_mcu_ms - *last_send_mcu_ms_) < 0)
            || (!history_.empty() && packet_stamp_ns < history_.back().stamp_ns))
        {
            reset();
        }
        last_send_mcu_ms_ = send_mcu_ms;
        const FeedbackPose current { packet_stamp_ns, roll_deg, pitch_deg, yaw_deg };
        if (!history_.empty() && history_.back().stamp_ns == packet_stamp_ns) {
            history_.back() = current;
        } else {
            history_.push_back(current);
        }
        // 保留年龄上限内的历史，并留下插值所需的前一帧。
        while (history_.size() > 2 && history_[1].stamp_ns < packet_stamp_ns - 250000000LL) {
            history_.pop_front();
        }

        const auto age = decode_yaw_data_age(timing_byte);
        if (!age.valid) {
            return std::nullopt;
        }
        const uint32_t yaw_mcu_ms = send_mcu_ms - age.milliseconds;
        if (age.supported && last_yaw_mcu_ms_
            && static_cast<int32_t>(yaw_mcu_ms - *last_yaw_mcu_ms_) <= 0) {
            return std::nullopt;
        }
        const int64_t pose_stamp_ns =
            packet_stamp_ns - (age.supported ? age.milliseconds * 1000000LL : 0LL);
        if (pose_stamp_ns < history_.front().stamp_ns
            || (last_pose_stamp_ns_ && pose_stamp_ns <= *last_pose_stamp_ns_))
        {
            return std::nullopt;
        }

        FeedbackPose pose { pose_stamp_ns, roll_deg, pitch_deg, yaw_deg };
        if (age.supported && age.milliseconds > 0) {
            for (size_t i = 0; i < history_.size(); ++i) {
                const auto& after = history_[i];
                if (after.stamp_ns < pose_stamp_ns) {
                    continue;
                }
                if (i == 0 || after.stamp_ns == pose_stamp_ns) {
                    pose.roll_deg = after.roll_deg;
                    pose.pitch_deg = after.pitch_deg;
                } else {
                    const auto& before = history_[i - 1];
                    const double weight = static_cast<double>(pose_stamp_ns - before.stamp_ns)
                        / static_cast<double>(after.stamp_ns - before.stamp_ns);
                    pose.roll_deg = before.roll_deg
                        + weight * std::remainder(after.roll_deg - before.roll_deg, 360.0);
                    pose.pitch_deg = before.pitch_deg
                        + weight * std::remainder(after.pitch_deg - before.pitch_deg, 360.0);
                }
                break;
            }
        }
        last_pose_stamp_ns_ = pose_stamp_ns;
        if (age.supported) {
            last_yaw_mcu_ms_ = yaw_mcu_ms;
        } else {
            last_yaw_mcu_ms_.reset();
        }
        return pose;
    }

private:
    std::deque<FeedbackPose> history_;
    std::optional<uint32_t> last_send_mcu_ms_;
    std::optional<uint32_t> last_yaw_mcu_ms_;
    std::optional<int64_t> last_pose_stamp_ns_;
};

} // namespace qd::serial_driver

#endif // RM_SERIAL_DRIVER_FEEDBACK_POSE_ALIGNER_HPP_
