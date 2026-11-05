#ifndef QD_UTILS_TIME_SYNCHRONIZER_HPP_
#define QD_UTILS_TIME_SYNCHRONIZER_HPP_

#include <algorithm>
#include <cstdint>

namespace qd::utils {

class TimeSynchronizer {
public:
    TimeSynchronizer() {
        reset();
    }

    /**
     * @brief 清除 MCU 计数及 PC 时间映射状态。
     */
    void reset() {
        is_initialized_ = false;
        skew_ = 1.0;
        last_mcu_time_ = 0.0;
        last_pc_time_ = 0.0;
        last_sync_time_ = 0.0;
        expected_pc_time_ = 0.0;
        
        last_raw_mcu_time_ = 0;
        raw_mcu_time_accumulator_ = 0;
    }

    /**
     * @brief 根据按序 MCU 计数和 PC 接收时间估计共同时间基准。
     * @param raw_mcu_timestamp 单片机发来的原始 32 位时间戳
     * @param current_pc_time_sec 串口刚收到该数据时 PC 系统的绝对时间 (秒)
     * @param is_timestamp_us MCU 时间戳的单位 (true=微秒, false=毫秒)
     * @return 估计的 PC 时钟时间，单位秒；不包含姿态年龄校正。
     * @note 串口帧按发送顺序输入；计数后退视为 MCU 重启并重建锚点，正常回绕保持连续。
     */
    double sync(uint32_t raw_mcu_timestamp, double current_pc_time_sec, bool is_timestamp_us = false) {
        // 1. 用无符号差处理计数回绕，并检测重复时间戳及 MCU 重启。
        if (!is_initialized_) {
            raw_mcu_time_accumulator_ = raw_mcu_timestamp;
            last_raw_mcu_time_ = raw_mcu_timestamp;
        } else {
            int32_t diff = static_cast<int32_t>(raw_mcu_timestamp - last_raw_mcu_time_);
            if (diff == 0) {
                return expected_pc_time_;
            }
            if (diff < 0) {
                reset();
                return sync(raw_mcu_timestamp, current_pc_time_sec, is_timestamp_us);
            }
            raw_mcu_time_accumulator_ += static_cast<uint32_t>(diff);
            last_raw_mcu_time_ = raw_mcu_timestamp;
        }

        // 2. 将离散的 MCU 时间重构为连续递增的绝对时间 (秒)
        double time_scale = is_timestamp_us ? 1e-6 : 1e-3;
        double current_mcu_time = raw_mcu_time_accumulator_ * time_scale;

        // 3. 首次启动：初始化基准锚点
        if (!is_initialized_) {
            last_mcu_time_ = current_mcu_time;
            last_pc_time_ = current_pc_time_sec;
            last_sync_time_ = current_pc_time_sec;
            expected_pc_time_ = current_pc_time_sec;
            is_initialized_ = true;
            return current_pc_time_sec;
        }

        double mcu_delta = current_mcu_time - last_mcu_time_;
        // 防御性逻辑：MCU 时钟异常停止或乱序包
        if (mcu_delta <= 0) {
            return expected_pc_time_; // 保持上一次的预测
        }

        // 4. 根据当前锁相环追踪的偏斜率(skew) 预测到达时间
        double expected_pc_time = last_pc_time_ + (mcu_delta * skew_);
        expected_pc_time_ = expected_pc_time;

        // 5. 计算系统真实流逝时间和抖动 Jitter
        double dt = current_pc_time_sec - last_sync_time_;
        if (dt <= 0.0) dt = 0.001; // 防止除零和极小时间
        last_sync_time_ = current_pc_time_sec;

        double jitter = current_pc_time_sec - expected_pc_time;

        // 6. 核心：基于时间积分的凸包下边界追踪算法 (解耦了通信频率)
        if (jitter < 0) {
            // 【极小值网络包】包到达得比我们预想的还要快！
            // 说明它没有遇到任何 USB 阻塞。我们立刻刷新锚点，把它作为新的“零延迟”基准
            last_mcu_time_ = current_mcu_time;
            last_pc_time_ = current_pc_time_sec;
            
            // 采用动态增益：根据超出预期多少（jitter），平滑修正斜率，引入 dt 防止高频过调
            skew_ += jitter * 0.05 * dt; 
            
            // 当前包的到达时间即为最真实的同步时间
            expected_pc_time = current_pc_time_sec;
            expected_pc_time_ = current_pc_time_sec;
        } else {
            // 【普通延迟包】经历了 Linux 内核调度或 USB 排队 (Jitter >= 0)
            // 此时绝不能修改锚点。
            // 同时施加一个基于真实时间 (dt) 的正向偏压，以对抗长期的晶振温漂反转
            // 固定斜率：相当于每过 1 秒 skew 增加 1e-5，这样 200Hz 和 1kHz 收敛速度完全一致
            skew_ += 1e-5 * dt; 
        }

        // 7. 物理边界截断：限制时钟偏斜在 ±500 ppm 范围内 (常规晶振的极限物理误差)
        skew_ = std::clamp(skew_, 0.9995, 1.0005);

        return expected_pc_time;
    }

private:
    bool is_initialized_;
    
    // 凸包追踪器核心状态
    double skew_;             // 频率追踪比率 (PC晶振频率 / MCU晶振频率)
    double last_mcu_time_;    // 最新锚点：MCU 侧的时间
    double last_pc_time_;     // 最新锚点：对应的无延迟 PC 时间
    double last_sync_time_;   // 上一次调用 sync 的 PC 时间 (用于 dt 计算)
    double expected_pc_time_; // 上一次的期待时间 (用于异常包处理)

    // 溢出处理状态
    uint32_t last_raw_mcu_time_;
    uint64_t raw_mcu_time_accumulator_;
};

} // namespace qd::utils

#endif // QD_UTILS_TIME_SYNCHRONIZER_HPP_
