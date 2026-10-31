#ifndef ARMOR_SOLVER_AXIS_MPC_HPP_
#define ARMOR_SOLVER_AXIS_MPC_HPP_

#include <memory>
#include <optional>
#include <vector>
#include <cmath>

namespace qd::auto_aim {
/** @brief 将 degree 角按最短角差展开至 anchor 附近。 */
inline double unwrap_degrees(double angle, double anchor) {
    return anchor + std::remainder(angle - anchor, 360.0);
}

/** @brief 使用 TinyMPC 规划单轴角度闭环指令，公共接口单位为 degree、s。 */
class AxisMpc {
public:
    struct Config {
        double response_time = 0.06;
        double command_weight = 0.05;
        double max_command_lead = 10.0;
        int horizon = 20;
        double dt = 0.01;
        double position_weight = 1.0;
        int max_iterations = 3000;
        double tolerance = 1e-5;
    };
    struct Sample {
        double position; // 发送角，不是预测实际角
        double velocity;
        double acceleration;
    };

    /** @brief 创建固定模型及约束的 TinyMPC 实例，非法配置使 plan 返回空值。 */
    explicit AxisMpc(const Config& config);
    ~AxisMpc();
    AxisMpc(const AxisMpc&) = delete;
    AxisMpc& operator=(const AxisMpc&) = delete;

    /**
     * @brief 求解绝对角度指令，返回的导数字段为零。
     * @param position 实际起始角，degree，yaw 已展开。
     * @param reference 初始点及 horizon 个未来角度，degree。
     * @return 收敛且满足约束的参考；非法输入或求解失败时为空。
     * @note 实例由求解线程独占，每次清空上一轮对偶状态以隔离目标切换。
     */
    std::optional<Sample> plan(double position, const std::vector<double>& reference) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace qd::auto_aim
#endif
