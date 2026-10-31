#include "armor_solver/axis_mpc.hpp"

#include <algorithm>
#include "tiny_api.hpp"

namespace qd::auto_aim {
struct AxisMpc::Impl {
    Config config;
    TinySolver* solver = nullptr;
    bool valid = false;
    explicit Impl(const Config& value) : config(value) {}
    ~Impl() {
        if (solver) {
            delete solver->solution;
            delete solver->cache;
            delete solver->settings;
            delete solver->work;
            delete solver;
        }
    }
};

AxisMpc::AxisMpc(const Config& config) : impl_(std::make_unique<Impl>(config)) {
    const auto positive = [](double x) { return std::isfinite(x) && x > 0; };
    const auto& c = config;
    if (c.horizon < 2 || c.horizon > 100 || !positive(c.dt)
        || !positive(c.position_weight) || c.max_iterations < 1 || !positive(c.tolerance)) {
        return;
    }
    if (!positive(c.response_time) || !positive(c.command_weight)
        || !positive(c.max_command_lead)) return;
    const double alpha = -std::expm1(-c.dt / c.response_time);
    // 模型为现有位置闭环的一阶响应，控制量是相对当前姿态的角度指令。
    if (tiny_setup(&impl_->solver, tinyMatrix::Constant(1, 1, 1.0 - alpha),
            tinyMatrix::Constant(1, 1, alpha), tinyVector::Zero(1),
            tinyMatrix::Constant(1, 1, c.position_weight),
            tinyMatrix::Constant(1, 1, c.command_weight),
            1.0, 1, 1, c.horizon + 1, 0) != 0) return;
    auto* solver = impl_->solver;
    if (tiny_set_bound_constraints(solver,
            tinyMatrix::Constant(1, c.horizon + 1, -1e17),
            tinyMatrix::Constant(1, c.horizon + 1, 1e17),
            tinyMatrix::Constant(1, c.horizon, -c.max_command_lead),
            tinyMatrix::Constant(1, c.horizon, c.max_command_lead)) != 0) return;
    solver->settings->max_iter = c.max_iterations;
    solver->settings->abs_pri_tol = c.tolerance;
    solver->settings->abs_dua_tol = c.tolerance;
    impl_->valid = solver->cache->Pinf.allFinite()
        && solver->cache->Kinf.allFinite() && solver->cache->Quu_inv.allFinite();
}

AxisMpc::~AxisMpc() = default;

std::optional<AxisMpc::Sample> AxisMpc::plan(double position, const std::vector<double>& reference) const {
    const auto& c = impl_->config;
    if (!impl_->valid || reference.size() != static_cast<size_t>(c.horizon + 1)
        || !std::isfinite(position)
        || !std::all_of(reference.begin(), reference.end(), [](double x) {
            return std::isfinite(x);
        })) return std::nullopt;
    auto* solver = impl_->solver;
    auto* w = solver->work;
    // 以当前姿态为局部原点，跨 yaw 接缝由调用方连续展开。
    w->x.setZero();
    w->u.setZero();
    w->v.setZero();
    w->vnew.setZero();
    w->z.setZero();
    w->znew.setZero();
    w->g.setZero();
    w->y.setZero();
    w->p.setZero();
    w->d.setZero();
    for (int k = 0; k <= c.horizon; ++k) {
        w->Xref(0, k) = reference[k] - position;
        if (k < c.horizon) w->Uref(0, k) = reference[k] - position;
    }
    if (!w->Xref.allFinite() || !w->Uref.allFinite()) return std::nullopt;
    update_linear_cost(solver);
    tiny_set_x0(solver, tinyVector::Zero(1));
    if (tiny_solve(solver) != 0 || !solver->solution->solved
        || !w->x.allFinite() || !w->u.allFinite()
        || w->u.cwiseAbs().maxCoeff() > c.max_command_lead + c.tolerance)
        return std::nullopt;
    const double command = position + std::clamp(w->u(0, 0),
        -c.max_command_lead, c.max_command_lead);
    if (!std::isfinite(command)) return std::nullopt;
    // 电控仅使用角度；不把模型内部导数当成可执行前馈。
    return Sample{command, 0.0, 0.0};
}
} // namespace qd::auto_aim
