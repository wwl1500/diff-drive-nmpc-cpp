#pragma once

#include "controller_base.hpp"

#include <vector>

struct diff_drive_solver_capsule;

struct NmpcControllerConfig {
    double dt = 0.05;
    int horizon_steps = 20;

    // 控制约束（必须与 OCP 生成时一致）
    double v_min = -0.5;
    double v_max = 1.0;
    double omega_min = -1.5;
    double omega_max = 1.5;
};

struct SolveStats {
    int total_solves = 0;
    int failed_solves = 0;
    double last_solve_time_ms = 0.0;
    double total_solve_time_ms = 0.0;
    double max_solve_time_ms = 0.0;

    double average_solve_time_ms() const {
        return total_solves > 0 ? total_solve_time_ms / total_solves : 0.0;
    }
};

class NmpcController : public ControllerBase {
public:
    explicit NmpcController(const NmpcControllerConfig& config = {});
    ~NmpcController() override;

    NmpcController(const NmpcController&) = delete;
    NmpcController& operator=(const NmpcController&) = delete;

    Control computeControl(
        const State& current_state,
        const std::vector<Reference>& ref_horizon
    ) override;

    const SolveStats& solveStats() const { return stats_; }
    double lastSolveTimeMs() const { return stats_.last_solve_time_ms; }
    bool lastSolveOk() const { return last_solve_ok_; }

private:
    void setInitialState(const State& state);
    void setReferences(const std::vector<Reference>& ref_horizon);
    void setWarmStart();
    void shiftWarmStart();
    Control readControl() const;
    Control fallbackControl() const;

    NmpcControllerConfig config_;
    diff_drive_solver_capsule* capsule_;

    // warm-start 存储：x_warm_[k*nx + i], u_warm_[k*nu + i]
    std::vector<double> x_warm_;   // nx * (N+1)
    std::vector<double> u_warm_;   // nu * N

    double last_v_ = 0.0;
    double last_omega_ = 0.0;
    bool first_solve_ = true;
    bool last_solve_ok_ = false;

    SolveStats stats_;
};
