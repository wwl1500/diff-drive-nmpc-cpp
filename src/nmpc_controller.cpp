#include "nmpc_controller.hpp"
#include "math_utils.hpp"

#include "acados_solver_diff_drive.h"
#include "acados_c/ocp_nlp_interface.h"

#include <cmath>
#include <chrono>
#include <cstring>
#include <stdexcept>

static constexpr int NX = DIFF_DRIVE_NX;   // 5
static constexpr int NU = DIFF_DRIVE_NU;   // 2
static constexpr int N  = DIFF_DRIVE_N;    // 20
static constexpr int NY = DIFF_DRIVE_NY;   // 8
static constexpr int NYN = DIFF_DRIVE_NYN; // 4

// 阶段残差 yref 的分量（与 generate_acados_solver.py 一致）：
// [sqrt(Q)*px_ref, sqrt(Q)*py_ref, sqrt(QT)*sin(theta_ref), sqrt(QT)*cos(theta_ref),
//  0, 0, 0, 0]
// 终端残差 yref_e 的分量：
// [sqrt(QT)*px_ref, sqrt(QT)*py_ref, sqrt(QTT)*sin(theta_ref), sqrt(QTT)*cos(theta_ref)]
//
// 权重系数（OCP 生成时已嵌入残差函数）：
static constexpr double Q_POS = 20.0;
static constexpr double Q_THETA = 2.0;
static constexpr double Q_TERMINAL_POS = 40.0;
static constexpr double Q_TERMINAL_THETA = 4.0;

NmpcController::NmpcController(const NmpcControllerConfig& config)
    : config_(config)
    , x_warm_(NX * (N + 1), 0.0)
    , u_warm_(NU * N, 0.0) {
    capsule_ = diff_drive_acados_create_capsule();
    if (!capsule_) {
        throw std::runtime_error("failed to create acados capsule");
    }
    int status = diff_drive_acados_create(capsule_);
    if (status != 0) {
        throw std::runtime_error("acados solver creation failed with status " + std::to_string(status));
    }
}

NmpcController::~NmpcController() {
    if (capsule_) {
        diff_drive_acados_free(capsule_);
        diff_drive_acados_free_capsule(capsule_);
    }
}

Control NmpcController::computeControl(
    const State& current_state,
    const std::vector<Reference>& ref_horizon) {
    auto t_start = std::chrono::high_resolution_clock::now();

    setInitialState(current_state);
    setReferences(ref_horizon);
    if (!first_solve_) {
        shiftWarmStart();
    }
    setWarmStart();

    int status = diff_drive_acados_solve(capsule_);

    auto t_end = std::chrono::high_resolution_clock::now();
    double solve_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    stats_.total_solves++;
    stats_.total_solve_time_ms += solve_time_ms;
    stats_.last_solve_time_ms = solve_time_ms;
    if (solve_time_ms > stats_.max_solve_time_ms) {
        stats_.max_solve_time_ms = solve_time_ms;
    }

    if (status == 0) {
        last_solve_ok_ = true;
        // 求解成功，读取 u[0] 和状态轨迹用于下一次 warm-start
        Control u0 = readControl();

        // 保存 warm-start 轨迹
        ocp_nlp_config* nlp_config = diff_drive_acados_get_nlp_config(capsule_);
        ocp_nlp_dims* nlp_dims = diff_drive_acados_get_nlp_dims(capsule_);
        ocp_nlp_out* nlp_out = diff_drive_acados_get_nlp_out(capsule_);

        for (int k = 0; k <= N; ++k) {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "x", &x_warm_[k * NX]);
        }
        for (int k = 0; k < N; ++k) {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "u", &u_warm_[k * NU]);
        }

        last_v_ = u0.v;
        last_omega_ = u0.omega;
        first_solve_ = false;
        return u0;
    }

    // 求解失败，使用 fallback
    last_solve_ok_ = false;
    stats_.failed_solves++;
    Control fb = fallbackControl();
    last_v_ = fb.v;
    last_omega_ = fb.omega;
    first_solve_ = false;
    return fb;
}

void NmpcController::setInitialState(const State& state) {
    ocp_nlp_config* nlp_config = diff_drive_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* nlp_dims = diff_drive_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* nlp_in = diff_drive_acados_get_nlp_in(capsule_);
    ocp_nlp_out* nlp_out = diff_drive_acados_get_nlp_out(capsule_);

    // 增广状态: [px, py, theta, v_prev, omega_prev]
    double x0[NX] = {state.x, state.y, state.theta, last_v_, last_omega_};
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
}

void NmpcController::setReferences(const std::vector<Reference>& ref_horizon) {
    ocp_nlp_config* nlp_config = diff_drive_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* nlp_dims = diff_drive_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* nlp_in = diff_drive_acados_get_nlp_in(capsule_);

    double sqrt_q = std::sqrt(Q_POS);
    double sqrt_qt = std::sqrt(Q_THETA);

    // 阶段 0..N-1：8 维残差参考
    for (int k = 0; k < N; ++k) {
        int idx = std::min(k, static_cast<int>(ref_horizon.size()) - 1);
        const auto& ref = ref_horizon[idx];
        double yref[NY] = {
            sqrt_q * ref.x,
            sqrt_q * ref.y,
            sqrt_qt * std::sin(ref.theta),
            sqrt_qt * std::cos(ref.theta),
            0.0, 0.0, 0.0, 0.0   // v, omega, dv, domega 的参考为 0
        };
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, k, "yref", yref);
    }

    // 终端阶段 N：4 维残差参考
    int idx = std::min(N, static_cast<int>(ref_horizon.size()) - 1);
    const auto& ref = ref_horizon[idx];
    double sqrt_qtp = std::sqrt(Q_TERMINAL_POS);
    double sqrt_qtt = std::sqrt(Q_TERMINAL_THETA);
    double yref_e[NYN] = {
        sqrt_qtp * ref.x,
        sqrt_qtp * ref.y,
        sqrt_qtt * std::sin(ref.theta),
        sqrt_qtt * std::cos(ref.theta)
    };
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
}

void NmpcController::setWarmStart() {
    ocp_nlp_config* nlp_config = diff_drive_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* nlp_dims = diff_drive_acados_get_nlp_dims(capsule_);
    ocp_nlp_out* nlp_out = diff_drive_acados_get_nlp_out(capsule_);
    ocp_nlp_in* nlp_in = diff_drive_acados_get_nlp_in(capsule_);

    for (int k = 0; k <= N; ++k) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "x", &x_warm_[k * NX]);
    }
    for (int k = 0; k < N; ++k) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "u", &u_warm_[k * NU]);
    }
}

void NmpcController::shiftWarmStart() {
    // 状态轨迹：x[1:N+1] -> x[0:N]，最后一个点复制
    for (int k = 0; k < N; ++k) {
        std::memcpy(&x_warm_[k * NX], &x_warm_[(k + 1) * NX], NX * sizeof(double));
    }
    std::memcpy(&x_warm_[N * NX], &x_warm_[(N - 1) * NX], NX * sizeof(double));

    // 控制轨迹：u[1:N] -> u[0:N-1]，最后一个点复制
    for (int k = 0; k < N - 1; ++k) {
        std::memcpy(&u_warm_[k * NU], &u_warm_[(k + 1) * NU], NU * sizeof(double));
    }
    std::memcpy(&u_warm_[(N - 1) * NU], &u_warm_[(N - 2) * NU], NU * sizeof(double));
}

Control NmpcController::readControl() const {
    ocp_nlp_config* nlp_config = diff_drive_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* nlp_dims = diff_drive_acados_get_nlp_dims(capsule_);
    ocp_nlp_out* nlp_out = diff_drive_acados_get_nlp_out(capsule_);

    double u[NU];
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u);

    Control c;
    c.v = math_utils::clamp(u[0], config_.v_min, config_.v_max);
    c.omega = math_utils::clamp(u[1], config_.omega_min, config_.omega_max);
    return c;
}

Control NmpcController::fallbackControl() const {
    // 优先使用 warm-start 中的 u[1]（shift 后），否则用上次控制量
    double v_fb = last_v_;
    double omega_fb = last_omega_;

    if (!first_solve_ && u_warm_.size() >= 2 * NU) {
        v_fb = u_warm_[NU];       // u_warm_[1] 即 u[1]（shift 后是原 u[2]）
        omega_fb = u_warm_[NU + 1];
    }

    Control c;
    c.v = math_utils::clamp(v_fb, config_.v_min, config_.v_max);
    c.omega = math_utils::clamp(omega_fb, config_.omega_min, config_.omega_max);
    return c;
}
