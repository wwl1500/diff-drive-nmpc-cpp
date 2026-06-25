#include "pure_pursuit_controller.hpp"

#include "math_utils.hpp"

#include <cmath>
#include <stdexcept>

PurePursuitController::PurePursuitController(const PurePursuitConfig& config)
    : config_(config) {
    if (config_.lookahead_distance <= 0.0) {
        throw std::invalid_argument("lookahead_distance must be positive");
    }
    if (config_.v_min > config_.v_max || config_.omega_min > config_.omega_max) {
        throw std::invalid_argument("control limits are invalid");
    }
}

Control PurePursuitController::computeControl(
    const State& current_state,
    const std::vector<Reference>& ref_horizon
) {
    if (ref_horizon.empty()) {
        return Control{0.0, 0.0};
    }

    const Reference target = selectTarget(current_state, ref_horizon);
    const double heading_to_target = std::atan2(target.y - current_state.y, target.x - current_state.x);
    const double alpha = math_utils::wrapToPi(heading_to_target - current_state.theta);

    return Control{
        math_utils::clamp(config_.nominal_velocity, config_.v_min, config_.v_max),
        math_utils::clamp(config_.k_alpha * alpha, config_.omega_min, config_.omega_max),
    };
}

Reference PurePursuitController::selectTarget(
    const State& current_state,
    const std::vector<Reference>& ref_horizon
) const {
    for (const auto& reference : ref_horizon) {
        const double distance = math_utils::distance2D(current_state.x, current_state.y, reference.x, reference.y);
        if (distance >= config_.lookahead_distance) {
            return reference;
        }
    }

    return ref_horizon.back();
}
