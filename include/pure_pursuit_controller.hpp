#pragma once

#include "controller_base.hpp"

struct PurePursuitConfig {
    double lookahead_distance = 0.5;
    double nominal_velocity = 0.4;
    double k_alpha = 2.0;
    double v_min = -0.5;
    double v_max = 1.0;
    double omega_min = -1.5;
    double omega_max = 1.5;
};

class PurePursuitController : public ControllerBase {
public:
    explicit PurePursuitController(const PurePursuitConfig& config = {});

    Control computeControl(
        const State& current_state,
        const std::vector<Reference>& ref_horizon
    ) override;

private:
    Reference selectTarget(
        const State& current_state,
        const std::vector<Reference>& ref_horizon
    ) const;

    PurePursuitConfig config_;
};
