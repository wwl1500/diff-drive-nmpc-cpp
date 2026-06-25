#pragma once

#include "common_types.hpp"

#include <vector>

enum class TrajectoryType {
    Circle,
    Eight,
    Sine,
};

struct ReferenceGeneratorConfig {
    double circle_radius = 2.0;
    double circle_period = 20.0;
    double eight_amplitude = 2.0;
    double eight_period = 20.0;
    double sine_forward_velocity = 0.3;
    double sine_amplitude = 1.0;
    double sine_omega = 0.5;
};

class ReferenceGenerator {
public:
    explicit ReferenceGenerator(const ReferenceGeneratorConfig& config = {});

    Reference getReference(double t, TrajectoryType type) const;
    Reference getCircleReference(double t) const;
    Reference getEightReference(double t) const;
    Reference getSReference(double t) const;

    std::vector<Reference> getHorizonReference(
        double current_time,
        double dt,
        int horizon_steps,
        TrajectoryType type
    ) const;

private:
    ReferenceGeneratorConfig config_;
};
