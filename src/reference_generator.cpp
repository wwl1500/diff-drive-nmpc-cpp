#include "reference_generator.hpp"

#include "math_utils.hpp"

#include <cmath>
#include <stdexcept>

ReferenceGenerator::ReferenceGenerator(const ReferenceGeneratorConfig& config)
    : config_(config) {}

Reference ReferenceGenerator::getReference(double t, TrajectoryType type) const {
    switch (type) {
    case TrajectoryType::Circle:
        return getCircleReference(t);
    case TrajectoryType::Eight:
        return getEightReference(t);
    case TrajectoryType::Sine:
        return getSReference(t);
    }

    return getCircleReference(t);
}

Reference ReferenceGenerator::getCircleReference(double t) const {
    if (config_.circle_period <= 0.0) {
        throw std::invalid_argument("circle_period must be positive");
    }

    const double radius = config_.circle_radius;
    const double omega = 2.0 * math_utils::pi() / config_.circle_period;
    const double angle = omega * t;
    const double dx = -radius * omega * std::sin(angle);
    const double dy = radius * omega * std::cos(angle);

    return Reference{
        radius * std::cos(angle),
        radius * std::sin(angle),
        std::atan2(dy, dx),
    };
}

Reference ReferenceGenerator::getEightReference(double t) const {
    if (config_.eight_period <= 0.0) {
        throw std::invalid_argument("eight_period must be positive");
    }

    const double amplitude = config_.eight_amplitude;
    const double omega = 2.0 * math_utils::pi() / config_.eight_period;
    const double angle = omega * t;
    const double sin_angle = std::sin(angle);
    const double cos_angle = std::cos(angle);
    const double dx = amplitude * omega * cos_angle;
    const double dy = amplitude * omega * std::cos(2.0 * angle);

    return Reference{
        amplitude * sin_angle,
        amplitude * sin_angle * cos_angle,
        std::atan2(dy, dx),
    };
}

Reference ReferenceGenerator::getSReference(double t) const {
    const double forward_velocity = config_.sine_forward_velocity;
    const double amplitude = config_.sine_amplitude;
    const double omega = config_.sine_omega;
    const double dx = forward_velocity;
    const double dy = amplitude * omega * std::cos(omega * t);

    return Reference{
        forward_velocity * t,
        amplitude * std::sin(omega * t),
        std::atan2(dy, dx),
    };
}

std::vector<Reference> ReferenceGenerator::getHorizonReference(
    double current_time,
    double dt,
    int horizon_steps,
    TrajectoryType type
) const {
    if (dt <= 0.0) {
        throw std::invalid_argument("dt must be positive");
    }
    if (horizon_steps < 0) {
        throw std::invalid_argument("horizon_steps must be non-negative");
    }

    std::vector<Reference> references;
    references.reserve(static_cast<std::size_t>(horizon_steps) + 1U);

    for (int i = 0; i <= horizon_steps; ++i) {
        references.push_back(getReference(current_time + static_cast<double>(i) * dt, type));
    }

    return references;
}
