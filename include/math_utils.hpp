#pragma once

#include <algorithm>
#include <cmath>

namespace math_utils {

inline constexpr double pi() {
    return 3.14159265358979323846;
}

inline double wrapToPi(double angle) {
    while (angle >= pi()) {
        angle -= 2.0 * pi();
    }
    while (angle < -pi()) {
        angle += 2.0 * pi();
    }
    return angle;
}

inline double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

inline double distance2D(double x1, double y1, double x2, double y2) {
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace math_utils
