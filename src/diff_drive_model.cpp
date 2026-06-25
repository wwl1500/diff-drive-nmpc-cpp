#include "diff_drive_model.hpp"

#include "math_utils.hpp"

#include <cmath>
#include <stdexcept>

DiffDriveModel::DiffDriveModel(double dt) : dt_(dt) {
    if (dt_ <= 0.0) {
        throw std::invalid_argument("dt must be positive");
    }
}

State DiffDriveModel::step(const State& state, const Control& control) const {
    return State{
        state.x + dt_ * control.v * std::cos(state.theta),
        state.y + dt_ * control.v * std::sin(state.theta),
        math_utils::wrapToPi(state.theta + dt_ * control.omega),
    };
}

double DiffDriveModel::dt() const {
    return dt_;
}
