#pragma once

#include "common_types.hpp"

class DiffDriveModel {
public:
    explicit DiffDriveModel(double dt);

    State step(const State& state, const Control& control) const;

    double dt() const;

private:
    double dt_;
};
