#pragma once

#include "common_types.hpp"

#include <vector>

class ControllerBase {
public:
    virtual ~ControllerBase() = default;

    virtual Control computeControl(
        const State& current_state,
        const std::vector<Reference>& ref_horizon
    ) = 0;
};
