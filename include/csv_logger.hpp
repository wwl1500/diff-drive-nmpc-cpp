#pragma once

#include "common_types.hpp"

#include <fstream>
#include <string>

class CsvLogger {
public:
    explicit CsvLogger(const std::string& output_dir);

    void logTrajectory(double time, const State& state, const Reference& reference);
    void logControl(double time, const Control& control);
    void logError(double time, double ex, double ey, double etheta, double position_error);

private:
    std::ofstream trajectory_file_;
    std::ofstream control_file_;
    std::ofstream error_file_;
};
