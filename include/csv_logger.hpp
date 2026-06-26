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
    void logSolveTime(double time, double solve_time_ms, bool solve_ok);

private:
    std::string output_dir_;
    std::ofstream trajectory_file_;
    std::ofstream control_file_;
    std::ofstream error_file_;
    std::ofstream solve_time_file_;
};
