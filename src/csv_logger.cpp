#include "csv_logger.hpp"

#include <iomanip>
#include <stdexcept>

namespace {

std::string makePath(const std::string& output_dir, const std::string& filename) {
    if (output_dir.empty()) {
        return filename;
    }
    if (output_dir.back() == '/') {
        return output_dir + filename;
    }
    return output_dir + "/" + filename;
}

void configureFile(std::ofstream& file) {
    file << std::fixed << std::setprecision(6);
}

}  // namespace

CsvLogger::CsvLogger(const std::string& output_dir)
    : output_dir_(output_dir),
      trajectory_file_(makePath(output_dir, "trajectory.csv")),
      control_file_(makePath(output_dir, "control.csv")),
      error_file_(makePath(output_dir, "error.csv")) {
    if (!trajectory_file_ || !control_file_ || !error_file_) {
        throw std::runtime_error("failed to open csv output files");
    }

    configureFile(trajectory_file_);
    configureFile(control_file_);
    configureFile(error_file_);

    trajectory_file_ << "time,x,y,theta,x_ref,y_ref,theta_ref\n";
    control_file_ << "time,v,omega\n";
    error_file_ << "time,ex,ey,etheta,position_error\n";
}

void CsvLogger::logTrajectory(double time, const State& state, const Reference& reference) {
    trajectory_file_ << time << ','
                     << state.x << ','
                     << state.y << ','
                     << state.theta << ','
                     << reference.x << ','
                     << reference.y << ','
                     << reference.theta << '\n';
}

void CsvLogger::logControl(double time, const Control& control) {
    control_file_ << time << ',' << control.v << ',' << control.omega << '\n';
}

void CsvLogger::logError(double time, double ex, double ey, double etheta, double position_error) {
    error_file_ << time << ',' << ex << ',' << ey << ',' << etheta << ',' << position_error << '\n';
}

void CsvLogger::logSolveTime(double time, double solve_time_ms, bool solve_ok) {
    if (!solve_time_file_.is_open()) {
        solve_time_file_.open(makePath(output_dir_, "solve_time.csv"));
        if (!solve_time_file_) {
            throw std::runtime_error("failed to open solve_time.csv");
        }
        configureFile(solve_time_file_);
        solve_time_file_ << "time,solve_time_ms,solve_ok\n";
    }
    solve_time_file_ << time << ',' << solve_time_ms << ',' << (solve_ok ? 1 : 0) << '\n';
}
