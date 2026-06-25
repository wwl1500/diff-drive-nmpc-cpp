#include "csv_logger.hpp"
#include "diff_drive_model.hpp"
#include "math_utils.hpp"
#include "pure_pursuit_controller.hpp"
#include "reference_generator.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

struct SimulationConfig {
    double dt = 0.05;
    double total_time = 40.0;
    int horizon_steps = 20;
    TrajectoryType trajectory_type = TrajectoryType::Circle;
    std::string output_dir = "results";
};

TrajectoryType parseTrajectoryType(const std::string& name) {
    if (name == "circle") {
        return TrajectoryType::Circle;
    }
    if (name == "eight") {
        return TrajectoryType::Eight;
    }
    if (name == "sine") {
        return TrajectoryType::Sine;
    }

    throw std::invalid_argument("unknown trajectory type: " + name);
}

const char* trajectoryName(TrajectoryType type) {
    switch (type) {
    case TrajectoryType::Circle:
        return "circle";
    case TrajectoryType::Eight:
        return "eight";
    case TrajectoryType::Sine:
        return "sine";
    }

    return "circle";
}

int main(int argc, char** argv) {
    try {
        SimulationConfig sim_config;
        if (argc > 1) {
            sim_config.trajectory_type = parseTrajectoryType(argv[1]);
        }

        std::filesystem::create_directories(sim_config.output_dir);

        ReferenceGenerator reference_generator;
        DiffDriveModel model(sim_config.dt);
        PurePursuitController controller;
        CsvLogger logger(sim_config.output_dir);

        const Reference initial_reference = reference_generator.getReference(0.0, sim_config.trajectory_type);
        State state{initial_reference.x, initial_reference.y - 0.2, initial_reference.theta};
        const int num_steps = static_cast<int>(std::round(sim_config.total_time / sim_config.dt));

        for (int step = 0; step <= num_steps; ++step) {
            const double time = static_cast<double>(step) * sim_config.dt;
            const Reference reference = reference_generator.getReference(time, sim_config.trajectory_type);
            const auto ref_horizon = reference_generator.getHorizonReference(
                time,
                sim_config.dt,
                sim_config.horizon_steps,
                sim_config.trajectory_type
            );

            const Control control = controller.computeControl(state, ref_horizon);
            const double ex = state.x - reference.x;
            const double ey = state.y - reference.y;
            const double etheta = math_utils::wrapToPi(state.theta - reference.theta);
            const double position_error = std::sqrt(ex * ex + ey * ey);

            logger.logTrajectory(time, state, reference);
            logger.logControl(time, control);
            logger.logError(time, ex, ey, etheta, position_error);

            state = model.step(state, control);
        }

        std::cout << "Simulation finished. trajectory=" << trajectoryName(sim_config.trajectory_type)
                  << ", output_dir=" << sim_config.output_dir << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        std::cerr << "Usage: diff_drive_sim [circle|eight|sine]\n";
        return 1;
    }
}
