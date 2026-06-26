#include "csv_logger.hpp"
#include "controller_base.hpp"
#include "diff_drive_model.hpp"
#include "math_utils.hpp"
#include "pure_pursuit_controller.hpp"
#include "reference_generator.hpp"

#ifdef HAS_NMPC
#include "nmpc_controller.hpp"
#endif

#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

enum class ControllerType {
    PurePursuit,
    NMPC,
};

struct SimulationConfig {
    double dt = 0.05;
    double total_time = 40.0;
    int horizon_steps = 20;
    TrajectoryType trajectory_type = TrajectoryType::Circle;
    ControllerType controller_type = ControllerType::PurePursuit;
    std::string output_dir = "results";
};

TrajectoryType parseTrajectoryType(const std::string& name) {
    if (name == "circle") return TrajectoryType::Circle;
    if (name == "eight") return TrajectoryType::Eight;
    if (name == "sine") return TrajectoryType::Sine;
    throw std::invalid_argument("unknown trajectory type: " + name);
}

ControllerType parseControllerType(const std::string& name) {
    if (name == "pp") return ControllerType::PurePursuit;
    if (name == "nmpc") return ControllerType::NMPC;
    throw std::invalid_argument("unknown controller type: " + name + " (use pp or nmpc)");
}

const char* trajectoryName(TrajectoryType type) {
    switch (type) {
    case TrajectoryType::Circle: return "circle";
    case TrajectoryType::Eight: return "eight";
    case TrajectoryType::Sine: return "sine";
    }
    return "circle";
}

const char* controllerName(ControllerType type) {
    switch (type) {
    case ControllerType::PurePursuit: return "pp";
    case ControllerType::NMPC: return "nmpc";
    }
    return "pp";
}

int main(int argc, char** argv) {
    try {
        SimulationConfig sim_config;
        if (argc > 1) {
            sim_config.trajectory_type = parseTrajectoryType(argv[1]);
        }
        if (argc > 2) {
            sim_config.controller_type = parseControllerType(argv[2]);
        }

        // NMPC 输出到独立目录
        if (sim_config.controller_type == ControllerType::NMPC) {
            sim_config.output_dir = std::string("results_nmpc_cpp/") + trajectoryName(sim_config.trajectory_type);
        }

        std::filesystem::create_directories(sim_config.output_dir);

        ReferenceGenerator reference_generator;
        DiffDriveModel model(sim_config.dt);
        CsvLogger logger(sim_config.output_dir);

        // 多态创建控制器
        std::unique_ptr<ControllerBase> controller;
#ifdef HAS_NMPC
        if (sim_config.controller_type == ControllerType::NMPC) {
            NmpcControllerConfig nmpc_config;
            nmpc_config.dt = sim_config.dt;
            nmpc_config.horizon_steps = sim_config.horizon_steps;
            controller = std::make_unique<NmpcController>(nmpc_config);
        } else {
#endif
            controller = std::make_unique<PurePursuitController>();
#ifdef HAS_NMPC
        }
#endif

        const Reference initial_reference = reference_generator.getReference(0.0, sim_config.trajectory_type);
        State state{initial_reference.x, initial_reference.y - 0.2, initial_reference.theta};
        const int num_steps = static_cast<int>(std::round(sim_config.total_time / sim_config.dt));

        for (int step = 0; step <= num_steps; ++step) {
            const double time = static_cast<double>(step) * sim_config.dt;
            const Reference reference = reference_generator.getReference(time, sim_config.trajectory_type);
            const auto ref_horizon = reference_generator.getHorizonReference(
                time, sim_config.dt, sim_config.horizon_steps, sim_config.trajectory_type);

            const Control control = controller->computeControl(state, ref_horizon);

            const double ex = state.x - reference.x;
            const double ey = state.y - reference.y;
            const double etheta = math_utils::wrapToPi(state.theta - reference.theta);
            const double position_error = std::sqrt(ex * ex + ey * ey);

            logger.logTrajectory(time, state, reference);
            logger.logControl(time, control);
            logger.logError(time, ex, ey, etheta, position_error);

#ifdef HAS_NMPC
            // NMPC 求解时间记录
            if (auto* nmpc = dynamic_cast<NmpcController*>(controller.get())) {
                logger.logSolveTime(time, nmpc->lastSolveTimeMs(), nmpc->lastSolveOk());
            }
#endif

            state = model.step(state, control);
        }

        std::cout << "Simulation finished. trajectory=" << trajectoryName(sim_config.trajectory_type)
                  << ", controller=" << controllerName(sim_config.controller_type)
                  << ", output_dir=" << sim_config.output_dir << '\n';

#ifdef HAS_NMPC
        if (auto* nmpc = dynamic_cast<NmpcController*>(controller.get())) {
            const auto& stats = nmpc->solveStats();
            std::cout << "NMPC stats: solves=" << stats.total_solves
                      << ", failures=" << stats.failed_solves
                      << ", avg_time_ms=" << stats.average_solve_time_ms()
                      << ", max_time_ms=" << stats.max_solve_time_ms << '\n';
        }
#endif

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        std::cerr << "Usage: diff_drive_sim [circle|eight|sine] [pp|nmpc]\n";
        return 1;
    }
}
