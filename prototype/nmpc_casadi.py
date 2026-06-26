#!/usr/bin/env python3

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import casadi as ca
import numpy as np


@dataclass(frozen=True)
class State:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Control:
    v: float
    omega: float


@dataclass(frozen=True)
class Reference:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.05
    total_time: float = 40.0
    horizon_steps: int = 20
    trajectory_type: str = "circle"
    output_dir: Path = Path("results_nmpc_python/circle")


@dataclass(frozen=True)
class ReferenceGeneratorConfig:
    circle_radius: float = 2.0
    circle_period: float = 20.0
    eight_amplitude: float = 2.0
    eight_period: float = 20.0
    sine_forward_velocity: float = 0.3
    sine_amplitude: float = 1.0
    sine_omega: float = 0.5


@dataclass(frozen=True)
class NmpcWeights:
    q_pos: float = 20.0
    q_theta: float = 2.0
    r_v: float = 0.1
    r_omega: float = 0.05
    rd_v: float = 0.5
    rd_omega: float = 0.1
    q_terminal_pos: float = 40.0
    q_terminal_theta: float = 4.0


@dataclass(frozen=True)
class NmpcConfig:
    dt: float = 0.05
    horizon_steps: int = 20
    v_min: float = -0.5
    v_max: float = 1.0
    omega_min: float = -1.5
    omega_max: float = 1.5
    weights: NmpcWeights = NmpcWeights()
    ipopt_max_iter: int = 100
    ipopt_tol: float = 1e-4
    ipopt_acceptable_tol: float = 1e-3
    ipopt_acceptable_iter: int = 5


class ReferenceGenerator:
    def __init__(self, config: ReferenceGeneratorConfig = ReferenceGeneratorConfig()):
        self.config = config

    def get_reference(self, t: float, trajectory_type: str) -> Reference:
        if trajectory_type == "circle":
            return self.get_circle_reference(t)
        if trajectory_type == "eight":
            return self.get_eight_reference(t)
        if trajectory_type == "sine":
            return self.get_sine_reference(t)
        raise ValueError(f"unknown trajectory type: {trajectory_type}")

    def get_circle_reference(self, t: float) -> Reference:
        if self.config.circle_period <= 0.0:
            raise ValueError("circle_period must be positive")

        radius = self.config.circle_radius
        omega = 2.0 * math.pi / self.config.circle_period
        angle = omega * t
        dx = -radius * omega * math.sin(angle)
        dy = radius * omega * math.cos(angle)

        return Reference(
            radius * math.cos(angle),
            radius * math.sin(angle),
            math.atan2(dy, dx),
        )

    def get_eight_reference(self, t: float) -> Reference:
        if self.config.eight_period <= 0.0:
            raise ValueError("eight_period must be positive")

        amplitude = self.config.eight_amplitude
        omega = 2.0 * math.pi / self.config.eight_period
        angle = omega * t
        sin_angle = math.sin(angle)
        cos_angle = math.cos(angle)
        dx = amplitude * omega * cos_angle
        dy = amplitude * omega * math.cos(2.0 * angle)

        return Reference(
            amplitude * sin_angle,
            amplitude * sin_angle * cos_angle,
            math.atan2(dy, dx),
        )

    def get_sine_reference(self, t: float) -> Reference:
        forward_velocity = self.config.sine_forward_velocity
        amplitude = self.config.sine_amplitude
        omega = self.config.sine_omega
        dx = forward_velocity
        dy = amplitude * omega * math.cos(omega * t)

        return Reference(
            forward_velocity * t,
            amplitude * math.sin(omega * t),
            math.atan2(dy, dx),
        )

    def get_horizon_reference(
        self,
        current_time: float,
        dt: float,
        horizon_steps: int,
        trajectory_type: str,
    ) -> list[Reference]:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if horizon_steps < 0:
            raise ValueError("horizon_steps must be non-negative")

        return [
            self.get_reference(current_time + i * dt, trajectory_type)
            for i in range(horizon_steps + 1)
        ]


def wrap_to_pi(angle: float) -> float:
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_error_numeric(angle: float, reference: float) -> float:
    return wrap_to_pi(angle - reference)


def step_diff_drive(state: State, control: Control, dt: float) -> State:
    return State(
        state.x + dt * control.v * math.cos(state.theta),
        state.y + dt * control.v * math.sin(state.theta),
        wrap_to_pi(state.theta + dt * control.omega),
    )


class CsvLogger:
    def __init__(self, output_dir: Path):
        output_dir.mkdir(parents=True, exist_ok=True)
        self.trajectory_file = (output_dir / "trajectory.csv").open("w", newline="")
        self.control_file = (output_dir / "control.csv").open("w", newline="")
        self.error_file = (output_dir / "error.csv").open("w", newline="")

        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.control_writer = csv.writer(self.control_file)
        self.error_writer = csv.writer(self.error_file)

        self.trajectory_writer.writerow(["time", "x", "y", "theta", "x_ref", "y_ref", "theta_ref"])
        self.control_writer.writerow(["time", "v", "omega"])
        self.error_writer.writerow(["time", "ex", "ey", "etheta", "position_error"])

    def close(self) -> None:
        self.trajectory_file.close()
        self.control_file.close()
        self.error_file.close()

    def log_trajectory(self, current_time: float, state: State, reference: Reference) -> None:
        self.trajectory_writer.writerow(format_row(
            current_time,
            state.x,
            state.y,
            state.theta,
            reference.x,
            reference.y,
            reference.theta,
        ))

    def log_control(self, current_time: float, control: Control) -> None:
        self.control_writer.writerow(format_row(current_time, control.v, control.omega))

    def log_error(
        self,
        current_time: float,
        ex: float,
        ey: float,
        etheta: float,
        position_error: float,
    ) -> None:
        self.error_writer.writerow(format_row(current_time, ex, ey, etheta, position_error))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def format_row(*values: float) -> list[str]:
    return [f"{value:.6f}" for value in values]


class NmpcController:
    def __init__(self, config: NmpcConfig):
        self.config = config
        self.last_x_solution: np.ndarray | None = None
        self.last_u_solution: np.ndarray | None = None
        self.last_control = np.zeros(2)
        self.solve_failures = 0
        self.solve_times: list[float] = []
        self._build_problem()

    def _build_problem(self) -> None:
        cfg = self.config
        weights = cfg.weights
        n = cfg.horizon_steps

        opti = ca.Opti()
        x = opti.variable(3, n + 1)
        u = opti.variable(2, n)
        x0 = opti.parameter(3)
        ref = opti.parameter(3, n + 1)
        u_prev = opti.parameter(2)

        opti.subject_to(x[:, 0] == x0)

        cost = 0
        for k in range(n):
            theta = x[2, k]
            v = u[0, k]
            omega = u[1, k]
            x_next = ca.vertcat(
                x[0, k] + cfg.dt * v * ca.cos(theta),
                x[1, k] + cfg.dt * v * ca.sin(theta),
                x[2, k] + cfg.dt * omega,
            )
            opti.subject_to(x[:, k + 1] == x_next)

            pos_error = x[0:2, k] - ref[0:2, k]
            theta_error = ca.atan2(ca.sin(x[2, k] - ref[2, k]), ca.cos(x[2, k] - ref[2, k]))
            if k == 0:
                du = u[:, k] - u_prev
            else:
                du = u[:, k] - u[:, k - 1]

            cost += weights.q_pos * ca.dot(pos_error, pos_error)
            cost += weights.q_theta * theta_error * theta_error
            cost += weights.r_v * v * v
            cost += weights.r_omega * omega * omega
            cost += weights.rd_v * du[0] * du[0]
            cost += weights.rd_omega * du[1] * du[1]

        terminal_pos_error = x[0:2, n] - ref[0:2, n]
        terminal_theta_error = ca.atan2(ca.sin(x[2, n] - ref[2, n]), ca.cos(x[2, n] - ref[2, n]))
        cost += weights.q_terminal_pos * ca.dot(terminal_pos_error, terminal_pos_error)
        cost += weights.q_terminal_theta * terminal_theta_error * terminal_theta_error

        opti.subject_to(opti.bounded(cfg.v_min, u[0, :], cfg.v_max))
        opti.subject_to(opti.bounded(cfg.omega_min, u[1, :], cfg.omega_max))
        opti.minimize(cost)

        solver_options = {
            "print_time": False,
            "ipopt": {
                "print_level": 0,
                "max_iter": cfg.ipopt_max_iter,
                "tol": cfg.ipopt_tol,
                "acceptable_tol": cfg.ipopt_acceptable_tol,
                "acceptable_iter": cfg.ipopt_acceptable_iter,
            },
        }
        opti.solver("ipopt", solver_options)

        self.opti = opti
        self.x = x
        self.u = u
        self.x0 = x0
        self.ref = ref
        self.u_prev = u_prev

    def compute_control(self, state: State, ref_horizon: list[Reference]) -> Control:
        cfg = self.config
        ref_matrix = references_to_matrix(ref_horizon)
        current_state = np.array([state.x, state.y, state.theta], dtype=float)
        x_guess, u_guess = self._make_initial_guess(current_state, ref_matrix)

        self.opti.set_value(self.x0, current_state)
        self.opti.set_value(self.ref, ref_matrix)
        self.opti.set_value(self.u_prev, self.last_control)
        self.opti.set_initial(self.x, x_guess)
        self.opti.set_initial(self.u, u_guess)

        start_time = time.perf_counter()
        try:
            solution = self.opti.solve()
            solve_time = time.perf_counter() - start_time
            self.solve_times.append(solve_time)

            x_solution = np.array(solution.value(self.x), dtype=float)
            u_solution = np.array(solution.value(self.u), dtype=float)
            control_array = u_solution[:, 0]

            self.last_x_solution = x_solution
            self.last_u_solution = u_solution
            self.last_control = self._clip_control(control_array)
            return Control(float(self.last_control[0]), float(self.last_control[1]))
        except RuntimeError as error:
            solve_time = time.perf_counter() - start_time
            self.solve_times.append(solve_time)
            self.solve_failures += 1
            fallback = self._fallback_control()
            self.last_control = fallback
            error_lines = str(error).splitlines()
            error_summary = error_lines[0] if error_lines else "unknown solver error"
            print(f"[NMPC] solve failed, fallback control used: {error_summary}")
            return Control(float(fallback[0]), float(fallback[1]))

    def _make_initial_guess(self, current_state: np.ndarray, ref_matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        n = self.config.horizon_steps
        if self.last_x_solution is not None and self.last_u_solution is not None:
            x_guess = np.empty((3, n + 1), dtype=float)
            u_guess = np.empty((2, n), dtype=float)
            x_guess[:, 0:n] = self.last_x_solution[:, 1:n + 1]
            x_guess[:, n] = self.last_x_solution[:, n]
            x_guess[:, 0] = current_state
            u_guess[:, 0:n - 1] = self.last_u_solution[:, 1:n]
            u_guess[:, n - 1] = self.last_u_solution[:, n - 1]
            return x_guess, u_guess

        x_guess = ref_matrix.copy()
        x_guess[:, 0] = current_state
        u_guess = self._estimate_control_guess(ref_matrix)
        return x_guess, u_guess

    def _estimate_control_guess(self, ref_matrix: np.ndarray) -> np.ndarray:
        cfg = self.config
        u_guess = np.zeros((2, cfg.horizon_steps), dtype=float)
        for k in range(cfg.horizon_steps):
            dx = ref_matrix[0, k + 1] - ref_matrix[0, k]
            dy = ref_matrix[1, k + 1] - ref_matrix[1, k]
            dtheta = angle_error_numeric(ref_matrix[2, k + 1], ref_matrix[2, k])
            u_guess[0, k] = np.clip(math.hypot(dx, dy) / cfg.dt, cfg.v_min, cfg.v_max)
            u_guess[1, k] = np.clip(dtheta / cfg.dt, cfg.omega_min, cfg.omega_max)
        return u_guess

    def _fallback_control(self) -> np.ndarray:
        if self.last_u_solution is not None and self.last_u_solution.shape[1] > 1:
            return self._clip_control(self.last_u_solution[:, 1])
        return self._clip_control(self.last_control)

    def _clip_control(self, control: np.ndarray) -> np.ndarray:
        cfg = self.config
        return np.array([
            np.clip(control[0], cfg.v_min, cfg.v_max),
            np.clip(control[1], cfg.omega_min, cfg.omega_max),
        ], dtype=float)

    @property
    def average_solve_time_ms(self) -> float:
        if not self.solve_times:
            return 0.0
        return 1000.0 * sum(self.solve_times) / len(self.solve_times)

    @property
    def max_solve_time_ms(self) -> float:
        if not self.solve_times:
            return 0.0
        return 1000.0 * max(self.solve_times)


def references_to_matrix(ref_horizon: list[Reference]) -> np.ndarray:
    return np.array(
        [[ref.x for ref in ref_horizon],
         [ref.y for ref in ref_horizon],
         [ref.theta for ref in ref_horizon]],
        dtype=float,
    )


def run_simulation(sim_config: SimulationConfig, nmpc_config: NmpcConfig) -> NmpcController:
    reference_generator = ReferenceGenerator()
    controller = NmpcController(nmpc_config)

    initial_reference = reference_generator.get_reference(0.0, sim_config.trajectory_type)
    state = State(initial_reference.x, initial_reference.y - 0.2, initial_reference.theta)
    num_steps = round(sim_config.total_time / sim_config.dt)

    with CsvLogger(sim_config.output_dir) as logger:
        for step in range(num_steps + 1):
            current_time = step * sim_config.dt
            reference = reference_generator.get_reference(current_time, sim_config.trajectory_type)
            ref_horizon = reference_generator.get_horizon_reference(
                current_time,
                sim_config.dt,
                sim_config.horizon_steps,
                sim_config.trajectory_type,
            )

            control = controller.compute_control(state, ref_horizon)
            ex = state.x - reference.x
            ey = state.y - reference.y
            etheta = wrap_to_pi(state.theta - reference.theta)
            position_error = math.hypot(ex, ey)

            logger.log_trajectory(current_time, state, reference)
            logger.log_control(current_time, control)
            logger.log_error(current_time, ex, ey, etheta, position_error)

            state = step_diff_drive(state, control, sim_config.dt)

    return controller


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a Python + CasADi NMPC prototype for the diff-drive simulator.")
    parser.add_argument("trajectory", nargs="?", default="circle", choices=["circle", "eight", "sine"])
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--total-time", type=float, default=40.0)
    parser.add_argument("--horizon-steps", type=int, default=20)
    parser.add_argument("--ipopt-max-iter", type=int, default=100)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.dt <= 0.0:
        raise ValueError("dt must be positive")
    if args.total_time < 0.0:
        raise ValueError("total-time must be non-negative")
    if args.horizon_steps <= 0:
        raise ValueError("horizon-steps must be positive")

    output_dir = args.output_dir if args.output_dir is not None else Path("results_nmpc_python") / args.trajectory
    sim_config = SimulationConfig(
        dt=args.dt,
        total_time=args.total_time,
        horizon_steps=args.horizon_steps,
        trajectory_type=args.trajectory,
        output_dir=output_dir,
    )
    nmpc_config = NmpcConfig(
        dt=args.dt,
        horizon_steps=args.horizon_steps,
        ipopt_max_iter=args.ipopt_max_iter,
    )

    controller = run_simulation(sim_config, nmpc_config)
    num_steps = round(sim_config.total_time / sim_config.dt) + 1
    print(
        "Simulation finished. "
        f"trajectory={sim_config.trajectory_type}, "
        f"output_dir={sim_config.output_dir}, "
        f"steps={num_steps}, "
        f"solve_failures={controller.solve_failures}/{num_steps}, "
        f"avg_solve_time_ms={controller.average_solve_time_ms:.3f}, "
        f"max_solve_time_ms={controller.max_solve_time_ms:.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
