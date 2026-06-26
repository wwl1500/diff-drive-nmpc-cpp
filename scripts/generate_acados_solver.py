#!/usr/bin/env python3
"""使用 acados_template 定义 OCP 并生成 C 求解器。

生成的代码位于 acados_generated/ 目录，参数与 prototype/nmpc_casadi.py 严格一致。
采用增广状态方法处理控制增量惩罚：v_prev, omega_prev 作为状态变量。
"""

import os

# acados_template 新版本优先读取 ACADOS_SOURCE_DIR。
if "ACADOS_SOURCE_DIR" not in os.environ and "ACADOS_SOURCE_PATH" in os.environ:
    os.environ["ACADOS_SOURCE_DIR"] = os.environ["ACADOS_SOURCE_PATH"]

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

# --- 与 prototype/nmpc_casadi.py 一致的参数 ---
DT = 0.05
N = 20

# 权重
Q_POS = 20.0
Q_THETA = 2.0
R_V = 0.1
R_OMEGA = 0.05
RD_V = 0.5
RD_OMEGA = 0.1
Q_TERMINAL_POS = 40.0
Q_TERMINAL_THETA = 4.0

# 控制约束
V_MIN = -0.5
V_MAX = 1.0
OMEGA_MIN = -1.5
OMEGA_MAX = 1.5


def create_model() -> AcadosModel:
    """定义增广状态模型（px, py, theta, v_prev, omega_prev）。"""
    # 状态变量
    px = ca.SX.sym("px")
    py = ca.SX.sym("py")
    theta = ca.SX.sym("theta")
    v_prev = ca.SX.sym("v_prev")
    omega_prev = ca.SX.sym("omega_prev")
    x = ca.vertcat(px, py, theta, v_prev, omega_prev)

    # 控制变量
    v = ca.SX.sym("v")
    omega = ca.SX.sym("omega")
    u = ca.vertcat(v, omega)

    # 离散动力学（显式欧拉，与原型一致）
    x_next = ca.vertcat(
        px + DT * v * ca.cos(theta),
        py + DT * v * ca.sin(theta),
        theta + DT * omega,
        v,           # v_prev' = v
        omega,       # omega_prev' = omega
    )

    model = AcadosModel()
    model.name = "diff_drive"
    model.x = x
    model.u = u
    model.disc_dyn_expr = x_next
    # 不需要连续动力学（使用离散模型）
    model.x_dot = ca.SX.sym("x_dot", 5)
    return model


def create_ocp() -> AcadosOcp:
    """创建完整的 OCP 问题。"""
    ocp = AcadosOcp()
    model = create_model()
    ocp.model = model

    # 维度
    ocp.solver_options.N_horizon = N

    # ---- 代价函数：非线性最小二乘 ----
    # 残差 y = [sqrt(q)*px, sqrt(q)*py, sqrt(q_theta)*theta, sqrt(r)*v, sqrt(r)*omega, sqrt(rd)*(v-v_prev), sqrt(rd)*(omega-omega_prev)]
    # 参考值通过 yref 传入，代价 = ||y - yref||^2
    # 角度部分：残差 = theta，yref = theta_ref（误差后 atan2 处理在 C++ 端用 sin/cos 残差替代）

    px, py, theta, v_prev, omega_prev = ca.vertsplit(model.x)
    v, omega = ca.vertsplit(model.u)

    # 阶段残差（7 维）：残差中只含模型变量，参考值通过 yref 设置
    # 对于角度误差，使用 sin(theta) 和 cos(theta) 作为残差分量，
    # yref 对应设置为 sin(theta_ref) 和 cos(theta_ref)，这样 ||y-yref||^2 近似角度误差
    stage_y = ca.vertcat(
        ca.sqrt(Q_POS) * px,
        ca.sqrt(Q_POS) * py,
        ca.sqrt(Q_THETA) * ca.sin(theta),
        ca.sqrt(Q_THETA) * ca.cos(theta),
        ca.sqrt(R_V) * v,
        ca.sqrt(R_OMEGA) * omega,
        ca.sqrt(RD_V) * (v - v_prev),
        ca.sqrt(RD_OMEGA) * (omega - omega_prev),
    )
    ocp.model.cost_y_expr = stage_y
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.yref = np.zeros(8)

    # 终端残差（4 维）
    terminal_y = ca.vertcat(
        ca.sqrt(Q_TERMINAL_POS) * px,
        ca.sqrt(Q_TERMINAL_POS) * py,
        ca.sqrt(Q_TERMINAL_THETA) * ca.sin(theta),
        ca.sqrt(Q_TERMINAL_THETA) * ca.cos(theta),
    )
    ocp.model.cost_y_expr_e = terminal_y
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.yref_e = np.zeros(4)

    # W 矩阵（对角权重，残差已含权重系数，故 W=I）
    ocp.cost.W = np.eye(8)
    ocp.cost.W_e = np.eye(4)

    # ---- 约束 ----
    # 控制量盒约束
    ocp.constraints.lbu = np.array([V_MIN, OMEGA_MIN])
    ocp.constraints.ubu = np.array([V_MAX, OMEGA_MAX])
    ocp.constraints.idxbu = np.array([0, 1])

    # 初始状态等式约束（会在 C++ 端动态设置）
    x0 = np.zeros(5)
    ocp.constraints.x0 = x0

    # ---- 求解器选项 ----
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.nlp_solver_max_iter = 50
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.tf = N * DT

    return ocp


def main() -> None:
    # 切换到项目根目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    os.chdir(project_dir)

    ocp = create_ocp()
    ocp.code_gen_opts.code_export_directory = "acados_generated"

    # json 文件放在项目根目录
    json_file = os.path.join(project_dir, "acados_ocp.json")

    print("正在生成 acados C 求解器...")
    print(f"  nx=5, nu=2, N={N}")
    print(f"  dt={DT}, tf={N * DT}")
    print(f"  输出目录: {os.path.join(project_dir, 'acados_generated')}")

    solver = AcadosOcpSolver(ocp, json_file=json_file, build=False, generate=True)

    print("acados C 求解器生成完成！")
    print("生成文件位于: acados_generated/")
    print("如需单独编译生成的求解器共享库:")
    print("  cd acados_generated && make shared_lib")


if __name__ == "__main__":
    main()
