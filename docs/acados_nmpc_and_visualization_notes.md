# 第三步与第四步项目笔记：acados C 求解器接入、数据记录与可视化

本文记录本项目从 Python + CasADi NMPC 原型推进到 C++ + acados NMPC 控制器，以及补齐数据记录与可视化对比的实现过程。

对应项目指南：

- 第三步：接入 acados 生成 C 求解器
- 第四步：数据记录与可视化

---

## 1. 阶段目标

### 1.1 第三步目标

将第二阶段已经验证过的 `prototype/nmpc_casadi.py` 中的 NMPC 问题迁移到 acados：

1. 用 Python + `acados_template` 定义 OCP。
2. 生成 acados C 求解器代码。
3. 在 C++ 中封装 `NmpcController`。
4. 让 C++ 仿真主循环可以选择 Pure Pursuit 或 NMPC 控制器。
5. 统计 NMPC 每次求解耗时。

### 1.2 第四步目标

在已有 CSV 记录与单结果绘图基础上补齐：

1. 记录 NMPC 求解时间。
2. 复用已有 `plot_results.py` 绘制单控制器结果。
3. 新增 PP vs NMPC 的误差对比图、求解时间图和综合柱状图。
4. 形成可量化比较：跟踪误差、控制输入、求解时间。

---

## 2. 环境准备

acados 通过源码安装在：

```text
/home/wwlwwl/acados
```

关键环境变量：

```bash
export ACADOS_SOURCE_PATH=/home/wwlwwl/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_PATH/lib:$LD_LIBRARY_PATH
```

Python 虚拟环境为项目根目录下的 `.venv`，已安装：

- `casadi`
- `acados_template`
- `jinja2`
- `numpy`
- `matplotlib`
- `scipy`

acados 生成模板需要 `tera_renderer`，已下载到：

```text
/home/wwlwwl/acados/bin/t_renderer
```

验证命令：

```bash
/home/wwlwwl/acados/bin/t_renderer --version
```

输出：

```text
t_renderer 0.2.0
```

---

## 3. acados OCP 设计

OCP 生成脚本位于：

```text
scripts/generate_acados_solver.py
```

该脚本是 C 求解器的源头，核心参数对齐 Python 原型：

```text
dt = 0.05 s
N = 20
v ∈ [-0.5, 1.0] m/s
omega ∈ [-1.5, 1.5] rad/s
```

### 3.1 状态与控制量

原始差速小车状态：

```text
x = [px, py, theta]
u = [v, omega]
```

为了表达控制增量惩罚，引入增广状态：

```text
x_aug = [px, py, theta, v_prev, omega_prev]
```

因此 acados 问题维度为：

```text
nx = 5
nu = 2
N = 20
```

生成的头文件 `acados_generated/acados_solver_diff_drive.h` 中可见：

```c
#define DIFF_DRIVE_NX     5
#define DIFF_DRIVE_NU     2
#define DIFF_DRIVE_N      20
#define DIFF_DRIVE_NY     8
#define DIFF_DRIVE_NYN    4
```

### 3.2 离散动力学

动力学使用显式欧拉离散化，与 C++ 仿真模型和 Python 原型一致：

```text
px_next = px + dt * v * cos(theta)
py_next = py + dt * v * sin(theta)
theta_next = theta + dt * omega
v_prev_next = v
omega_prev_next = omega
```

其中 `v_prev_next = v` 与 `omega_prev_next = omega` 让下一阶段能够计算控制增量：

```text
v(k) - v_prev(k)
omega(k) - omega_prev(k)
```

### 3.3 代价函数

Python 原型中的权重为：

```text
q_pos = 20.0
q_theta = 2.0
r_v = 0.1
r_omega = 0.05
rd_v = 0.5
rd_omega = 0.1
q_terminal_pos = 40.0
q_terminal_theta = 4.0
```

acados 中使用 `NONLINEAR_LS` 代价形式。阶段残差为 8 维：

```text
y = [
  sqrt(q_pos) * px,
  sqrt(q_pos) * py,
  sqrt(q_theta) * sin(theta),
  sqrt(q_theta) * cos(theta),
  sqrt(r_v) * v,
  sqrt(r_omega) * omega,
  sqrt(rd_v) * (v - v_prev),
  sqrt(rd_omega) * (omega - omega_prev)
]
```

对应参考 `yref` 在 C++ 中每步设置为：

```text
yref = [
  sqrt(q_pos) * x_ref,
  sqrt(q_pos) * y_ref,
  sqrt(q_theta) * sin(theta_ref),
  sqrt(q_theta) * cos(theta_ref),
  0,
  0,
  0,
  0
]
```

角度误差没有直接写成 `theta - theta_ref`，而是通过 `sin/cos` 残差表达。这样可以避免角度跨越 `±pi` 附近时的突变问题，也符合轨迹跟踪中常用的圆周角误差处理方式。

终端残差为 4 维：

```text
y_e = [
  sqrt(q_terminal_pos) * px,
  sqrt(q_terminal_pos) * py,
  sqrt(q_terminal_theta) * sin(theta),
  sqrt(q_terminal_theta) * cos(theta)
]
```

终端参考为：

```text
yref_e = [
  sqrt(q_terminal_pos) * x_ref_N,
  sqrt(q_terminal_pos) * y_ref_N,
  sqrt(q_terminal_theta) * sin(theta_ref_N),
  sqrt(q_terminal_theta) * cos(theta_ref_N)
]
```

### 3.4 控制约束

acados 中设置控制输入盒约束：

```text
lbu = [-0.5, -1.5]
ubu = [ 1.0,  1.5]
idxbu = [0, 1]
```

即：

```text
v ∈ [-0.5, 1.0] m/s
omega ∈ [-1.5, 1.5] rad/s
```

### 3.5 求解器选项

当前配置：

```text
qp_solver = PARTIAL_CONDENSING_HPIPM
hessian_approx = GAUSS_NEWTON
integrator_type = DISCRETE
nlp_solver_type = SQP_RTI
nlp_solver_max_iter = 50
qp_solver_iter_max = 50
tf = N * dt = 1.0 s
```

`SQP_RTI` 适合实时 NMPC 场景，每次控制周期只执行一次 SQP 迭代，结合 warm-start 可以获得较低求解耗时。

---

## 4. acados 生成流程

生成命令：

```bash
source .venv/bin/activate
export ACADOS_SOURCE_PATH=/home/wwlwwl/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_PATH/lib:$LD_LIBRARY_PATH
python3 scripts/generate_acados_solver.py
```

生成文件位于：

```text
acados_generated/
├── acados_solver_diff_drive.c
├── acados_solver_diff_drive.h
├── acados_solver.pxd
├── diff_drive_cost/
├── diff_drive_model/
├── main_diff_drive.c
└── Makefile
```

如需单独编译 acados 生成的共享库：

```bash
cd acados_generated
make shared_lib
```

本项目的主 CMake 不依赖该共享库，而是直接把生成的 `.c` 文件编译为 `acados_ocp_solver` 静态库，并链接到 `diff_drive_core`。

---

## 5. C++ NMPCController 封装

新增文件：

```text
include/nmpc_controller.hpp
src/nmpc_controller.cpp
```

`NmpcController` 继承已有统一接口：

```cpp
class NmpcController : public ControllerBase {
public:
    Control computeControl(
        const State& current_state,
        const std::vector<Reference>& ref_horizon
    ) override;
};
```

这样 `main.cpp` 可以通过 `std::unique_ptr<ControllerBase>` 在 Pure Pursuit 与 NMPC 间切换。

### 5.1 生命周期管理

构造函数中创建 acados capsule：

```cpp
capsule_ = diff_drive_acados_create_capsule();
diff_drive_acados_create(capsule_);
```

析构函数中释放 acados 资源：

```cpp
diff_drive_acados_free(capsule_);
diff_drive_acados_free_capsule(capsule_);
```

### 5.2 每次 computeControl 的流程

`computeControl()` 每个控制周期执行：

1. 设置初始状态约束：

   ```text
   [state.x, state.y, state.theta, last_v, last_omega]
   ```

2. 设置阶段参考 `yref` 和终端参考 `yref_e`。
3. 设置 warm-start 初值。
4. 调用：

   ```cpp
   diff_drive_acados_solve(capsule_);
   ```

5. 统计求解耗时。
6. 成功时读取第一个控制量 `u0`。
7. 失败时使用 fallback 控制。
8. 保存求解状态、控制量和 warm-start 轨迹。

### 5.3 warm-start

控制周期之间保存上一轮求解得到的：

```text
x_warm_: nx * (N + 1)
u_warm_: nu * N
```

下一周期求解前进行 shift：

```text
x[1:N+1] -> x[0:N]
u[1:N]   -> u[0:N-1]
```

最后一个点复制前一个点。这样可以让 RTI 求解从更接近当前最优解的位置开始，提升收敛速度并降低耗时。

### 5.4 fallback 策略

若 acados 返回非 0 状态：

1. `failed_solves++`
2. `last_solve_ok_ = false`
3. 优先使用 warm-start 中的后续控制量
4. 否则沿用上一控制量
5. 对 `v` 与 `omega` 再做限幅

当前验证中三条轨迹均未触发 fallback。

### 5.5 求解统计

`SolveStats` 记录：

```cpp
int total_solves;
int failed_solves;
double last_solve_time_ms;
double total_solve_time_ms;
double max_solve_time_ms;
```

控制器额外提供：

```cpp
double lastSolveTimeMs() const;
bool lastSolveOk() const;
const SolveStats& solveStats() const;
```

---

## 6. CMake 集成

`CMakeLists.txt` 的关键变化：

1. 项目语言从 `CXX` 扩展为 `C CXX`，因为需要编译 acados 生成的 C 文件。
2. 添加选项：

   ```cmake
   option(ENABLE_NMPC "构建 acados NMPC 控制器" ON)
   ```

3. 通过 `ACADOS_SOURCE_PATH` 定位 acados 安装目录。
4. 将生成的 acados C 文件编译为静态库：

   ```cmake
   add_library(acados_ocp_solver STATIC ...)
   ```

5. `diff_drive_core` 链接：

   ```cmake
   acados_ocp_solver
   libacados.so
   libhpipm.so
   libblasfeo.so
   m
   ```

6. 定义：

   ```cmake
   target_compile_definitions(diff_drive_core PUBLIC HAS_NMPC=1)
   ```

`main.cpp` 通过 `#ifdef HAS_NMPC` 条件编译 NMPC 相关代码。

---

## 7. 主程序命令行扩展

`diff_drive_sim` 当前用法：

```bash
./build/diff_drive_sim [circle|eight|sine] [pp|nmpc]
```

示例：

```bash
./build/diff_drive_sim circle pp
./build/diff_drive_sim circle nmpc
./build/diff_drive_sim eight nmpc
./build/diff_drive_sim sine nmpc
```

默认：

- 轨迹：`circle`
- 控制器：`pp`

输出目录：

```text
PP:   results/
NMPC: results_nmpc_cpp/<trajectory>/
```

---

## 8. 数据记录

`CsvLogger` 现在负责四类 CSV 文件。

### 8.1 通用输出

所有控制器都会输出：

```text
trajectory.csv: time,x,y,theta,x_ref,y_ref,theta_ref
control.csv:    time,v,omega
error.csv:      time,ex,ey,etheta,position_error
```

### 8.2 NMPC 求解时间输出

只有 NMPC 调用 `logSolveTime()` 时才会生成：

```text
solve_time.csv: time,solve_time_ms,solve_ok
```

该文件采用懒创建策略：Pure Pursuit 模式不会生成空的 `solve_time.csv`。

示例：

```text
time,solve_time_ms,solve_ok
0.000000,1.025850,1
0.050000,0.744534,1
```

---

## 9. 可视化脚本

### 9.1 单控制器绘图

已有脚本：

```text
scripts/plot_results.py
```

用法：

```bash
python3 scripts/plot_results.py results_nmpc_cpp/circle
```

输出：

```text
trajectory_plot.png
error_plot.png
control_plot.png
```

### 9.2 PP vs NMPC 对比绘图

新增脚本：

```text
scripts/plot_comparison.py
```

用法：

```bash
python3 scripts/plot_comparison.py results results_nmpc_cpp/circle
```

输出到 NMPC 目录：

```text
solve_time_plot.png
error_comparison_plot.png
comparison_bar_plot.png
```

三张图分别表示：

1. NMPC 求解时间曲线与直方图。
2. PP 与 NMPC 的位置误差、角度误差对比。
3. 平均位置误差、RMS 位置误差、最大位置误差、平均角度误差、平均求解时间柱状图。

---

## 10. 验证结果

### 10.1 构建验证

命令：

```bash
export ACADOS_SOURCE_PATH=/home/wwlwwl/acados
cmake -S . -B build -DACADOS_SOURCE_PATH=$ACADOS_SOURCE_PATH
cmake --build build
```

结果：

```text
[100%] Built target diff_drive_sim
```

### 10.2 NMPC 三轨迹验证

运行：

```bash
export LD_LIBRARY_PATH=/home/wwlwwl/acados/lib:$LD_LIBRARY_PATH
./build/diff_drive_sim circle nmpc
./build/diff_drive_sim eight nmpc
./build/diff_drive_sim sine nmpc
```

验证结果：

```text
circle: solves=801, failures=0, avg_time_ms≈0.095~0.126 ms
eight:  solves=801, failures=0, avg_time_ms≈0.104~0.105 ms
sine:   solves=801, failures=0, avg_time_ms≈0.098~0.123 ms
```

### 10.3 PP 回归验证

运行：

```bash
./build/diff_drive_sim circle pp
```

结果：

```text
Simulation finished. trajectory=circle, controller=pp, output_dir=results
```

PP 模式正常运行，且不再生成空的 `solve_time.csv`。

### 10.4 对比绘图验证

运行：

```bash
python3 scripts/plot_results.py results_nmpc_cpp/circle
python3 scripts/plot_comparison.py results results_nmpc_cpp/circle
```

输出：

```text
Plots saved to results_nmpc_cpp/circle
solve_time_plot.png
error_comparison_plot.png
comparison_bar_plot.png
```

circle 轨迹对比中，观测到：

```text
PP mean position error ≈ 1.6446 m
NMPC mean position error ≈ 0.0041 m
```

说明 NMPC 在该参考轨迹上显著优于当前 Pure Pursuit baseline。

---

## 11. 当前文件变更总结

### 新增文件

```text
scripts/generate_acados_solver.py
scripts/plot_comparison.py
include/nmpc_controller.hpp
src/nmpc_controller.cpp
docs/acados_nmpc_and_visualization_notes.md
```

### 修改文件

```text
CMakeLists.txt
src/main.cpp
include/csv_logger.hpp
src/csv_logger.cpp
.gitignore
CLAUDE.md
```

### 生成文件（不纳入 Git）

```text
acados_generated/
acados_ocp.json
build/
results_nmpc_cpp/
```

这些已加入 `.gitignore`。

---

## 12. 注意事项与后续建议

### 12.1 环境变量

运行 C++ 可执行文件前需要：

```bash
export LD_LIBRARY_PATH=/home/wwlwwl/acados/lib:$LD_LIBRARY_PATH
```

否则会出现：

```text
error while loading shared libraries: libhpipm.so: cannot open shared object file
```

### 12.2 OCP 参数固化

当前 OCP 权重、控制约束、horizon 长度在 `generate_acados_solver.py` 中固化。若修改这些参数，应重新运行：

```bash
python3 scripts/generate_acados_solver.py
cmake --build build
```

### 12.3 角度误差实现差异

Python CasADi 原型使用：

```text
atan2(sin(theta - theta_ref), cos(theta - theta_ref))
```

acados 当前实现使用 `sin/cos` 残差形式：

```text
sin(theta) - sin(theta_ref)
cos(theta) - cos(theta_ref)
```

该形式在轨迹跟踪的小角度误差场景下表现稳定，且避免直接在 `yref` 中引入自由符号。

### 12.4 下一步建议

1. 对 circle/eight/sine 做更系统的指标统计表。
2. 将求解时间指标写入 summary CSV，方便后续报告引用。
3. 增加障碍物约束，进入避障 NMPC。
4. 比较 `SQP_RTI` 与 `SQP` 的精度/耗时差异。
5. 将 `LD_LIBRARY_PATH` 写入项目运行说明或提供 `scripts/run_with_acados_env.sh`。
