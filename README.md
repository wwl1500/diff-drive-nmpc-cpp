# diff-drive-nmpc-cpp

基于 C++17 与 acados 的差速移动机器人 NMPC 轨迹跟踪控制项目。

项目当前包含：

- 纯 C++ 差速小车仿真框架
- Pure Pursuit baseline 控制器
- Python + CasADi NMPC 原型
- acados 生成的 C 求解器
- C++ `NmpcController` 封装
- CSV 数据记录、求解时间统计和 PP/NMPC 对比可视化
- ROS2 Jazzy 扩展（控制节点 + 仿真节点 + rviz2 可视化）

详细实现笔记见：`docs/acados_nmpc_and_visualization_notes.md`。

---

## 1. 功能概览

- 差速小车离散运动学模型
- 圆轨迹、八字轨迹、S 型轨迹参考生成
- `ControllerBase` 统一控制器接口
- `PurePursuitController` baseline
- `NmpcController`（C++ 调用 acados C 求解器）
- 轨迹、控制量、误差、NMPC 求解时间 CSV 记录
- 单控制器绘图：轨迹、误差、控制输入
- PP vs NMPC 对比绘图：误差对比、求解时间、综合指标柱状图
- ROS2 Jazzy 扩展：控制节点、仿真节点、launch 文件、rviz2 可视化

---

## 2. 环境依赖

### 2.1 C++ 与构建工具

- CMake >= 3.16
- C++17 编译器
- acados 源码编译安装

本机默认 acados 安装路径：

```bash
export ACADOS_SOURCE_PATH=/home/wwlwwl/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_PATH/lib:$LD_LIBRARY_PATH
```

运行 `diff_drive_sim` 前必须设置 `LD_LIBRARY_PATH`，否则可能找不到 `libhpipm.so`、`libblasfeo.so` 等动态库。

### 2.2 ROS2（可选）

ROS2 扩展需要 Jazzy 桌面版安装（`/opt/ros/jazzy/`）。不安装 ROS2 时纯 CMake 构建不受影响。

依赖的 ROS2 包：

- `rclcpp`、`nav_msgs`、`geometry_msgs`、`tf2_ros`、`visualization_msgs`
- 运行时：`robot_state_publisher`、`rviz2`、`launch_ros`

### 2.3 Python 环境

Python 虚拟环境：`.venv`

常用依赖：

- `casadi`
- `acados_template`
- `numpy`
- `matplotlib`
- `scipy`
- `jinja2`

激活环境：

```bash
source .venv/bin/activate
```

---

## 3. 构建

### 3.1 纯 CMake 构建

```bash
export ACADOS_SOURCE_PATH=/home/wwlwwl/acados
cmake -S . -B build -DACADOS_SOURCE_PATH=$ACADOS_SOURCE_PATH
cmake --build build
```

如果只想构建不含 NMPC 的版本：

```bash
cmake -S . -B build -DENABLE_NMPC=OFF
cmake --build build
```

### 3.2 colcon 构建（ROS2）

```bash
mkdir -p ~/ros2_ws/src
ln -s ~/diff-drive-nmpc-cpp ~/ros2_ws/src/diff_drive_nmpc_cpp
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
export ACADOS_SOURCE_PATH=~/acados
colcon build --packages-select diff_drive_nmpc_cpp \
  --cmake-args -DACADOS_SOURCE_PATH=$ACADOS_SOURCE_PATH
source install/setup.bash
```

---

## 4. 运行 C++ 仿真

命令格式：

```bash
./build/diff_drive_sim [circle|eight|sine] [pp|nmpc]
```

示例：

```bash
export LD_LIBRARY_PATH=/home/wwlwwl/acados/lib:$LD_LIBRARY_PATH

# Pure Pursuit baseline
./build/diff_drive_sim circle pp
./build/diff_drive_sim eight pp
./build/diff_drive_sim sine pp

# C++ acados NMPC
./build/diff_drive_sim circle nmpc
./build/diff_drive_sim eight nmpc
./build/diff_drive_sim sine nmpc
```

默认参数：

```text
trajectory = circle
controller = pp
```

输出目录：

```text
Pure Pursuit: results/
NMPC:         results_nmpc_cpp/<trajectory>/
```

---

## 5. 生成 acados C 求解器

通常只有修改 OCP 参数、代价函数、约束或模型后才需要重新生成。

```bash
source .venv/bin/activate
export ACADOS_SOURCE_PATH=/home/wwlwwl/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_PATH/lib:$LD_LIBRARY_PATH
python3 scripts/generate_acados_solver.py
```

生成目录：

```text
acados_generated/
```

本项目的 `CMakeLists.txt` 会直接编译生成目录中的 `.c` 文件，不要求单独构建共享库。

如需单独编译 acados 生成的共享库：

```bash
cd acados_generated
make shared_lib
```

---

## 6. Python + CasADi NMPC 原型

原型入口：

```text
prototype/nmpc_casadi.py
```

运行：

```bash
source .venv/bin/activate
python3 prototype/nmpc_casadi.py circle
python3 prototype/nmpc_casadi.py eight
python3 prototype/nmpc_casadi.py sine
```

默认输出目录：

```text
results_nmpc_python/circle
results_nmpc_python/eight
results_nmpc_python/sine
```

原型用于验证建模、权重和控制效果；C++ NMPC 使用 acados 生成求解器实现实时调用。

---

## 7. CSV 输出

通用输出：

```text
trajectory.csv: time,x,y,theta,x_ref,y_ref,theta_ref
control.csv:    time,v,omega
error.csv:      time,ex,ey,etheta,position_error
```

NMPC 额外输出：

```text
solve_time.csv: time,solve_time_ms,solve_ok
```

`solve_time.csv` 只在 NMPC 模式下生成；Pure Pursuit 模式不会生成空文件。

---

## 8. 绘图

### 8.1 单控制器结果绘图

```bash
python3 scripts/plot_results.py results_nmpc_cpp/circle
```

生成：

```text
trajectory_plot.png
error_plot.png
control_plot.png
```

### 8.2 PP vs NMPC 对比图

先运行 PP 和 NMPC：

```bash
export LD_LIBRARY_PATH=/home/wwlwwl/acados/lib:$LD_LIBRARY_PATH
./build/diff_drive_sim circle pp
./build/diff_drive_sim circle nmpc
```

再生成对比图：

```bash
source .venv/bin/activate
python3 scripts/plot_comparison.py results results_nmpc_cpp/circle
```

生成：

```text
solve_time_plot.png
error_comparison_plot.png
comparison_bar_plot.png
```

---

## 9. 运动学模型

状态量：

```text
x = [p_x, p_y, theta]
```

控制量：

```text
u = [v, omega]
```

离散时间模型：

```text
p_x(k+1) = p_x(k) + dt * v(k) * cos(theta(k))
p_y(k+1) = p_y(k) + dt * v(k) * sin(theta(k))
theta(k+1) = theta(k) + dt * omega(k)
```

默认仿真参数：

```text
dt = 0.05 s
total_time = 40.0 s
horizon_steps = 20
v_min = -0.5 m/s
v_max = 1.0 m/s
omega_min = -1.5 rad/s
omega_max = 1.5 rad/s
```

---

## 10. NMPC OCP 摘要

C++ acados NMPC 使用增广状态：

```text
x_aug = [px, py, theta, v_prev, omega_prev]
u = [v, omega]
```

使用 `v_prev` 与 `omega_prev` 表达控制增量惩罚：

```text
v(k) - v(k-1)
omega(k) - omega(k-1)
```

主要权重：

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

求解器配置：

```text
SQP_RTI + PARTIAL_CONDENSING_HPIPM
N = 20
tf = 1.0 s
```

---

## 11. 项目结构

```text
.
├── CMakeLists.txt
├── package.xml                 # ROS2 包清单（colcon 构建用）
├── CLAUDE.md
├── README.md
├── include/
│   ├── common_types.hpp
│   ├── controller_base.hpp
│   ├── csv_logger.hpp
│   ├── diff_drive_model.hpp
│   ├── math_utils.hpp
│   ├── nmpc_controller.hpp
│   ├── pure_pursuit_controller.hpp
│   └── reference_generator.hpp
├── src/
│   ├── csv_logger.cpp
│   ├── diff_drive_model.cpp
│   ├── main.cpp
│   ├── nmpc_controller.cpp
│   ├── pure_pursuit_controller.cpp
│   ├── reference_generator.cpp
│   └── ros2/
│       ├── diff_drive_nmpc_node.cpp    # ROS2 控制节点
│       └── diff_drive_sim_node.cpp     # ROS2 仿真节点
├── launch/
│   ├── controller_only.launch.py       # 实机用
│   └── sim_with_rviz.launch.py         # 仿真测试
├── config/
│   ├── diff_drive_params.yaml          # ROS2 默认参数
│   └── diff_drive.rviz                 # RViz2 配置
├── urdf/
│   └── diff_drive.urdf                 # 差速车 URDF
├── ament_env_hooks/
│   └── diff_drive_nmpc_cpp.dsv.in      # acados LD_LIBRARY_PATH 钩子
├── scripts/
│   ├── generate_acados_solver.py
│   ├── plot_comparison.py
│   ├── plot_results.py
│   └── run_experiments.py
├── prototype/
│   ├── nmpc_casadi.py
│   └── requirements.txt
├── docs/
│   └── acados_nmpc_and_visualization_notes.md
├── acados_generated/       # 生成文件，gitignore
├── results/                # PP 输出，gitignore 部分文件
├── results_nmpc_cpp/       # C++ NMPC 输出，gitignore
├── results_nmpc_python/    # Python NMPC 输出，gitignore
└── results_experiments/    # 批量实验结果，gitignore
```

---

## 12. 当前验证结果

已验证：

```text
circle NMPC: solves=801, failures=0
eight  NMPC: solves=801, failures=0
sine   NMPC: solves=801, failures=0
```

circle 轨迹对比中：

```text
Pure Pursuit 平均位置误差 ≈ 1.6446 m
NMPC 平均位置误差 ≈ 0.0041 m
```

---

## 13. 第五步：批量实验设计

为便于在 `circle/eight/sine` 三类轨迹上系统对比 Pure Pursuit 与 NMPC 控制器，项目提供了批量实验脚本，可一键运行全部组合并生成统一 `summary.csv` 与对比图。

### 运行方式

```bash
export ACADOS_SOURCE_PATH=~/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_PATH/lib:$LD_LIBRARY_PATH

# 构建
cmake --build build

# 运行全部实验
python3 scripts/run_experiments.py --output results_experiments

# 仅重新生成图表和 summary（跳过仿真）
python3 scripts/run_experiments.py --output results_experiments --skip-run

# 仅运行仿真并生成 summary（跳过绘图）
python3 scripts/run_experiments.py --output results_experiments --skip-plots
```

仿真入口也支持指定输出目录（可用于任意单组实验）：

```bash
./build/diff_drive_sim circle pp results_experiments/circle/pp
./build/diff_drive_sim circle nmpc results_experiments/circle/nmpc
```

### 输出结构

```text
results_experiments/
  circle/
    pp/             # PP 仿真 CSV + 单控制器图
    nmpc/           # NMPC 仿真 CSV + 单控制器图
    comparison/     # PP vs NMPC 对比图
  eight/
    pp/ nmpc/ comparison/
  sine/
    pp/ nmpc/ comparison/
  summary.csv       # 全部实验的统一指标表
```

### summary.csv 字段说明

每行一组 `trajectory + controller`：

| 字段 | 说明 |
|------|------|
| `mean_position_error` | 平均位置误差 [m] |
| `rms_position_error` | RMS 位置误差 [m] |
| `max_position_error` | 最大位置误差 [m] |
| `mean_abs_theta_error` | 平均绝对角度误差 [rad] |
| `rms_theta_error` | RMS 角度误差 [rad] |
| `max_abs_theta_error` | 最大绝对角度误差 [rad] |
| `solve_time_mean_ms` | NMPC 平均求解时间 [ms] |
| `solve_time_p95_ms` | NMPC 求解时间 P95 [ms] |
| `solve_failure_count` | NMPC 求解失败次数 |

Pure Pursuit 的求解时间字段留空。

---

## 14. ROS2 扩展（第六步）

项目根目录即 ROS2 包，采用双构建模式：`find_package(ament_cmake QUIET)` 自动检测 ROS2 环境，未安装 ROS2 时纯 CMake 构建不受影响。

### 14.1 节点架构

```text
参考轨迹（内部生成）──→ diff_drive_nmpc_node ──→ /cmd_vel ──→ diff_drive_sim_node
                              ↑                                              │
                            /odom ←──────────────────────────────────────────┘
```

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `diff_drive_nmpc_node` | `/odom` | `/cmd_vel`、`/reference_path`、`/robot_path` | NMPC / PurePursuit 控制 |
| `diff_drive_sim_node` | `/cmd_vel` | `/odom` + TF | DiffDriveModel 植物模拟 |

控制节点可单独用于实机（订阅外部里程计），仿真节点仅用于离线测试。

### 14.2 运行

```bash
# colcon 构建（参照 3.2 节）
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$HOME/acados/lib:$LD_LIBRARY_PATH

# 仿真测试（默认 NMPC + 圆轨迹）
ros2 launch diff_drive_nmpc_cpp sim_with_rviz.launch.py

# 切换控制器和轨迹
ros2 launch diff_drive_nmpc_cpp sim_with_rviz.launch.py \
  controller_type:=pure_pursuit trajectory_type:=eight

# 关闭 rviz2
ros2 launch diff_drive_nmpc_cpp sim_with_rviz.launch.py use_rviz:=false

# 实机用（仅控制节点，需外部发布 /odom）
ros2 launch diff_drive_nmpc_cpp controller_only.launch.py controller_type:=nmpc
```

### 14.3 ROS2 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `controller_type` | `nmpc` | `nmpc` 或 `pure_pursuit` |
| `trajectory_type` | `circle` | `circle` / `eight` / `sine` |
| `dt` | `0.05` | 控制周期 [s] |
| `horizon_steps` | `20` | NMPC 预测步数 |
| `circle_radius` | `2.0` | 圆形轨迹半径 [m] |
| `circle_period` | `20.0` | 圆形轨迹周期 [s] |
| `sine_amplitude` | `1.0` | 正弦幅度 [m] |
| `sine_omega` | `0.5` | 正弦频率 [rad/s] |
| `frame_id` | `odom` | 固定坐标系 |
| `child_frame_id` | `base_link` | 机器人坐标系 |

---

## 15. 后续计划

- 调参比较 `SQP_RTI` 与 `SQP`
- 添加障碍物约束
- 接入 Gazebo 物理仿真
