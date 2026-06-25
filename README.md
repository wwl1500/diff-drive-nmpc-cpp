# diff-drive-nmpc-cpp

基于 C++ 与 acados 的差速移动机器人 NMPC 轨迹跟踪与避障控制。

当前阶段实现的是纯 C++17 仿真框架和 Pure Pursuit baseline，后续再接入 Python/CasADi NMPC 原型、acados 求解器、ROS2 和 Gazebo。

## 当前功能

- 差速小车离散运动学模型
- 圆轨迹、八字轨迹、S 型轨迹参考生成
- 控制器基类接口
- Pure Pursuit 轨迹跟踪控制器
- CSV 数据记录
- Python/matplotlib 绘图脚本

## 运动学模型

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

## 构建

```bash
cmake -S . -B build
cmake --build build
```

## 运行仿真

默认运行圆轨迹：

```bash
./build/diff_drive_sim
```

指定轨迹类型：

```bash
./build/diff_drive_sim circle
./build/diff_drive_sim eight
./build/diff_drive_sim sine
```

仿真结果输出到 `results/`：

```text
results/trajectory.csv
results/control.csv
results/error.csv
```

CSV 字段：

```text
trajectory.csv: time,x,y,theta,x_ref,y_ref,theta_ref
control.csv: time,v,omega
error.csv: time,ex,ey,etheta,position_error
```

## 绘图

需要 Python 和 matplotlib：

```bash
python3 scripts/plot_results.py
```

脚本会生成：

```text
results/trajectory_plot.png
results/error_plot.png
results/control_plot.png
```

## 项目结构

```text
.
├── CMakeLists.txt
├── include/
│   ├── common_types.hpp
│   ├── math_utils.hpp
│   ├── diff_drive_model.hpp
│   ├── reference_generator.hpp
│   ├── controller_base.hpp
│   ├── pure_pursuit_controller.hpp
│   └── csv_logger.hpp
├── src/
│   ├── diff_drive_model.cpp
│   ├── reference_generator.cpp
│   ├── pure_pursuit_controller.cpp
│   ├── csv_logger.cpp
│   └── main.cpp
├── scripts/
│   └── plot_results.py
└── results/
```

## 后续计划

- Python + CasADi 实现 NMPC 原型
- C++ 封装 NMPCController
- 接入 acados 生成的 C 求解器
- 加入输入约束和障碍物约束实验
- 与 Pure Pursuit 做定量对比
- 扩展到 ROS2 / Gazebo
