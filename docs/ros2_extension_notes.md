# 第六步项目笔记：ROS2 扩展

本文记录将差速小车 NMPC 控制器封装为 ROS2 节点的实现过程。项目从纯 CMake 仿真框架扩展为同时支持 colcon 构建的 ROS2 包，同时保持原有纯 CMake 构建完全不受影响。

对应项目指南：第六步 —— ROS2 扩展。

---

## 1. 阶段目标

1. 将 `diff_drive_core` 库封装为 ROS2 节点，订阅里程计、发布速度指令。
2. 创建仿真节点，用 `DiffDriveModel` 模拟植物动力学，支持无实机端到端测试。
3. 保持双构建模式：纯 CMake 与 colcon 均可正常构建。
4. 提供 launch 文件，一条命令启动控制 + 仿真 + rviz2 可视化。
5. 通过 ROS2 参数切换控制器类型和轨迹类型。

---

## 2. 环境准备

### 2.1 ROS2 安装

系统已安装 ROS2 Jazzy Jalisco（LTS，支持至 2029 年 5 月），路径：

```text
/opt/ros/jazzy/
```

关键库确认存在：

```text
rclcpp, nav_msgs, geometry_msgs, tf2_ros, tf2_msgs,
visualization_msgs, robot_state_publisher, rviz2
```

构建工具 `colcon` 位于 `/usr/bin/colcon`，环境变量已自动设置：

```bash
ROS_VERSION=2
ROS_DISTRO=jazzy
AMENT_PREFIX_PATH=/opt/ros/jazzy
```

### 2.2 acados 与 ROS2 共存

acados 动态库路径需在运行时可用。项目通过两种机制保证：

- **RPATH**：`CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE`，编译时将 acados 库绝对路径写入 ELF RPATH。
- **ament 环境钩子**：`ament_env_hooks/diff_drive_nmpc_cpp.dsv.in`，在 `source install/setup.bash` 时自动将 `$ACADOS_SOURCE_PATH/lib` 追加到 `LD_LIBRARY_PATH`。

---

## 3. 架构设计

### 3.1 双构建检测机制

核心思路：在 `CMakeLists.txt` 末尾追加 `find_package(ament_cmake QUIET)`。当通过 colcon 构建时 `ament_cmake_FOUND` 为 TRUE，走 ROS2 路径；纯 `cmake` 时为 FALSE，整个 ROS2 块被跳过。

```cmake
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  # ROS2 节点构建...
  ament_package()
endif()
```

原有内容（`diff_drive_core` 库、`diff_drive_sim` 可执行文件、acados 集成）保持完全不变。

### 3.2 节点架构

三个运行组件通过标准 ROS2 话题通信：

```text
参考轨迹（内部生成）
      │
      ▼
diff_drive_nmpc_node ──/cmd_vel──→ diff_drive_sim_node
      ↑                                    │
      └──────────── /odom ────────────────┘
```

| 节点 | 角色 | 订阅 | 发布 |
|------|------|------|------|
| `diff_drive_nmpc_node` | 控制器 | `/odom`（nav_msgs/Odometry） | `/cmd_vel`（Twist）、`/reference_path`（Path）、`/robot_path`（Path） |
| `diff_drive_sim_node` | 植物模拟 | `/cmd_vel`（Twist） | `/odom`（Odometry）+ TF（odom→base_link） |

控制节点不发布 TF（遵循 ROS2 惯例：TF 由里程计源负责）。仿真节点同时发布 Odometry 和 TF。

### 3.3 话题与 QoS

| 话题 | QoS | 说明 |
|------|-----|------|
| `/odom` | SensorDataQoS（best-effort） | 与真实传感器一致 |
| `/cmd_vel` | SystemDefaultsQoS（reliable） | 速度指令不能丢失 |
| `/reference_path` | SystemDefaultsQoS | 可视化用，低频 |
| `/robot_path` | SystemDefaultsQoS | 累积轨迹，低频 |

---

## 4. CMakeLists.txt 修改

在现有内容末尾追加，不改动原有任何一行。关键设计点：

### 4.1 ROS2 依赖查找

```cmake
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
```

### 4.2 链接方式

使用 `target_link_libraries` 替代过时的 `ament_target_dependencies`：

```cmake
add_executable(diff_drive_nmpc_node src/ros2/diff_drive_nmpc_node.cpp)
target_link_libraries(diff_drive_nmpc_node PRIVATE
  diff_drive_core
  rclcpp::rclcpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  tf2_ros::tf2_ros
  tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
  visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
```

各 ROS2 包的 CMake target 命名规则为 `<pkg>::<pkg>__rosidl_typesupport_cpp`（消息包）或 `<pkg>::<pkg>`（库包如 `rclcpp`、`tf2_ros`）。

`ament_target_dependencies` 内部使用 plain 签名的 `target_link_libraries`，与 `PRIVATE` 关键字签名不兼容，故全部替换为显式 `target_link_libraries`。

### 4.3 RPATH 配置

```cmake
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
```

acados 的 `libacados.so`、`libhpipm.so`、`libblasfeo.so` 通过 `diff_drive_core` 的 `PUBLIC` 链接传递到 ROS2 可执行文件，RPATH 自动嵌入这些库的绝对路径。

### 4.4 安装规则

```cmake
install(TARGETS diff_drive_nmpc_node diff_drive_sim_node diff_drive_sim
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)
```

`diff_drive_sim`（原有纯 CMake 可执行文件）也一并安装，使其可通过 `ros2 run` 调用。

---

## 5. 环境钩子

文件 `ament_env_hooks/diff_drive_nmpc_cpp.dsv.in`：

```
prepend-non-duplicate;LD_LIBRARY_PATH;@ACADOS_SOURCE_PATH@/lib
```

CMakeLists.txt 中通过 `configure_file` 将 `@ACADOS_SOURCE_PATH@` 替换为实际路径：

```cmake
configure_file(ament_env_hooks/${PROJECT_NAME}.dsv.in
               ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.dsv @ONLY)
ament_environment_hooks("${PROJECT_BINARY_DIR}/${PROJECT_NAME}.dsv")
```

效果：`source install/setup.bash` 后 acados 库路径自动可用，无需手动设置 `LD_LIBRARY_PATH`。

---

## 6. 控制节点实现

文件：`src/ros2/diff_drive_nmpc_node.cpp`

### 6.1 类结构

```cpp
class DiffDriveNmpcNode : public rclcpp::Node {
    // ROS2 基础设施
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_path_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 核心组件（来自 diff_drive_core）
    std::unique_ptr<ControllerBase> controller_;
    ReferenceGenerator reference_generator_;
};
```

### 6.2 ROS2 参数（13 个）

构造函数中声明并读取，对应 `ReferenceGeneratorConfig` 和 `NmpcControllerConfig` 已有字段：

| 参数 | 类型 | 默认值 | 对应结构体字段 |
|------|------|--------|----------------|
| `controller_type` | string | `"nmpc"` | 控制器选择 |
| `trajectory_type` | string | `"circle"` | `TrajectoryType` 枚举 |
| `dt` | double | `0.05` | `NmpcControllerConfig::dt` |
| `horizon_steps` | int | `20` | `NmpcControllerConfig::horizon_steps` |
| `circle_radius` | double | `2.0` | `ReferenceGeneratorConfig::circle_radius` |
| `circle_period` | double | `20.0` | `ReferenceGeneratorConfig::circle_period` |
| `eight_amplitude` | double | `2.0` | `ReferenceGeneratorConfig::eight_amplitude` |
| `eight_period` | double | `20.0` | `ReferenceGeneratorConfig::eight_period` |
| `sine_forward_velocity` | double | `0.3` | `ReferenceGeneratorConfig::sine_forward_velocity` |
| `sine_amplitude` | double | `1.0` | `ReferenceGeneratorConfig::sine_amplitude` |
| `sine_omega` | double | `0.5` | `ReferenceGeneratorConfig::sine_omega` |
| `frame_id` | string | `"odom"` | TF/消息坐标系 |
| `child_frame_id` | string | `"base_link"` | TF/消息坐标系 |

`ReferenceGeneratorConfig` 已有全部轨迹参数字段，无需修改头文件。

### 6.3 控制器创建

复用 `main.cpp` 中的多态工厂模式，通过 `#ifdef HAS_NMPC` 条件编译：

```cpp
if (controller_type_ == "nmpc") {
#ifdef HAS_NMPC
    NmpcControllerConfig nmpc_config;
    nmpc_config.dt = dt_;
    nmpc_config.horizon_steps = horizon_steps_;
    controller_ = std::make_unique<NmpcController>(nmpc_config);
#else
    RCLCPP_FATAL(get_logger(), "...");
    rclcpp::shutdown();
#endif
} else {
    controller_ = std::make_unique<PurePursuitController>();
}
```

运行时未编译 NMPC 时请求 `nmpc` 控制器会报 FATAL 并安全退出。

### 6.4 Odometry 回调

从 `nav_msgs/Odometry` 提取 `State`：

```cpp
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    current_state_.theta = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    has_odom_ = true;
}
```

四元数转欧拉角使用标准 yaw 提取公式，假设无 roll/pitch（2D 平面机器人）。

### 6.5 控制定时器回调

20Hz wall timer，核心循环：

```cpp
void control_timer_callback() {
    if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "等待 /odom 里程计数据...");
        return;
    }
    const auto ref_horizon = reference_generator_.getHorizonReference(
        sim_time_, dt_, horizon_steps_, trajectory_type_);
    const auto control = controller_->computeControl(current_state_, ref_horizon);

    publish_cmd_vel(control);
    publish_reference_path(sim_time_);
    publish_robot_path(current_state_);
    publish_diagnostics(control, sim_time_, ref, current_state_);

    sim_time_ += dt_;
}
```

时间管理：内部计数器 `sim_time_` 从 0 开始每步累加 `dt_`，与原 `main.cpp` 一致。实机模式下可改用 `now().seconds()`（代码中用 TODO 标注）。

### 6.6 可视化辅助

**参考轨迹路径**：每步发布未来 20 秒、200 个采样点的 Path 消息，用于 rviz2 显示绿色参考曲线。

**实际轨迹路径**：累积 `robot_path_msg_`，每步追加当前位姿，显示红色实际轨迹。

**诊断日志**：每秒输出一次跟踪误差和控制量。NMPC 模式额外输出求解时间：

```text
t=0.00 pos_err=0.2000 v=-0.011 w=1.500 solve=0.42ms ok=1
```

通过 `dynamic_cast<NmpcController*>` 访问 NMPC 专有接口（`lastSolveTimeMs()`、`lastSolveOk()`），仅在 `#ifdef HAS_NMPC` 下编译。

---

## 7. 仿真节点实现

文件：`src/ros2/diff_drive_sim_node.cpp`

### 7.1 类结构

```cpp
class DiffDriveSimNode : public rclcpp::Node {
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<DiffDriveModel> model_;
    State state_;
    geometry_msgs::msg::Twist latest_cmd_vel_;
};
```

### 7.2 构造函数

`DiffDriveModel` 只接受 `double dt` 参数的显式构造函数，不能默认构造。使用 `std::unique_ptr` 延迟初始化：

```cpp
const double dt = get_parameter("dt").as_double();
model_ = std::make_unique<DiffDriveModel>(dt);
```

参数：`dt`、`initial_x`、`initial_y`、`initial_theta`、`frame_id`、`child_frame_id`。

默认初始位姿 `(2.0, -0.2, 0.0)` 与 `main.cpp:100` 一致（圆形轨迹参考起点偏移 -0.2m）。

### 7.3 仿真定时器回调

```cpp
void sim_timer_callback() {
    Control control{latest_cmd_vel_.linear.x, latest_cmd_vel_.angular.z};
    state_ = model_->step(state_, control);

    // 发布 Odometry
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.pose.pose.position.x = state_.x;
    odom_msg.pose.pose.position.y = state_.y;
    odom_msg.pose.pose.orientation.z = std::sin(state_.theta / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(state_.theta / 2.0);
    odom_pub_->publish(odom_msg);

    // 广播 TF
    tf_broadcaster_->sendTransform(tf);
}
```

四元数使用 `sin(theta/2)` / `cos(theta/2)`，假设纯 Z 轴旋转（2D 平面正确）。

---

## 8. 配置文件

### 8.1 默认参数 `config/diff_drive_params.yaml`

```yaml
diff_drive_nmpc_node:
  ros__parameters:
    controller_type: "nmpc"
    trajectory_type: "circle"
    dt: 0.05
    horizon_steps: 20
    circle_radius: 2.0
    circle_period: 20.0
    ...

diff_drive_sim_node:
  ros__parameters:
    dt: 0.05
    initial_x: 2.0
    initial_y: -0.2
    initial_theta: 0.0
    ...
```

### 8.2 RViz2 配置 `config/diff_drive.rviz`

显示内容：
- Grid（odom 坐标系 XY 平面）
- Path `/reference_path`（绿色，alpha 0.8，线宽 0.03）
- Path `/robot_path`（红色，alpha 1.0，线宽 0.05）
- TF 坐标系显示
- 固定坐标系 `odom`，俯视视角（Pitch = π/2）

### 8.3 URDF `urdf/diff_drive.urdf`

最小差速车模型，仅用于 rviz2 可视化：

```xml
<link name="base_link">
  <visual>
    <geometry><box size="0.3 0.2 0.1"/></geometry>
    <material name="blue"><color rgba="0.2 0.4 0.8 1.0"/></material>
  </visual>
</link>
<joint name="odom_joint" type="fixed">
  <parent link="odom"/><child link="base_link"/>
</joint>
```

---

## 9. Launch 文件

### 9.1 `launch/controller_only.launch.py`

实机用，仅启动控制节点：

```bash
ros2 launch diff_drive_nmpc_cpp controller_only.launch.py controller_type:=nmpc
```

加载参数文件，支持 `controller_type` launch argument。

### 9.2 `launch/sim_with_rviz.launch.py`

仿真测试，启动四个组件：

```bash
ros2 launch diff_drive_nmpc_cpp sim_with_rviz.launch.py \
  controller_type:=nmpc trajectory_type:=circle
```

组件列表：
1. `diff_drive_sim_node`（仿真器）
2. `diff_drive_nmpc_node`（控制器）
3. `robot_state_publisher`（URDF 发布）
4. `rviz2`（可视化，可通过 `use_rviz:=false` 关闭）

支持 launch arguments：`controller_type`、`trajectory_type`、`use_rviz`、`params_file`。

URDF 内容在 launch 文件中直接读取并传入 `robot_state_publisher` 的 `robot_description` 参数：

```python
with open(urdf_file, 'r') as f:
    robot_description = f.read()
```

---

## 10. 设计决策记录

### 10.1 控制节点不发布 TF

遵循 ROS2 惯例：控制节点是控制器，不是定位系统。TF 由里程计源（仿真节点或实机驱动）负责。避免了仿真模式下两个节点同时发布同一 TF 的冲突。

### 10.2 单线程执行器

使用默认的 `rclcpp::spin()`（单线程执行器），确保 acados 求解器在单一线程中调用。Odometry 回调仅写入 POD 结构体 `State`，不存在并发问题。

### 10.3 参数启动时读取

参数在构造函数中一次性读取，运行时不支持动态变更。切换控制器类型需重启节点。代码中用 TODO 标注后续可添加 `set_on_parameters_set_callback`。

### 10.4 `ament_target_dependencies` → `target_link_libraries`

`ament_target_dependencies` 内部使用 plain 签名的 `target_link_libraries`，与 `PRIVATE` 关键字签名冲突。改用显式 `target_link_libraries(target PRIVATE rclcpp::rclcpp ...)` 是 Jazzy 的推荐做法。

### 10.5 DiffDriveModel 延迟初始化

`DiffDriveModel` 只有显式构造函数 `DiffDriveModel(double dt)`，不能作为类成员默认构造。使用 `std::unique_ptr<DiffDriveModel>` 在读取参数后构造。

---

## 11. package.xml

```xml
<package format="3">
  <name>diff_drive_nmpc_cpp</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_msgs</depend>
  <depend>visualization_msgs</depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

`<build_type>ament_cmake</build_type>` 告诉 colcon 此为 ament 包。`<exec_depend>` 确保 `rosdep` 可解析运行时依赖。

---

## 12. 验证结果

### 12.1 纯 CMake 构建（回归测试）

```bash
rm -rf build && cmake -S . -B build -DACADOS_SOURCE_PATH=$HOME/acados
cmake --build build
```

结果：

```text
[100%] Built target diff_drive_sim
[100%] Built target diff_drive_nmpc_node
[100%] Built target diff_drive_sim_node
```

原有功能正常：

```bash
./build/diff_drive_sim circle nmpc
# NMPC stats: solves=801, failures=0, avg_time_ms=0.110004
```

### 12.2 colcon 构建

```bash
mkdir -p /tmp/ros2_ws/src
ln -s ~/diff-drive-nmpc-cpp /tmp/ros2_ws/src/diff_drive_nmpc_cpp
cd /tmp/ros2_ws
colcon build --packages-select diff_drive_nmpc_cpp
```

结果：

```text
Starting >>> diff_drive_nmpc_cpp
Finished <<< diff_drive_nmpc_cpp [12.2s]
```

三个可执行文件注册成功：

```bash
ros2 pkg executables diff_drive_nmpc_cpp
# diff_drive_nmpc_cpp diff_drive_nmpc_node
# diff_drive_nmpc_cpp diff_drive_sim
# diff_drive_nmpc_cpp diff_drive_sim_node
```

### 12.3 节点启动测试

仿真节点：

```text
[INFO] [diff_drive_sim_node]: 仿真节点已启动 (dt=0.050, 初始位姿=[0.00, 0.00, 0.00])
```

控制节点（PurePursuit，无 odom）：

```text
[INFO] [diff_drive_nmpc_node]: 控制器已创建: pure_pursuit
[INFO] [diff_drive_nmpc_node]: 控制节点已启动 (controller=pure_pursuit, trajectory=circle, dt=0.050)
[WARN] [diff_drive_nmpc_node]: 等待 /odom 里程计数据...
```

控制节点（NMPC，无 odom）：

```text
[INFO] [diff_drive_nmpc_node]: 控制器已创建: nmpc
[INFO] [diff_drive_nmpc_node]: 控制节点已启动 (controller=nmpc, trajectory=circle, dt=0.050)
[WARN] [diff_drive_nmpc_node]: 等待 /odom 里程计数据...
```

### 12.4 端到端集成测试

```bash
ros2 launch diff_drive_nmpc_cpp sim_with_rviz.launch.py \
  use_rviz:=false controller_type:=pure_pursuit
```

输出（4 秒运行）：

```text
[diff_drive_sim_node]: 仿真节点已启动 (dt=0.050, 初始位姿=[2.00, -0.20, 0.00])
[diff_drive_nmpc_node]: 控制器已创建: pure_pursuit
[diff_drive_nmpc_node]: 控制节点已启动 (controller=pure_pursuit, trajectory=circle, dt=0.050)
[diff_drive_nmpc_node]: t=0.00 pos_err=0.2000 v=0.400 w=1.500
[diff_drive_nmpc_node]: t=1.05 pos_err=0.7044 v=0.400 w=1.175
[diff_drive_nmpc_node]: t=2.10 pos_err=0.9569 v=0.400 w=0.165
[diff_drive_nmpc_node]: t=3.10 pos_err=1.1782 v=0.400 w=0.129
```

NMPC 端到端测试：

```text
[diff_drive_nmpc_node]: 控制器已创建: nmpc
[diff_drive_nmpc_node]: t=0.00 pos_err=0.2000 v=-0.011 w=1.500 solve=0.42ms ok=1
[diff_drive_nmpc_node]: t=1.05 pos_err=1.0348 v=-0.500 w=1.500 solve=0.43ms ok=1
[diff_drive_nmpc_node]: t=2.10 pos_err=2.0377 v=-0.500 w=1.500 solve=0.29ms ok=1
[diff_drive_nmpc_node]: t=3.10 pos_err=2.4928 v=1.000 w=-1.500 solve=0.79ms ok=1
```

NMPC 求解器在 ROS2 节点中正常工作，求解时间 0.3-0.9ms，全部成功。

---

## 13. 当前文件变更总结

### 新增文件

```text
package.xml                              ROS2 包清单
ament_env_hooks/diff_drive_nmpc_cpp.dsv.in  acados LD_LIBRARY_PATH 环境钩子
src/ros2/diff_drive_nmpc_node.cpp        ROS2 控制节点（~195 行）
src/ros2/diff_drive_sim_node.cpp         ROS2 仿真节点（~100 行）
config/diff_drive_params.yaml            默认 ROS2 参数
config/diff_drive.rviz                   RViz2 配置
launch/controller_only.launch.py         实机用启动文件
launch/sim_with_rviz.launch.py           仿真测试启动文件
urdf/diff_drive.urdf                     最小差速车 URDF
docs/ros2_extension_notes.md             本文
```

### 修改文件

```text
CMakeLists.txt    追加 ament 守卫块 + ROS2 构建目标
.gitignore        新增 install/, log/, colcon.meta
CLAUDE.md         更新当前阶段、目录结构、ROS2 章节
README.md         新增 ROS2 章节、更新项目结构和后续计划
```

### 未修改文件（设计目标）

```text
include/common_types.hpp
include/controller_base.hpp
include/nmpc_controller.hpp
include/pure_pursuit_controller.hpp
include/reference_generator.hpp
include/diff_drive_model.hpp
include/csv_logger.hpp
include/math_utils.hpp
src/main.cpp
src/*.cpp（全部核心实现）
```

---

## 14. 注意事项与后续建议

### 14.1 colcon 工作区设置

项目根目录即 ROS2 包。通过符号链接放入 colcon 工作区：

```bash
mkdir -p ~/ros2_ws/src
ln -s ~/diff-drive-nmpc-cpp ~/ros2_ws/src/diff_drive_nmpc_cpp
```

colcon 构建产物在 `~/ros2_ws/build/`、`install/`、`log/`，不在项目目录内。

### 14.2 acados 库运行时

两种方式确保 acados 动态库可用（可同时使用）：

- RPATH：编译时嵌入绝对路径，单机最可靠。
- ament 环境钩子：`source install/setup.bash` 后自动设置。

### 14.3 单线程保证

`rclcpp::spin()` 使用单线程执行器。Odometry 回调和控制定时器回调不会并发执行。若改用多线程执行器，需为 acados 求解器加锁。

### 14.4 时间管理

当前使用内部计数器 `sim_time_` 从 0 开始递增。实机模式下应改用 `now().seconds()` 计算墙钟时间。代码中标注了 TODO。

### 14.5 外部轨迹源

当前 `use_internal_reference` 默认为 `true`，轨迹由内置 `ReferenceGenerator` 生成。后续可扩展为订阅外部 `nav_msgs/Path` 话题，需要解决时间参数化问题。

### 14.6 下一步建议

1. 接入 Gazebo 物理仿真，替代 `diff_drive_sim_node`。
2. 添加动态参数回调，支持运行时切换控制器/轨迹。
3. 集成 Nav2 导航栈，将 NMPC 作为局部控制器插件。
4. 添加 rosbag2 录制支持，替代 CsvLogger。
5. 性能测试：对比 ROS2 节点与纯 CMake 仿真的求解时间开销。
