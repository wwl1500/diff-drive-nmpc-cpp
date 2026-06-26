cmake_minimum_required(VERSION 3.16)
project(diff_drive_nmpc_cpp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- acados 集成 ---
option(ENABLE_NMPC "构建 acados NMPC 控制器" ON)

if(ENABLE_NMPC)
    # acados 安装路径
    if(NOT DEFINED ACADOS_SOURCE_PATH)
        if(DEFINED ENV{ACADOS_SOURCE_PATH})
            set(ACADOS_SOURCE_PATH $ENV{ACADOS_SOURCE_PATH})
        else()
            set(ACADOS_SOURCE_PATH "$ENV{HOME}/acados")
        endif()
    endif()

    set(ACADOS_GENERATED_DIR "${CMAKE_SOURCE_DIR}/acados_generated")

    if(NOT EXISTS "${ACADOS_GENERATED_DIR}/acados_solver_diff_drive.c")
        message(FATAL_ERROR
            "acados 求解器未生成，请先运行: python3 scripts/generate_acados_solver.py")
    endif()

    set(ACADOS_INCLUDE_DIRS
        ${ACADOS_SOURCE_PATH}/include
        ${ACADOS_SOURCE_PATH}/include/acados
        ${ACADOS_SOURCE_PATH}/include/blasfeo/include
        ${ACADOS_SOURCE_PATH}/include/hpipm/include
        ${ACADOS_GENERATED_DIR}
    )

    # 编译生成的 acados 求解器为静态库
    file(GLOB ACADOS_COST_SRCS  "${ACADOS_GENERATED_DIR}/diff_drive_cost/*.c")
    file(GLOB ACADOS_MODEL_SRCS "${ACADOS_GENERATED_DIR}/diff_drive_model/*.c")
    set(ACADOS_SOLVER_SRC "${ACADOS_GENERATED_DIR}/acados_solver_diff_drive.c")

    add_library(acados_ocp_solver STATIC
        ${ACADOS_COST_SRCS}
        ${ACADOS_MODEL_SRCS}
        ${ACADOS_SOLVER_SRC}
    )
    target_include_directories(acados_ocp_solver PUBLIC ${ACADOS_INCLUDE_DIRS})
    target_compile_options(acados_ocp_solver PRIVATE -fPIC -O2)
endif()

# --- 核心库 ---
set(CORE_SOURCES
    src/diff_drive_model.cpp
    src/reference_generator.cpp
    src/pure_pursuit_controller.cpp
    src/csv_logger.cpp
)

if(ENABLE_NMPC)
    list(APPEND CORE_SOURCES src/nmpc_controller.cpp)
endif()

add_library(diff_drive_core ${CORE_SOURCES})
target_include_directories(diff_drive_core PUBLIC include)

if(ENABLE_NMPC)
    target_include_directories(diff_drive_core PUBLIC ${ACADOS_INCLUDE_DIRS})
    target_link_libraries(diff_drive_core PUBLIC
        acados_ocp_solver
        ${ACADOS_SOURCE_PATH}/lib/libacados.so
        ${ACADOS_SOURCE_PATH}/lib/libhpipm.so
        ${ACADOS_SOURCE_PATH}/lib/libblasfeo.so
        m
    )
    target_compile_definitions(diff_drive_core PUBLIC HAS_NMPC=1)
endif()

target_compile_options(diff_drive_core PRIVATE -Wall -Wextra -Wpedantic)

# --- 可执行文件 ---
add_executable(diff_drive_sim src/main.cpp)
target_link_libraries(diff_drive_sim PRIVATE diff_drive_core)
target_compile_options(diff_drive_sim PRIVATE -Wall -Wextra -Wpedantic)

# --- ROS2 / ament 检测 ---
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  # RPATH 设置（acados 共享库运行时可找到）
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  # 控制节点
  add_executable(diff_drive_nmpc_node src/ros2/diff_drive_nmpc_node.cpp)
  target_link_libraries(diff_drive_nmpc_node PRIVATE
    diff_drive_core
    rclcpp::rclcpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    tf2_ros::tf2_ros
    tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)

  # 仿真节点
  add_executable(diff_drive_sim_node src/ros2/diff_drive_sim_node.cpp)
  target_link_libraries(diff_drive_sim_node PRIVATE
    diff_drive_core
    rclcpp::rclcpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    tf2_ros::tf2_ros
    tf2_msgs::tf2_msgs__rosidl_typesupport_cpp)

  # 安装
  install(TARGETS diff_drive_nmpc_node diff_drive_sim_node diff_drive_sim
          DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
  install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)

  # acados 环境钩子
  if(ENABLE_NMPC)
    configure_file(ament_env_hooks/${PROJECT_NAME}.dsv.in
                   ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.dsv @ONLY)
    ament_environment_hooks("${PROJECT_BINARY_DIR}/${PROJECT_NAME}.dsv")
  endif()

  ament_package()
endif()
