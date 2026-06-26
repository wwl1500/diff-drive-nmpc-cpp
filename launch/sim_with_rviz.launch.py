import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_nmpc_cpp')
    default_params = os.path.join(pkg_share, 'config', 'diff_drive_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'diff_drive.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'diff_drive.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('controller_type', default_value='nmpc'),
        DeclareLaunchArgument('trajectory_type', default_value='circle'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # 仿真节点（发布 /odom，订阅 /cmd_vel）
        Node(
            package='diff_drive_nmpc_cpp',
            executable='diff_drive_sim_node',
            name='diff_drive_sim_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'initial_x': 2.0, 'initial_y': -0.2, 'initial_theta': 0.0},
            ],
        ),

        # 控制节点（订阅 /odom，发布 /cmd_vel）
        Node(
            package='diff_drive_nmpc_cpp',
            executable='diff_drive_nmpc_node',
            name='diff_drive_nmpc_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'controller_type': LaunchConfiguration('controller_type'),
                    'trajectory_type': LaunchConfiguration('trajectory_type'),
                },
            ],
        ),

        # robot_state_publisher（URDF 发布）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # RViz2（可通过 use_rviz:=false 关闭）
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
