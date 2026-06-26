import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_nmpc_cpp')
    default_params = os.path.join(pkg_share, 'config', 'diff_drive_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('controller_type', default_value='nmpc'),

        Node(
            package='diff_drive_nmpc_cpp',
            executable='diff_drive_nmpc_node',
            name='diff_drive_nmpc_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'controller_type': LaunchConfiguration('controller_type')},
            ],
        ),
    ])
