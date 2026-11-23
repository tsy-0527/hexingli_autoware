from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('route_teach_repeat')
    default_cfg = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg),
        Node(
            package='route_teach_repeat',
            executable='route_player',
            name='route_player',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
