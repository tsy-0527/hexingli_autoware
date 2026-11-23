from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hunter_autoware_bridge',
            executable='bridge_node',
            name='hunter_autoware_bridge',
            output='screen',
            parameters=[{
                'wheelbase': 0.648,
                'max_steering_angle': 0.52,
                'max_linear_speed': 4.0,
                'max_steer_angle':  0.52,
                'control_mode_can': 1,    # ← 实车确认后改这里
            }]
        )
    ])
