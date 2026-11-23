from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    baudrate = LaunchConfiguration('gins2000_baudrate', default='921600')
    frame_id = LaunchConfiguration('gins2000_frame_id', default='gins2000_link')
    topic = LaunchConfiguration('gins2000_topic', default='gins2000')

    gins2000_node = Node(
        package='gins2000',
        executable='gins2000_node',
        name='gins2000_node',
        output='screen',
        parameters=[{
            'port': port,
            'gins2000_baudrate': baudrate,
            'gins2000_frame_id': frame_id,
            'gins2000_topic': topic,
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gins2000_broadcaster',
        arguments=['0.0', '0.0', '0.23', '0', '0', '0', 'base_footprint', 'gins2000_link']
    )

    return LaunchDescription([
        gins2000_node,
        static_tf,
    ])
