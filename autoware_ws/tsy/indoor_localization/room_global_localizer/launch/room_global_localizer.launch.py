import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('room_global_localizer'),
                               'config', 'room_global_localizer.param.yaml')
    lidar_cfg = os.path.join(get_package_share_directory('room_global_localizer'),
                             'config', 'mid360s_config.json')
    rviz_cfg = os.path.join(get_package_share_directory('room_global_localizer'),
                            'rviz', 'room_global_localizer.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('param_file', default_value=default_cfg),
        DeclareLaunchArgument('launch_driver', default_value='true'),
        DeclareLaunchArgument('launch_sensor_tf', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('publish_debug_map', default_value='true'),
        DeclareLaunchArgument('use_autoware_ndt', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('auto_start_delay', default_value='5.0'),
        DeclareLaunchArgument(
            'pointcloud_topic', default_value='/sensing/lidar/mid360s/pointcloud'),
        DeclareLaunchArgument(
            'pointcloud_map_path',
            default_value=(
                '/home/tang/tsy/indoor_localization/'
                'map_20261117_denoised/pointcloud_map.pcd')),
        DeclareLaunchArgument(
            'loc_map_yaml',
            default_value=(
                '/home/tang/tsy/indoor_localization/'
                'map_20261117_denoised/localization_map.yaml')),
        DeclareLaunchArgument(
            'nav_map_yaml',
            default_value=(
                '/home/tang/tsy/indoor_localization/'
                'map_20261117_denoised/nav_map.yaml')),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='mid360s_driver', output='screen',
             condition=IfCondition(LaunchConfiguration('launch_driver')),
             parameters=[{
                 'xfer_format': 0,
                 'multi_topic': 0,
                 'data_src': 0,
                 'publish_freq': 10.0,
                 'output_data_type': 0,
                 'frame_id': 'livox_front/livox_front_base_link',
                 'user_config_path': lidar_cfg,
             }],
             remappings=[('livox/lidar', '/sensing/lidar/mid360s/pointcloud')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='mid360s_base_tf', output='screen',
             condition=IfCondition(LaunchConfiguration('launch_sensor_tf')),
             arguments=['--x', '0.60', '--y', '0.01', '--z', '0.57',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'base_link',
                        '--child-frame-id', 'livox_front/livox_front_base_link']),
        Node(package='room_global_localizer', executable='room_global_localizer',
             name='room_global_localizer', output='screen',
             parameters=[
                 LaunchConfiguration('param_file'),
                 {'use_autoware_ndt': ParameterValue(
                     LaunchConfiguration('use_autoware_ndt'), value_type=bool),
                  'auto_start': ParameterValue(
                      LaunchConfiguration('auto_start'), value_type=bool),
                  'auto_start_delay': ParameterValue(
                      LaunchConfiguration('auto_start_delay'), value_type=float),
                  'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                  'loc_map_yaml': LaunchConfiguration('loc_map_yaml'),
                  'nav_map_yaml': LaunchConfiguration('nav_map_yaml')},
             ]),
        Node(package='pcl_ros', executable='pcd_to_pointcloud',
             name='task5_map_publisher', output='screen',
             condition=IfCondition(LaunchConfiguration('publish_debug_map')),
             parameters=[{
                 'file_name': LaunchConfiguration('pointcloud_map_path'),
                 'tf_frame': 'map',
                 'publishing_period_ms': 1000,
             }],
             remappings=[('cloud_pcd', '/room_global_localizer/map_cloud')]),
        # Codex/VS Code is installed as a Snap on this machine. Its inherited
        # GTK/GDK paths make system RViz load an incompatible core20 libc, so
        # launch RViz with those GUI-only Snap variables removed.
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen',
            cmd=['/usr/bin/env',
                 '-u', 'SNAP', '-u', 'SNAP_LIBRARY_PATH',
                 '-u', 'GTK_PATH', '-u', 'GTK_EXE_PREFIX',
                 '-u', 'GTK_IM_MODULE_FILE', '-u', 'GDK_PIXBUF_MODULEDIR',
                 '-u', 'GDK_PIXBUF_MODULE_FILE', '-u', 'GIO_MODULE_DIR',
                 '-u', 'LOCPATH', '-u', 'XDG_DATA_HOME', '-u', 'XDG_DATA_DIRS',
                 'rviz2', '-d', rviz_cfg]),
    ])
