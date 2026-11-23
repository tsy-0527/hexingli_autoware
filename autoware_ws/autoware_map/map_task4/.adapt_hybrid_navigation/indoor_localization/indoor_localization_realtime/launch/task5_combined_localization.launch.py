"""One localization chain: room initialization -> Autoware NDT -> EKF tracking.

Autoware starts map/sensing/vehicle/API once, while this package supplies the
indoor-tuned localization component.  The room global localizer calls the same
NDT align and initialization services, so no bridge or duplicate NDT is used.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    autoware_launch = os.path.join(
        get_package_share_directory('autoware_launch'), 'launch', 'autoware.launch.xml')
    indoor_share = get_package_share_directory('indoor_localization')
    localization_launch = os.path.join(
        indoor_share, 'launch', 'task5_localization_component.launch.xml')
    room_launch = os.path.join(
        get_package_share_directory('room_global_localizer'),
        'launch', 'room_global_localizer.launch.py')
    navigation_launch = os.path.join(
        get_package_share_directory('indoor_navigation'),
        'launch', 'indoor_navigation.launch.py')
    nodes_param = os.path.join(
        indoor_share, 'config', 'indoor_localization_nodes.param.yaml')

    perception = LaunchConfiguration('launch_perception')
    planning = LaunchConfiguration('launch_planning')
    control = LaunchConfiguration('launch_control')
    map_path = LaunchConfiguration('map_path')
    grid_map_path = LaunchConfiguration('grid_map_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path',
            default_value=(
                '/home/tang/autoware_map/map_task4/indoor_localization/'
                'map_20261117_denoised')),
        # 室内栅格用旧室内地图按 legacy_room_registration（2026-11-17 重新配准）变换后的版本，
        # 含南门门前区域；整个校园的 denoised 栅格会让 Nav2 在室外也能规划。
        DeclareLaunchArgument(
            'grid_map_path',
            default_value=(
                '/home/tang/autoware_map/map_task4/indoor_localization/'
                'map_20261117_legacy_indoor')),
        DeclareLaunchArgument('launch_perception', default_value='false'),
        DeclareLaunchArgument('launch_planning', default_value='false'),
        DeclareLaunchArgument('launch_control', default_value='false'),
        DeclareLaunchArgument('launch_system', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        # The legacy room polygon is registered into the new campus map frame,
        # so the original room-constrained automatic initialization is valid.
        DeclareLaunchArgument('auto_start', default_value='true'),
        # Map, API and sensing are ready within a few seconds in this reduced
        # stack. Eight seconds avoids a long period of expected "EKF not
        # activated" warnings being mistaken for initialization failure.
        DeclareLaunchArgument('auto_start_delay', default_value='8.0'),
        DeclareLaunchArgument('launch_navigation', default_value='true'),
        DeclareLaunchArgument('navigation_controller', default_value='rpp'),
        DeclareLaunchArgument('launch_mode_manager', default_value='true'),
        DeclareLaunchArgument('initial_mode', default_value='indoor'),
        DeclareLaunchArgument('gnss_enabled', default_value='false'),
        DeclareLaunchArgument('automatic_pose_initializer_enabled', default_value='false'),
        DeclareLaunchArgument(
            'pose_initializer_gnss_pose_topic',
            default_value='/sensing/gnss/pose_with_covariance'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(autoware_launch),
            launch_arguments={
                'map_path': map_path,
                'vehicle_model': 'yunle_vehicle',
                'sensor_model': 'livox_sensor_kit',
                'pose_source': 'ndt',
                'twist_source': 'gyro_odom',
                'gnss_enabled': LaunchConfiguration('gnss_enabled'),
                'automatic_pose_initializer_enabled': LaunchConfiguration(
                    'automatic_pose_initializer_enabled'),
                'pose_initializer_gnss_pose_topic': LaunchConfiguration(
                    'pose_initializer_gnss_pose_topic'),
                'use_sim_time': 'false',
                'system_run_mode': 'online',
                # The System diagnostics layer is not needed by this Nav2
                # composition. Keep API enabled: room_global_localizer calls
                # /api/localization/initialize for Autoware NDT/EKF startup.
                'launch_system': LaunchConfiguration('launch_system'),
                'launch_api': 'true',
                'launch_map': 'true',
                'launch_sensing': 'true',
                'launch_vehicle': 'true',
                # The indoor component below is the only localization stack.
                'launch_localization': 'false',
                'launch_perception': perception,
                # MID-360S indoor mode is lidar-only. Do not load camera
                # TensorRT traffic-light models that consume RViz GPU memory.
                'perception_mode': 'lidar',
                'use_traffic_light_recognition': 'false',
                'launch_planning': planning,
                'launch_control': control,
                'rviz': 'false',
            }.items()),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map_path': map_path,
                # Keep the extended Livox fields required by Autoware's crop /
                # voxel / random filter chain.  The custom PointXYZI output is
                # on a separate topic, so random_downsample_filter is the sole
                # publisher of the final NDT input.
                'input_pointcloud': '/sensing/lidar/concatenated/pointcloud',
                'vehicle_model': 'yunle_vehicle',
                # 修正：定位栈实际在这里启动（上面 Autoware 的 launch_localization=false），
                # GNSS 初始化相关参数必须传到这里，原来只传给了 Autoware，等于没生效。
                'gnss_enabled': LaunchConfiguration('gnss_enabled'),
                'automatic_pose_initializer_enabled': LaunchConfiguration(
                    'automatic_pose_initializer_enabled'),
                'pose_initializer_gnss_pose_topic': LaunchConfiguration(
                    'pose_initializer_gnss_pose_topic'),
            }.items()),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(room_launch),
            launch_arguments={
                'launch_driver': 'false',
                'launch_sensor_tf': 'false',
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'publish_debug_map': 'false',
                'use_autoware_ndt': 'true',
                'auto_start': LaunchConfiguration('auto_start'),
                'auto_start_delay': LaunchConfiguration('auto_start_delay'),
                'pointcloud_topic': '/sensing/lidar/concatenated/pointcloud',
                'pointcloud_map_path': PathJoinSubstitution(
                    [map_path, 'pointcloud_map.pcd']),
                'loc_map_yaml': PathJoinSubstitution(
                    [grid_map_path, 'localization_map.yaml']),
                'nav_map_yaml': PathJoinSubstitution(
                    [grid_map_path, 'nav_map.yaml']),
            }.items()),

        Node(
            package='indoor_localization',
            executable='mid360_preprocessor',
            name='mid360_preprocessor',
            namespace='/localization/util',
            output='screen',
            parameters=[nodes_param]),

        Node(
            package='indoor_localization',
            executable='localization_health_monitor',
            name='localization_health_monitor',
            namespace='/localization/util',
            output='screen',
            parameters=[nodes_param, {
                'state_topic': '/api/localization/initialization_state',
                'ndt_pose_topic': '/localization/pose_estimator/pose_with_covariance',
                'ekf_pose_topic': '/localization/pose_with_covariance',
            }]),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(navigation_launch),
            condition=IfCondition(LaunchConfiguration('launch_navigation')),
            launch_arguments={
                'controller': LaunchConfiguration('navigation_controller'),
                'use_sim_time': 'false',
                'nav_map_yaml': PathJoinSubstitution([grid_map_path, 'nav_map.yaml']),
                'speed_mask_yaml': PathJoinSubstitution([grid_map_path, 'speed_mask.yaml']),
                'keepout_mask_yaml': PathJoinSubstitution([grid_map_path, 'keepout_mask.yaml']),
                'launch_mode_manager': LaunchConfiguration('launch_mode_manager'),
                'initial_mode': LaunchConfiguration('initial_mode'),
            }.items()),

    ])
