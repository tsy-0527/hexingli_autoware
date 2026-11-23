"""Unified campus launch: lane driving outdoors, occupancy-grid Nav2 indoors."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    indoor_loc = os.path.join(
        get_package_share_directory('indoor_localization'), 'launch',
        'task5_combined_localization.launch.py')
    control_launch = os.path.join(
        get_package_share_directory('autoware_launch'), 'launch', 'components',
        'tier4_control_component.launch.xml')
    own_share = get_package_share_directory('indoor_navigation')
    own_params = os.path.join(own_share, 'config', 'indoor_navigation_nodes.param.yaml')
    switch_zones = os.path.join(own_share, 'config', 'hybrid_switch_zones.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path', default_value='/home/tang/autoware_map/map_task4',
            description='Combined NDT point cloud and outdoor Lanelet2 map'),
        DeclareLaunchArgument(
            'grid_map_path',
            # 修正：原来指向 map_20261117_denoised（整个校园 265 x 252 m 的栅格），Nav2 在室外也能规划。
            # 改成旧室内地图按 legacy_room_registration（2026-11-17 重新配准）变换后的栅格，
            # 只含室内和南门门前区域（主路、坡道），站点全部可达。
            default_value=(
                '/home/tang/autoware_map/map_task4/indoor_localization/'
                'map_20261117_legacy_indoor')),
        DeclareLaunchArgument('vehicle_model', default_value='yunle_vehicle'),
        DeclareLaunchArgument('vehicle_id', default_value='default'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_control', default_value='true'),
        DeclareLaunchArgument('launch_perception', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('navigation_controller', default_value='rpp'),
        DeclareLaunchArgument('control_module_preset', default_value='default'),
        DeclareLaunchArgument('check_external_emergency_heartbeat', default_value='false'),
        DeclareLaunchArgument('initial_mode', default_value='outdoor'),
        DeclareLaunchArgument('automatic_switching', default_value='false'),
        # 修正：GNSS 自动初始化默认关闭。当前地图 projector_type 是 Local，且 20261117 地图
        # 还没有和经纬度做配准（经纬度匹配点未录入），GNSS 位姿不能直接当这张图里的初值。
        # 原来这两个参数其实也没传到定位栈，实际一直是关闭的；这里把行为显式化。
        # 配准完成、并核对过 gnss_poser 输出与 NDT 位姿一致后，再用 gnss_init:=true 打开。
        DeclareLaunchArgument('gnss_init', default_value='false'),

        # Start vehicle/sensing/map once.  NDT + gyro odometry remains the
        # canonical localization over the shared campus frame.  GINS supplies
        # the GNSS initializer outdoors; it is not allowed to publish a second
        # canonical localization stream in parallel.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(indoor_loc),
            launch_arguments={
                'map_path': LaunchConfiguration('map_path'),
                'grid_map_path': LaunchConfiguration('grid_map_path'),
                'launch_perception': LaunchConfiguration('launch_perception'),
                'launch_planning': 'true',
                'launch_control': 'false',
                'launch_system': 'true',
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'launch_navigation': 'true',
                'navigation_controller': LaunchConfiguration('navigation_controller'),
                'launch_mode_manager': 'false',
                'gnss_enabled': LaunchConfiguration('gnss_init'),
                'automatic_pose_initializer_enabled': LaunchConfiguration('gnss_init'),
                # livox_sensor_kit intentionally keeps gnss_poser separate
                # from the gnss-ins pose estimator. In NDT mode this is the
                # GINS-backed GNSS pose available for initialization.
                'pose_initializer_gnss_pose_topic': (
                    '/sensing/gnss/gnss_poser/pose_with_covariance'),
                # A room-only FFT initialization must not run while the car is
                # outdoors. It remains available for explicit indoor recovery.
                'auto_start': 'false',
            }.items()),

        # The standard Autoware control gate remains in the path, but its
        # output is private.  hybrid_mode_manager is the sole publisher to the
        # vehicle bridge's final command topics.
        GroupAction(
            condition=IfCondition(LaunchConfiguration('launch_control')),
            actions=[
                SetRemap(
                    src='/control/command/control_cmd',
                    dst='/hybrid/outdoor/control_cmd'),
                SetRemap(
                    src='/control/command/gear_cmd',
                    dst='/hybrid/outdoor/gear_cmd'),
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(control_launch),
                    launch_arguments={
                        'module_preset': LaunchConfiguration('control_module_preset'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'vehicle_model': LaunchConfiguration('vehicle_model'),
                    }.items()),
            ]),

        Node(
            package='indoor_navigation',
            executable='hybrid_mode_manager',
            name='hybrid_mode_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_control')),
            parameters=[own_params, {
                'initial_mode': LaunchConfiguration('initial_mode'),
                'automatic_switching': LaunchConfiguration('automatic_switching'),
                'switch_zones_file': switch_zones,
            }]),
    ])

