"""Indoor planning + control (ROS 2 Humble, Nav2) for the Yunle Ackermann car.

  route_navigator ──NavigateThroughPoses──► bt_navigator ─► planner_server (Smac Hybrid-A*, Reeds-Shepp)
                                                        └─► controller_server (MPPI Ackermann | RPP)
                                                                   │ /cmd_vel_nav  (also behavior_server BackUp)
                                                                   ▼
                                                          collision_monitor ─► /cmd_vel
                                                                   ▼
                              ackermann_cmd_converter ─► Autoware Control/Gear topics
                                                       ─► Yunle chassis bridge

Requires: indoor_localization running (TF map -> base_link, /localization/kinematic_state,
/localization/util/downsample/pointcloud, /indoor_localization/health).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    lc = lambda k: LaunchConfiguration(k).perform(context)  # noqa
    share = get_package_share_directory('indoor_navigation')
    nav2_params = os.path.join(share, 'config', 'nav2_indoor.param.yaml')
    own_params = os.path.join(share, 'config', 'indoor_navigation_nodes.param.yaml')
    ctrl_params = [nav2_params]
    if lc('controller') == 'rpp':
        ctrl_params.append(os.path.join(share, 'config', 'controller_rpp.param.yaml'))
    nav_map = lc('nav_map_yaml') or os.path.join(share, 'maps', 'indoor_nav_2d.yaml')
    mask = lc('speed_mask_yaml') or os.path.join(share, 'maps', 'speed_mask.yaml')
    stations = lc('stations_file') or os.path.join(share, 'config', 'stations.yaml')
    bt_dir = os.path.join(share, 'behavior_trees')
    common = {'use_sim_time': lc('use_sim_time') == 'true'}
    odom_remap = [('odom', '/localization/kinematic_state')]

    return [
        # ---------------- maps ----------------
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[nav2_params, common, {'yaml_filename': nav_map}]),
        Node(package='nav2_map_server', executable='map_server', name='filter_mask_server', output='screen',
             parameters=[nav2_params, common, {'yaml_filename': mask}]),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server', output='screen', parameters=[nav2_params, common]),
        # ---------------- Nav2 core ----------------
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[nav2_params, common]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=ctrl_params + [common], remappings=odom_remap + [('cmd_vel', '/cmd_vel_nav')]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[nav2_params, common], remappings=odom_remap + [('cmd_vel', '/cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[nav2_params, common, {
                 'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'navigate_through_poses_ackermann.xml'),
                 'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_to_pose_ackermann.xml')}],
             remappings=odom_remap),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             output='screen', parameters=[nav2_params, common]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[nav2_params, common]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_filters',
             output='screen', parameters=[nav2_params, common]),
        # ---------------- vehicle side ----------------
        Node(package='indoor_navigation', executable='ackermann_cmd_converter', name='ackermann_cmd_converter',
             output='screen', parameters=[own_params, common]),
        Node(package='indoor_navigation', executable='route_navigator', name='route_navigator',
             output='screen', parameters=[own_params, common, {'stations_file': stations}]),
        # Standalone indoor launch still needs the sole final-command owner.
        # The campus hybrid wrapper disables this instance and starts its own
        # manager with outdoor initial authority.
        Node(package='indoor_navigation', executable='hybrid_mode_manager',
             name='hybrid_mode_manager', output='screen',
             condition=IfCondition(LaunchConfiguration('launch_mode_manager')),
             parameters=[own_params, common, {
                 'initial_mode': LaunchConfiguration('initial_mode')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='mppi', description='mppi | rpp'),
        DeclareLaunchArgument('nav_map_yaml', default_value='', description='default: package maps/indoor_nav_2d.yaml'),
        DeclareLaunchArgument('speed_mask_yaml', default_value='', description='default: package maps/speed_mask.yaml'),
        DeclareLaunchArgument('stations_file', default_value='', description='default: package config/stations.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_mode_manager', default_value='true'),
        DeclareLaunchArgument('initial_mode', default_value='indoor'),
        OpaqueFunction(function=_setup),
    ])
