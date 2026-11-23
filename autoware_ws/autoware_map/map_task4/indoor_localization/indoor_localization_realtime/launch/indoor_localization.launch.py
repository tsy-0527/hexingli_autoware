"""Indoor real-time localization: MID-360 + NDT + gyro_odometer + EKF (Autoware nodes)
plus the custom indoor nodes of this package.

    MID-360 ─► mid360_preprocessor ─► NDT ─► degeneracy_covariance_relay ─► EKF ─► stop_filter ─► /localization/kinematic_state
                     ▲  deskew twist                                      ▲
    VelocityReport ─► velocity_report_converter ─► gyro_odometer ◄─ IMU   │ twist
                                                        └──────────────────┘
    /room_global_localizer/pose ─► initial_pose_bridge ─► pose_initializer (NDT align) ─► EKF reset

Not started here: vehicle interface (VelocityReport), IMU driver, MID-360 driver,
static TFs (base_link -> lidar / imu), room_global_localizer.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _include(pkg, launch_file, args):
    path = os.path.join(get_package_share_directory(pkg), 'launch', launch_file)
    return IncludeLaunchDescription(AnyLaunchDescriptionSource(path),
                                    launch_arguments={k: str(v) for k, v in args.items()}.items())


def _setup(context):
    lc = lambda k: LaunchConfiguration(k).perform(context)  # noqa
    pre = lc('autoware_pkg_prefix')                         # 'autoware_' (2024+) or '' (older releases)
    own = get_package_share_directory('indoor_localization')
    nodes_param = os.path.join(own, 'config', 'indoor_localization_nodes.param.yaml')
    ndt_param = lc('ndt_param_path') or os.path.join(own, 'config', 'ndt_scan_matcher.indoor.param.yaml')
    ekf_param = lc('ekf_param_path') or os.path.join(own, 'config', 'ekf_localizer.indoor.param.yaml')
    vehicle_twist = '/sensing/vehicle_velocity_converter/twist_with_covariance'
    actions = []

    # ---------------- map ----------------
    actions.append(Node(
        package=f'{pre}map_loader',
        executable='autoware_pointcloud_map_loader' if pre else 'pointcloud_map_loader',
        name='pointcloud_map_loader', namespace='map', output='screen',
        parameters=[{
            'enable_whole_load': True,
            'enable_downsampled_whole_load': False,
            'enable_partial_load': True,
            'enable_selected_load': False,
            'leaf_size': 3.0,
            'pcd_paths_or_directory': [lc('pointcloud_map_path')],
            'pcd_metadata_path': lc('pointcloud_map_metadata_path'),   # empty / missing -> single-file mode
        }],
        remappings=[('output/pointcloud_map', '/map/pointcloud_map'),
                    ('output/pointcloud_map_metadata', '/map/pointcloud_map_metadata'),
                    ('service/get_partial_pcd_map', '/map/get_partial_pointcloud_map'),
                    ('service/get_differential_pcd_map', '/map/get_differential_pointcloud_map'),
                    ('service/get_selected_pcd_map', '/map/get_selected_pointcloud_map')]))

    # ---------------- twist estimator ----------------
    twist = [PushRosNamespace('localization'), PushRosNamespace('twist_estimator')]
    if lc('use_velocity_converter') == 'true':
        twist.append(Node(package='indoor_localization', executable='velocity_report_converter',
                          name='velocity_report_converter', output='screen',
                          parameters=[nodes_param, {'input_topic': lc('velocity_report_topic'),
                                                    'output_topic': vehicle_twist}]))
    twist.append(_include(f'{pre}gyro_odometer', 'gyro_odometer.launch.xml', {
        'input_vehicle_twist_with_covariance_topic': vehicle_twist,
        'input_imu_topic': lc('imu_topic'),
        'output_twist_raw_topic': '/localization/twist_estimator/gyro_twist_raw',
        'output_twist_with_covariance_raw_topic': '/localization/twist_estimator/gyro_twist_with_covariance_raw',
        'output_twist_topic': '/localization/twist_estimator/twist',
        'output_twist_with_covariance_topic': '/localization/twist_estimator/twist_with_covariance',
    }))
    actions.append(GroupAction(twist))

    # ---------------- pointcloud preprocessing ----------------
    actions.append(GroupAction([
        PushRosNamespace('localization'), PushRosNamespace('util'),
        Node(package='indoor_localization', executable='mid360_preprocessor', name='mid360_preprocessor',
             output='screen', parameters=[nodes_param, {'input_topic': lc('lidar_topic')}]),
    ]))

    # ---------------- pose estimator: NDT + covariance relay ----------------
    actions.append(GroupAction([
        PushRosNamespace('localization'), PushRosNamespace('pose_estimator'),
        _include(f'{pre}ndt_scan_matcher', 'ndt_scan_matcher.launch.xml', {
            'input_pointcloud': '/localization/util/downsample/pointcloud',
            'input_initial_pose_topic': '/localization/pose_twist_fusion_filter/biased_pose_with_covariance',
            'input_regularization_pose_topic': '/sensing/gnss/pose_with_covariance',
            'input_service_trigger_node': '/localization/pose_estimator/trigger_node',
            'output_pose_topic': '/localization/pose_estimator/pose',
            'output_pose_with_covariance_topic': '/localization/pose_estimator/ndt_pose_with_covariance_raw',
            'client_map_loader': '/map/get_differential_pointcloud_map',
            'param_file': ndt_param,
        }),
        Node(package='indoor_localization', executable='degeneracy_covariance_relay',
             name='degeneracy_covariance_relay', output='screen', parameters=[nodes_param]),
    ]))

    # ---------------- pose twist fusion: EKF + stop filter + twist2accel ----------------
    actions.append(GroupAction([
        PushRosNamespace('localization'), PushRosNamespace('pose_twist_fusion_filter'),
        _include(f'{pre}ekf_localizer', 'ekf_localizer.launch.xml', {
            'input_initial_pose_name': '/initialpose3d',
            'input_pose_with_cov_name': '/localization/pose_estimator/pose_with_covariance',
            'input_twist_with_cov_name': '/localization/twist_estimator/twist_with_covariance',
            'output_odom_name': '/localization/pose_twist_fusion_filter/kinematic_state',
            'output_pose_name': 'pose',
            'output_pose_with_covariance_name': '/localization/pose_with_covariance',
            'output_biased_pose_name': 'biased_pose',
            'output_biased_pose_with_covariance_name': 'biased_pose_with_covariance',
            'output_twist_name': 'twist',
            'output_twist_with_covariance_name': 'twist_with_covariance',
            'param_file': ekf_param,
        }),
        _include(f'{pre}stop_filter', 'stop_filter.launch.xml', {
            'input_odom_name': '/localization/pose_twist_fusion_filter/kinematic_state',
            'output_odom_name': '/localization/kinematic_state',
        }),
        _include(f'{pre}twist2accel', 'twist2accel.launch.xml', {
            'in_odom': '/localization/kinematic_state',
            'in_twist': '/localization/twist_estimator/twist_with_covariance',
            'out_accel': '/localization/acceleration',
        }),
    ]))

    # ---------------- initialization + monitoring ----------------
    actions.append(GroupAction([
        PushRosNamespace('localization'), PushRosNamespace('util'),
        _include(f'{pre}pose_initializer', 'pose_initializer.launch.xml', {
            'user_defined_initial_pose/enable': 'false',
            'ndt_enabled': 'true',
            'gnss_enabled': 'false',          # indoor: no GNSS
            'ekf_enabled': 'true',
            'yabloc_enabled': 'false',
            'stop_check_enabled': 'true',
            'sub_gnss_pose_cov': '/sensing/gnss/pose_with_covariance',
            'stop_check_twist_topic': vehicle_twist,
            'gnss_initial_pose_auto_fix_target': 'pointcloud_map',
        }),
        Node(package='indoor_localization', executable='initial_pose_bridge', name='initial_pose_bridge',
             output='screen', parameters=[nodes_param]),
        Node(package='indoor_localization', executable='localization_health_monitor',
             name='localization_health_monitor', output='screen', parameters=[nodes_param]),
    ]))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('pointcloud_map_path', description='indoor_map_leveled_clean.pcd'),
        DeclareLaunchArgument('pointcloud_map_metadata_path', default_value='/nonexistent/pointcloud_map_metadata.yaml'),
        DeclareLaunchArgument('lidar_topic', default_value='/sensing/lidar/mid360s/pointcloud'),
        DeclareLaunchArgument('imu_topic', default_value='/sensing/imu/imu_data',
                              description='sensor_msgs/Imu from GINS2000 (frame must have TF to base_link)'),
        DeclareLaunchArgument('velocity_report_topic', default_value='/vehicle/status/velocity_status'),
        DeclareLaunchArgument('use_velocity_converter', default_value='true',
                              description='false if autoware vehicle_velocity_converter already runs'),
        DeclareLaunchArgument('autoware_pkg_prefix', default_value='autoware_',
                              description="'autoware_' for 2024+ releases, '' for older ones"),
        DeclareLaunchArgument('ndt_param_path', default_value=''),
        DeclareLaunchArgument('ekf_param_path', default_value=''),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
