"""Launch scan conversion, wheel odometry, EKF and optional AMCL."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Launch scan conversion, odometry fusion and optional AMCL."""
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_amcl = LaunchConfiguration("use_amcl")
    lock_map_to_odom = LaunchConfiguration("lock_map_to_odom")
    soft_map_correction = LaunchConfiguration("soft_map_correction")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")

    bringup_share = FindPackageShare("yunle_nav2_bringup")
    vehicle_params = PathJoinSubstitution([bringup_share, "config", "vehicle_adapter.yaml"])
    ekf_params = PathJoinSubstitution([bringup_share, "config", "ekf.yaml"])
    scan_params = PathJoinSubstitution(
        [bringup_share, "config", "pointcloud_to_laserscan.yaml"]
    )
    amcl_tf_broadcast = PythonExpression(
        ["'", lock_map_to_odom, "'.lower() != 'true'"]
    )
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={"tf_broadcast": amcl_tf_broadcast},
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_amcl", default_value="true"),
            DeclareLaunchArgument("lock_map_to_odom", default_value="true"),
            DeclareLaunchArgument("soft_map_correction", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/sensing/lidar/concatenated/pointcloud",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[scan_params, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("~/input/pointcloud", pointcloud_topic),
                    ("~/output/laserscan", "/scan"),
                ],
            ),
            Node(
                package="yunle_nav2_vehicle",
                executable="vehicle_odometry",
                name="yunle_vehicle_odometry",
                output="screen",
                parameters=[vehicle_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odometry/filtered")],
            ),
            Node(
                package="yunle_nav2_vehicle",
                executable="map_odom_lock",
                name="map_odom_lock",
                output="screen",
                condition=IfCondition(lock_map_to_odom),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "soft_correction_enabled": soft_map_correction,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
                    )
                ),
                condition=IfCondition(use_amcl),
                launch_arguments={
                    "map": map_file,
                    "params_file": configured_params,
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                    # Humble Nav2 evaluates this value in a PythonExpression.
                    # It therefore requires Python's capitalized boolean literal.
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
