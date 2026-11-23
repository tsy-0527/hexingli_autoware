"""Complete localization-mode bringup for the real Yunle vehicle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose sensing, localization and navigation for the real vehicle."""
    map_file = LaunchConfiguration("map")
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_gins = LaunchConfiguration("launch_gins")
    lock_map_to_odom = LaunchConfiguration("lock_map_to_odom")
    soft_map_correction = LaunchConfiguration("soft_map_correction")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    share = FindPackageShare("yunle_nav2_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to a grid-map YAML file"),
            DeclareLaunchArgument("launch_vehicle", default_value="true"),
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_gins", default_value="true"),
            DeclareLaunchArgument("lock_map_to_odom", default_value="true"),
            DeclareLaunchArgument("soft_map_correction", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([share, "launch", "sensors_vehicle.launch.py"])
                ),
                launch_arguments={
                    "launch_vehicle": launch_vehicle,
                    "launch_lidar": launch_lidar,
                    "launch_gins": launch_gins,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([share, "launch", "localization.launch.py"])
                ),
                launch_arguments={
                    "map": map_file,
                    "use_amcl": "true",
                    "use_sim_time": use_sim_time,
                    "lock_map_to_odom": lock_map_to_odom,
                    "soft_map_correction": soft_map_correction,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([share, "launch", "navigation.launch.py"])
                ),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
