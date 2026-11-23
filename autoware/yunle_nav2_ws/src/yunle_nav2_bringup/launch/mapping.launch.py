"""Launch the real vehicle sensing stack and SLAM Toolbox mapping mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch sensing, odometry and SLAM Toolbox for grid-map creation."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    share = FindPackageShare("yunle_nav2_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([share, "launch", "sensors_vehicle.launch.py"])
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([share, "launch", "localization.launch.py"])
                ),
                launch_arguments={
                    "use_amcl": "false",
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": PathJoinSubstitution(
                        [share, "config", "slam_toolbox.yaml"]
                    ),
                }.items(),
            ),
        ]
    )
