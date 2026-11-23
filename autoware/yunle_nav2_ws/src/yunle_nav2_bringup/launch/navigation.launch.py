"""Launch Nav2 navigation servers and the safety-gated vehicle adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Launch Nav2 servers, Ackermann BT and the vehicle command adapter."""
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_share = FindPackageShare("yunle_nav2_bringup")
    ackermann_bt = PathJoinSubstitution(
        [bringup_share, "config", "navigate_ackermann.xml"]
    )
    ackermann_through_poses_bt = PathJoinSubstitution(
        [bringup_share, "config", "navigate_through_poses_ackermann.xml"]
    )
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "default_nav_to_pose_bt_xml": ackermann_bt,
            "default_nav_through_poses_bt_xml": ackermann_through_poses_bt,
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "params_file": configured_params,
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                    # Humble Nav2 evaluates this value in a PythonExpression.
                    # It therefore requires Python's capitalized boolean literal.
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="yunle_nav2_vehicle",
                executable="twist_to_control",
                name="yunle_twist_to_control",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [bringup_share, "config", "vehicle_adapter.yaml"]
                    ),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "rviz_launch.py"]
                    )
                ),
                condition=IfCondition(use_rviz),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
