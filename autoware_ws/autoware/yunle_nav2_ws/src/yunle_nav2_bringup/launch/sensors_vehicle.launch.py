"""Launch the existing Yunle chassis, Livox driver, GINS2000 and static TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch calibrated vehicle description, sensing and chassis interfaces."""
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_gins = LaunchConfiguration("launch_gins")
    use_sim_time = LaunchConfiguration("use_sim_time")

    description_file = PathJoinSubstitution(
        [FindPackageShare("yunle_nav2_bringup"), "urdf", "yunle_nav2.urdf.xacro"]
    )
    mirror_file = PathJoinSubstitution(
        [FindPackageShare("yunle_vehicle_description"), "config", "mirror.param.yaml"]
    )
    imu_params = PathJoinSubstitution(
        [
            FindPackageShare("livox_sensor_kit_description"),
            "config",
            "imu_corrector.param_gins2000.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_vehicle", default_value="true"),
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_gins", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": ParameterValue(
                            Command(["xacro ", description_file]), value_type=str
                        ),
                    }
                ],
            ),
            ComposableNodeContainer(
                condition=IfCondition(launch_lidar),
                name="pointcloud_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
            ),
            # Autoware's sensor-kit launch files expect to be included under
            # /sensing.  Keep the component container at the root because the
            # pointcloud preprocessor loads components into
            # /pointcloud_container by its absolute name.
            GroupAction(
                actions=[
                    PushRosNamespace("sensing"),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("livox_sensor_kit_launch"),
                                    "launch",
                                    "lidar.launch.xml",
                                ]
                            )
                        ),
                        condition=IfCondition(launch_lidar),
                        launch_arguments={
                            "launch_driver": "true",
                            "vehicle_mirror_param_file": mirror_file,
                            "pointcloud_container_name": "pointcloud_container",
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("livox_sensor_kit_launch"),
                                    "launch",
                                    "gins2000.launch.xml",
                                ]
                            )
                        ),
                        condition=IfCondition(launch_gins),
                        launch_arguments={"launch_driver": "true"}.items(),
                    ),
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("autoware_imu_corrector"),
                            "launch",
                            "imu_corrector.launch.xml",
                        ]
                    )
                ),
                condition=IfCondition(launch_gins),
                launch_arguments={
                    "input_topic": "/sensing/imu/imu_raw",
                    "output_topic": "/sensing/imu/imu_data",
                    "param_file": imu_params,
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("chassis_driver"),
                            "launch",
                            "chassis_driver.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(launch_vehicle),
            ),
            # Reuse the existing bridge's physical AUTO handoff, watchdog,
            # command sign and feedback normalization.  Do not launch the
            # Autoware ADAPI mode gate; a Nav2-only system has no ADAPI server.
            Node(
                condition=IfCondition(launch_vehicle),
                package="yunle_vehicle_launch",
                executable="yunle_autoware_bridge.py",
                name="yunle_autoware_bridge",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("yunle_vehicle_launch"),
                            "config",
                            "yunle_autoware_bridge.param.yaml",
                        ]
                    )
                ],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("autoware_vehicle_velocity_converter"),
                            "launch",
                            "vehicle_velocity_converter.launch.xml",
                        ]
                    )
                ),
                condition=IfCondition(launch_vehicle),
                launch_arguments={
                    "input_vehicle_velocity_topic": "/vehicle/status/velocity_status",
                    "output_twist_with_covariance": (
                        "/sensing/vehicle_velocity_converter/twist_with_covariance"
                    ),
                }.items(),
            ),
        ]
    )
