import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    autoware_launch = os.path.join(
        get_package_share_directory('autoware_launch'), 'launch', 'autoware.launch.xml')
    room_launch = os.path.join(
        get_package_share_directory('room_global_localizer'),
        'launch', 'room_global_localizer.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(autoware_launch),
            launch_arguments={
                'map_path': '/home/tang/autoware_map/map_task4/indoor_localization/map_task5_leveled',
                'vehicle_model': 'yunle_vehicle',
                'sensor_model': 'livox_sensor_kit',
                'pose_source': 'ndt',
                'twist_source': 'gyro_odom',
                'gnss_enabled': 'false',
                'automatic_pose_initializer_enabled': 'false',
                'use_sim_time': 'false',
                'system_run_mode': 'online',
                # Use the room-localization RViz layout instead of starting a
                # second Autoware RViz process.
                'rviz': 'false',
            }.items()),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(room_launch),
            launch_arguments={
                # Autoware livox_sensor_kit already owns the driver and TF.
                'launch_driver': 'false',
                'launch_sensor_tf': 'false',
                'launch_rviz': 'true',
                'use_autoware_ndt': 'true',
                'pointcloud_topic': '/sensing/lidar/concatenated/pointcloud',
            }.items()),
    ])
