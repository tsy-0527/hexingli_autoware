# Yunle indoor Nav2 workspace (ROS 2 Humble)

This workspace reuses the existing Yunle chassis driver, vehicle safety bridge,
Livox Mid-360 driver, GINS2000 IMU and their calibrated TF.  Autoware planning
and control nodes are not launched.

## Install runtime dependencies

Run once in an interactive terminal with sudo access:

```bash
sudo apt-get install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-ackermann-msgs
```

## Build

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/yunle_chassis-main/install/setup.bash
source ~/autoware/install/setup.bash

cd ~/autoware/yunle_nav2_ws
colcon build --symlink-install
source install/setup.bash
```

## Create a grid map

```bash
ros2 launch yunle_nav2_bringup mapping.launch.py
```

After driving the vehicle slowly through the area, save both the serialized
SLAM state and the occupancy map.  Example:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/tang/autoware/yunle_nav2_ws/maps/site'}"
ros2 run nav2_map_server map_saver_cli -f \
  /home/tang/autoware/yunle_nav2_ws/maps/site
```

## Localize and navigate

```bash
ros2 launch yunle_nav2_bringup bringup.launch.py \
  map:=/home/tang/autoware/yunle_nav2_ws/maps/site.yaml
```

Before enabling physical AUTO mode, verify `/scan`, `/odometry/wheel`,
`/odometry/filtered`, the complete TF tree, and the local/global costmaps.
Keep the wheels off the ground for the first command-direction and steering-sign
test.

## Imported calibration

- Wheelbase: 0.475 m
- Vehicle footprint relative to rear-axle `base_link`:
  front 0.6635 m, rear -0.1885 m, left/right 0.286 m
- Maximum planning steering angle: 0.418879 rad (24 deg)
- `base_link -> sensor_kit_base_link`: (0.60, 0.01, 0.57) m
- GINS2000 corrected IMU topic: `/sensing/imu/imu_data`
- Chassis feedback topics: `/vehicle/status/velocity_status` and
  `/vehicle/status/steering_status`

The physical steering limit still needs a measured calibration.  The current
repositories contain 21, 24 and 27 degree limits in different paths, so 24
degrees is retained only to match the active Autoware vehicle bridge.
