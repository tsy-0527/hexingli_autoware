# Autoware 启动条件、定位、避障与控制配置说明

本文档整理当前工程中 Autoware 无人驾驶运行相关的主要条件、配置文件路径、避障功能启用方式，以及避障不生效时的排查方向。当前工程根目录为：

```text
/home/tang/autoware
```

## 1. 主启动入口

实车测试常用启动命令：

```bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_6 \
  vehicle_model:=hunter_vehicle \
  sensor_model:=livox_sensor_kit
```

主启动文件：

```text
src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
```  

该文件会根据参数启动 Map、Sensing、Localization、Perception、Planning、Control、Vehicle、System 等组件。

主要组件入口：

```text
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_control_component.launch.xml
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch.xml
```

## 2. 无人驾驶 Auto 模式启动条件

Auto 按键是否可用，主要由诊断图决定。

核心配置文件：

```text
src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/autoware-main.yaml
```

其中 `/autoware/modes/autonomous` 需要以下模块都满足条件：

```text
/autoware/map
/autoware/localization
/autoware/planning
/autoware/perception
/autoware/control
/autoware/vehicle
/autoware/system
/adapi/mrm_request/delegate
```

也就是说，Auto 常亮并不是单个节点决定的，而是地图、定位、规划、感知、控制、底盘、系统状态全部 OK 后才会稳定可用。

各模块诊断配置文件：

```text
src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/map.yaml
src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/localization.yaml
src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/planning.yaml
src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/perception.yaml
src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/control.yaml
src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/vehicle.yaml
src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/system.yaml
```

常用检查命令：

```bash
ros2 topic echo /diagnostics_graph/status --once
ros2 topic echo /api/system/diagnostics/status --once
```

查看 Auto 模式不可用原因：

```bash
ros2 topic echo /diagnostics_graph/status --once | grep -A80 "/autoware/modes/autonomous"
```

## 3. 定位功能

### 3.1 定位模块入口

定位组件入口：

```text
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml
```

底层定位 launch：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch/launch/localization.launch.xml
src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml
src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch/launch/pose_twist_estimator/ndt_scan_matcher.launch.xml
src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch/launch/pose_twist_estimator/gyro_odometer.launch.xml
src/launcher/autoware_launch/tier4_universe_launch/tier4_localization_launch/launch/pose_twist_fusion_filter/pose_twist_fusion_filter.launch.xml
```

### 3.2 定位主要数据流

点云定位：

```text
/sensing/lidar/concatenated/pointcloud
-> NDT scan matcher
-> /localization/pose_estimator/pose_with_covariance
```

速度与 IMU：

```text
/vehicle/status/velocity_status
-> /sensing/vehicle_velocity_converter/twist_with_covariance
-> gyro_odometer
-> /localization/twist_estimator/twist_with_covariance
```

EKF 融合：

```text
/localization/pose_estimator/pose_with_covariance
/localization/twist_estimator/twist_with_covariance
-> EKF localizer
-> /localization/pose_twist_fusion_filter/pose
-> /localization/kinematic_state
-> map -> base_link
```

### 3.3 定位配置文件

EKF 参数：

```text
src/launcher/autoware_launch/autoware_launch/config/localization/ekf_localizer.param.yaml
```

当前已为实车低速调试调整：

```yaml
predict_frequency: 20.0
tf_rate: 20.0
```

NDT 参数：

```text
src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml
```

NDT 点云预处理：

```text
src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range.param.yaml
src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter.param.yaml
src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/pointcloud_preprocessor/random_downsample_filter.param.yaml
```

定位误差监控：

```text
src/launcher/autoware_launch/autoware_launch/config/localization/localization_error_monitor.param.yaml
```

初始化相关：

```text
src/launcher/autoware_launch/autoware_launch/config/localization/pose_initializer.param.yaml
src/launcher/autoware_launch/autoware_launch/config/localization/stop_filter.param.yaml
src/launcher/autoware_launch/autoware_launch/config/localization/pose_instability_detector.param.yaml
src/launcher/autoware_launch/autoware_launch/config/localization/twist2accel.param.yaml
```

### 3.4 定位常用检查命令

```bash
ros2 topic hz /localization/pose_estimator/pose_with_covariance
ros2 topic hz /localization/twist_estimator/twist_with_covariance
ros2 topic hz /localization/pose_twist_fusion_filter/pose
ros2 topic hz /localization/kinematic_state
ros2 run tf2_ros tf2_echo map base_link
```

查看 EKF 状态：

```bash
ros2 topic echo /diagnostics | grep -A70 "name: 'localization: ekf_localizer'"
```

查看 NDT 状态：

```bash
ros2 topic echo /diagnostics | grep -A80 "ndt_scan_matcher: scan_matching_status"
```

## 4. 感知与障碍物输入

避障能否生效，首先取决于障碍物是否被感知模块正确输出。

规划模块主要依赖两个障碍物输入：

```text
/perception/object_recognition/objects
/perception/obstacle_segmentation/pointcloud
```

其中：

```text
/perception/object_recognition/objects
```

用于动态/静态目标识别、跟踪、预测等。

```text
/perception/obstacle_segmentation/pointcloud
```

用于无地面点云、障碍物点云、周围障碍物检测、AEB、碰撞检测等。

感知组件入口：

```text
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml
```

重要参数：

```text
use_empty_dynamic_object_publisher
use_pointcloud_map
use_irregular_object_detector
use_obstacle_segmentation_single_frame_filter
use_obstacle_segmentation_time_series_filter
occupancy_grid_map_method
detected_objects_validation_method
lidar_detection_model
```

障碍物分割配置：

```text
src/launcher/autoware_launch/autoware_launch/config/perception/obstacle_segmentation/ground_segmentation/ground_segmentation.param.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/obstacle_segmentation/ground_segmentation/elevation_map_parameters.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/obstacle_segmentation/occupancy_grid_based_outlier_filter/occupancy_grid_map_outlier_filter.param.yaml
```

占据栅格配置：

```text
src/launcher/autoware_launch/autoware_launch/config/perception/occupancy_grid_map/pointcloud_based_occupancy_grid_map.param.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/occupancy_grid_map/multi_lidar_pointcloud_based_occupancy_grid_map.param.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/occupancy_grid_map/laserscan_based_occupancy_grid_map.param.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/occupancy_grid_map/binary_bayes_filter_updater.param.yaml
```

点云聚类与目标识别配置：

```text
src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/clustering/euclidean_cluster.param.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/lidar_model/centerpoint.param.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/tracking/multi_object_tracker/multi_object_tracker_node.param.yaml
src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/prediction/map_based_prediction.param.yaml
```

常用检查命令：

```bash
ros2 topic hz /perception/obstacle_segmentation/pointcloud
ros2 topic echo --once /perception/obstacle_segmentation/pointcloud --field header
ros2 topic hz /perception/object_recognition/objects
ros2 topic echo --once /perception/object_recognition/objects
ros2 topic hz /perception/occupancy_grid_map/map
```

## 5. 避障功能

Autoware 中的“避障”不是一个单独节点完成的，而是由多个模块共同完成。大致分为三类：

```text
1. 路径级避障：改变行驶路径，绕开障碍物。
2. 速度级避障：不改变路径，但减速、停车、限速。
3. 控制安全检查：AEB、碰撞检测、轨迹跟踪安全检查。
```

### 5.1 路径级避障

路径级避障在 Behavior Path Planner 中启用。

launch 文件：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
```

主要开关：

```xml
<arg name="launch_static_obstacle_avoidance" default="true"/>
<arg name="launch_dynamic_obstacle_avoidance" default="true"/>
<arg name="launch_avoidance_by_lane_change_module" default="true"/>
```

对应模块：

```text
StaticObstacleAvoidanceModuleManager
DynamicObstacleAvoidanceModuleManager
AvoidanceByLaneChangeModuleManager
```

配置文件：

```text
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml
```

Behavior Path Planner 主配置：

```text
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/behavior_path_planner.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/scene_module_manager.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml
```

路径级避障输入：

```text
/planning/mission_planning/route
/map/vector_map
/perception/object_recognition/objects
/perception/occupancy_grid_map/map
/localization/kinematic_state
```

输出：

```text
/planning/scenario_planning/lane_driving/behavior_planning/path
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
```

### 5.2 速度级避障

速度级避障在 Motion Velocity Planner 中启用。它通常不改变路径，而是在轨迹上插入减速、停车、限速。

launch 文件：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml
```

主要开关：

```xml
<arg name="launch_obstacle_stop_module" default="true"/>
<arg name="launch_obstacle_slow_down_module" default="true"/>
<arg name="launch_obstacle_cruise_module" default="true"/>
<arg name="launch_dynamic_obstacle_stop_module" default="true"/>
<arg name="launch_obstacle_velocity_limiter_module" default="true"/>
<arg name="launch_run_out_module" default="true"/>
<arg name="launch_boundary_departure_prevention_module" default="true"/>
<arg name="launch_road_user_stop_module" default="true"/>
```

对应配置文件：

```text
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/motion_velocity_planner.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/dynamic_obstacle_stop.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_velocity_limiter.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/out_of_lane.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/run_out.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/boundary_departure_prevention.param.yaml
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/road_user_stop.param.yaml
```

速度级避障输入：

```text
/planning/scenario_planning/lane_driving/behavior_planning/path
/map/vector_map
/localization/kinematic_state
/localization/acceleration
/perception/object_recognition/objects
/perception/obstacle_segmentation/pointcloud
/perception/occupancy_grid_map/map
```

输出：

```text
/planning/scenario_planning/lane_driving/trajectory
/planning/scenario_planning/status/stop_reasons
/planning/velocity_factors/motion_velocity_planner
```

### 5.3 周围障碍物检查

Surround Obstacle Checker 用于检查车辆周围是否存在障碍物，常用于起步或低速状态下防止车辆直接启动撞到近距离障碍物。

launch 文件：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml
```

配置文件：

```text
src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker/surround_obstacle_checker.param.yaml
```

源码包默认配置参考：

```text
src/universe/autoware_universe/planning/autoware_surround_obstacle_checker/config/surround_obstacle_checker.param.yaml
```

输入：

```text
/localization/kinematic_state
/perception/object_recognition/objects
/perception/obstacle_segmentation/pointcloud
```

输出：

```text
/planning/scenario_planning/max_velocity_candidates
/planning/scenario_planning/status/no_start_reason
/planning/scenario_planning/status/stop_reasons
```

### 5.4 控制安全避障

控制侧还有 AEB 和碰撞检测等安全检查，不属于规划绕行，但可以在危险时停车或触发安全状态。

控制组件入口：

```text
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_control_component.launch.xml
```

底层控制 launch：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml
```

AEB 默认启用：

```xml
<arg name="launch_autonomous_emergency_braking" default="true"/>
```

碰撞检测默认启用：

```xml
<arg name="launch_collision_detector" default="true"/>
```

Obstacle Collision Checker 默认未启用：

```xml
<arg name="launch_obstacle_collision_checker" default="false"/>
```

控制安全相关配置文件：

```text
src/launcher/autoware_launch/autoware_launch/config/control/autoware_autonomous_emergency_braking/autonomous_emergency_braking.param.yaml
src/launcher/autoware_launch/autoware_launch/config/control/autoware_collision_detector/collision_detector.param.yaml
src/launcher/autoware_launch/autoware_launch/config/control/obstacle_collision_checker/obstacle_collision_checker.param.yaml
src/launcher/autoware_launch/autoware_launch/config/control/predicted_path_checker/predicted_path_checker.param.yaml
src/launcher/autoware_launch/autoware_launch/config/control/lane_departure_checker/lane_departure_checker.param.yaml
src/launcher/autoware_launch/autoware_launch/config/control/control_validator/control_validator.param.yaml
```

## 6. 控制算法

当前控制组件默认配置：

```text
lateral_controller_mode: mpc
longitudinal_controller_mode: pid
```

配置入口：

```text
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_control_component.launch.xml
```

控制算法配置文件：

```text
src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/trajectory_follower_node.param.yaml
src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml
src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral/pure_pursuit.param.yaml
src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml
```

控制输入：

```text
/planning/trajectory
/localization/kinematic_state
/vehicle/status/steering_status
/localization/acceleration
```

控制输出：

```text
/control/trajectory_follower/control_cmd
/control/command/control_cmd
/control/command/gear_cmd
/control/command/turn_indicators_cmd
/control/command/hazard_lights_cmd
```

Hunter 底盘对接链路：

```text
/control/command/control_cmd
-> hunter_autoware_bridge
-> /cmd_vel
-> hunter_base
-> Hunter 底盘
```

## 7. 如何启用避障功能

### 7.1 默认状态

当前工程中，大部分规划避障模块默认已经启用。

路径级避障默认启用：

```xml
launch_static_obstacle_avoidance="true"
launch_dynamic_obstacle_avoidance="true"
launch_avoidance_by_lane_change_module="true"
```

速度级避障默认启用：

```xml
launch_obstacle_stop_module="true"
launch_obstacle_slow_down_module="true"
launch_obstacle_cruise_module="true"
launch_dynamic_obstacle_stop_module="true"
launch_obstacle_velocity_limiter_module="true"
```

控制安全模块中：

```xml
launch_autonomous_emergency_braking="true"
launch_collision_detector="true"
launch_obstacle_collision_checker="false"
```

因此，普通避障/减速/停车功能理论上随 `autoware.launch.xml` 启动后已经加载。

### 7.2 启用或关闭某个避障模块

如果要改默认启用状态，主要改这些 launch 文件中的默认参数：

路径级避障：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
```

速度级避障：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml
```

控制安全检查：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml
```

如果只是调参数，不建议直接关掉安全模块；更建议调整对应 `.param.yaml` 中的检测范围、速度阈值、停止距离、目标类型过滤等参数。

### 7.3 启用 Obstacle Collision Checker

该模块默认是：

```xml
launch_obstacle_collision_checker="false"
```

如果需要启用，需要在：

```text
src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml
```

中改为：

```xml
<arg name="launch_obstacle_collision_checker" default="true"/>
```

对应参数文件：

```text
src/launcher/autoware_launch/autoware_launch/config/control/obstacle_collision_checker/obstacle_collision_checker.param.yaml
```

## 8. 避障不生效的常见原因

### 8.1 感知没有输出障碍物

检查：

```bash
ros2 topic hz /perception/obstacle_segmentation/pointcloud
ros2 topic hz /perception/object_recognition/objects
ros2 topic echo --once /perception/object_recognition/objects
```

如果没有障碍物输出，规划模块无法避障。

可能原因：

```text
1. LiDAR 点云没有进入 /sensing/lidar/concatenated/pointcloud。
2. 障碍物分割 crop box 过滤过严。
3. 点云 frame_id 与 TF 不匹配。
4. 感知模块未启动或模型文件缺失。
5. 目标被 map filter、lanelet filter 或 object filter 过滤掉。
```

### 8.2 障碍物点云被过滤为空

典型诊断：

```text
crop_box_filter: No valid output points
Output point count: 0
```

检查：

```bash
ros2 topic echo --once /perception/obstacle_segmentation/pointcloud --field header
ros2 topic hz /perception/obstacle_segmentation/pointcloud
```

需要重点检查：

```text
1. LiDAR 外参是否正确。
2. base_link -> sensor frame 的 TF 是否正确。
3. crop_box_filter 的 min_z/max_z/输入 frame 是否合理。
4. 点云是否被错误地转换到了 base_link 下方或上方。
```

### 8.3 TF 不通或时间戳不一致

避障模块需要把点云、目标、车辆位姿转换到统一坐标系。

检查：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link livox_front/livox_front_base_link
ros2 run tf2_ros tf2_monitor map base_link
```

常见问题：

```text
1. map -> base_link 不连续。
2. base_link 同时被底盘和 Autoware 定位发布，造成 TF 冲突。
3. 点云时间戳早于 TF buffer 可用时间，出现 extrapolation into the past。
4. 传感器 frame_id 与 sensor_kit_calibration.yaml 中不一致。
```

### 8.4 规划没有有效路线

路径级避障依赖有效 route 和 lanelet 地图。

检查：

```bash
ros2 topic echo --once /planning/mission_planning/route
ros2 topic hz /planning/trajectory
ros2 topic echo --once /planning/scenario_planning/status/stop_reasons
```

可能原因：

```text
1. 没有设置 goal。
2. lanelet2_map.osm 与点云地图/车辆定位不对齐。
3. 车辆不在 lanelet 可行驶区域内。
4. 路线规划失败或 route 未发布。
```

### 8.5 避障模块启动了，但目标不满足触发条件

例如静态避障通常会判断：

```text
目标类型
目标速度
目标是否在自车路径附近
目标是否在可避让范围内
道路宽度是否足够
是否允许跨线或借道
RTC/approval 状态
```

对应参数重点看：

```text
static_obstacle_avoidance.param.yaml
dynamic_obstacle_avoidance.param.yaml
obstacle_stop.param.yaml
obstacle_slow_down.param.yaml
dynamic_obstacle_stop.param.yaml
```

如果障碍物在 RViz 中能看到，但车辆只是停车不绕行，可能是：

```text
1. 车道太窄，不满足绕行横向空间。
2. 障碍物太近，无法生成平滑避障路径。
3. 避障模块要求 approval，但未自动批准。
4. 目标类型没有被设置为 avoidance target。
5. 速度级停车模块优先触发，路径级绕行还没生成。
```

### 8.6 控制链路未正常输出到底盘

如果规划轨迹已经变化，但车辆没有执行，需要检查控制链路：

```bash
ros2 topic hz /planning/trajectory
ros2 topic hz /control/trajectory_follower/control_cmd
ros2 topic hz /control/command/control_cmd
ros2 topic hz /cmd_vel
ros2 topic hz /vehicle/status/velocity_status
```

链路应为：

```text
/planning/trajectory
-> /control/trajectory_follower/control_cmd
-> /control/command/control_cmd
-> hunter_autoware_bridge
-> /cmd_vel
-> hunter_base
```

可能问题：

```text
1. Auto 没有 engage。
2. vehicle_cmd_gate 没有放行自动控制命令。
3. control_mode 不是 autonomous。
4. hunter_autoware_bridge 未运行。
5. hunter_base 没有订阅 /cmd_vel。
6. 底盘 CAN 或驱动未连接。
```

## 9. 建议的实车避障验证流程

1. 启动 Autoware。

```bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_6 \
  vehicle_model:=hunter_vehicle \
  sensor_model:=livox_sensor_kit
```

2. 确认定位稳定。

```bash
ros2 topic hz /localization/kinematic_state
ros2 run tf2_ros tf2_echo map base_link
```

3. 确认感知输出。

```bash
ros2 topic hz /perception/obstacle_segmentation/pointcloud
ros2 topic hz /perception/object_recognition/objects
```

4. 在 RViz 中确认车辆、点云地图、车道线、实时点云、障碍物处于同一区域。

5. 设置路线和目标点。

6. 检查规划轨迹。

```bash
ros2 topic hz /planning/trajectory
ros2 topic echo --once /planning/scenario_planning/status/stop_reasons
ros2 topic echo --once /planning/velocity_factors/motion_velocity_planner
```

7. 低速 engage，先验证停车避障，再验证绕行避障。

8. 若没有避障效果，按以下顺序排查：

```text
感知输出 -> TF/时间戳 -> route/lanelet -> 避障模块参数 -> planning trajectory -> control command -> chassis
```

## 10. 总结

当前工程中，避障功能主要由 Planning 和 Control 两侧共同完成：

```text
Behavior Path Planner:
负责路径级绕行，例如静态障碍物避让、动态障碍物避让、借道避让。

Motion Velocity Planner:
负责速度级避障，例如障碍物停车、减速、跟车、动态障碍物停车。

Control Safety Modules:
负责最后的安全检查，例如 AEB、碰撞检测、车道偏离检查。
```

对于当前 Hunter 小车平台，建议先保证：

```text
1. 定位稳定。
2. /perception/obstacle_segmentation/pointcloud 稳定有数据。
3. /perception/object_recognition/objects 能输出有效障碍物。
4. /planning/trajectory 会因障碍物发生停车、减速或绕行变化。
5. /control/command/control_cmd 能被 hunter_autoware_bridge 转换并发送到底盘。
```

只有这几条链路全部稳定后，避障功能才可能在实车上可靠生效。
