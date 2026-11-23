# MID-360S 独立室内初始化定位

本包不启动或依赖 Autoware。它直接启动 Livox MID-360S 驱动，累积标准
`PointCloud2`，完成房间约束的全局匹配后发布初始位姿。

此外提供推荐的 Autoware NDT 组合模式：房间搜索负责全局粗定位，Autoware
NDT 使用 task5 三维点云做 6 自由度精配准并持续跟踪。

## 推荐：Autoware NDT 组合模式

初始化和实时定位统一运行时，优先使用新入口：

```bash
cd /home/tang/autoware_map/map_task4/indoor_localization
./run_task5_combined_localization.sh
```

该入口使用 `indoor_localization` 包的室内 NDT/EKF 参数，并保持标准输出
`/localization/kinematic_state`，可直接接后续 Autoware 规划控制。

```bash
cd /home/tang/autoware_map/map_task4/indoor_localization
./run_autoware_ndt_localization.sh
```

该脚本等价于以以下关键参数启动 Autoware：

```text
map_path=/home/tang/autoware_map/map_task5
vehicle_model=yunle_vehicle
sensor_model=livox_sensor_kit
pose_source=ndt
twist_source=gyro_odom
gnss_enabled=false
automatic_pose_initializer_enabled=false
system_run_mode=online
```

组合模式下 Autoware 负责 MID-360S 驱动和传感器 TF，本包订阅
`/sensing/lidar/concatenated/pointcloud`。房间搜索先保留 Top-3 粗定位候选，逐个调用
`/localization/pose_estimator/ndt_align_srv` 做 6 自由度 NDT 精配准。2D 最高分候选只要通过
NDT 的 `reliable` 检查，才采用它的 6DoF 精配准结果；Top-2/Top-3 仅用于诊断对称性，
不能“投票”或回退覆盖主候选。主候选精配准成功后才把位姿送到
`/api/localization/initialize`；然后等待
`/api/localization/initialization_state` 变为 `INITIALIZED`，并把
`/localization/kinematic_state` 作为持续定位结果显示和输出。

车辆静止、Autoware 完全启动后触发：

```bash
source /opt/ros/humble/setup.bash
ros2 service call /room_global_localizer/trigger std_srvs/srv/Trigger '{}'
```

组合模式不再因为旧的单候选 `min_score=0.70` 或 `ratio` 门槛而在 NDT 前提前拒绝。
`score >= ndt_min_seed_score`（当前 0.60）只允许候选进入 NDT 验证，并不代表接受；
主候选 NDT 不可靠或超时时会重新采集；第二次仍失败则拒绝，而且不会用低排名候选发布
正式初始位姿。

## 已配置的硬件和地图

- 雷达：Livox MID-360S，`192.168.1.149`
- 主机雷达网卡：`eno2`，`192.168.1.50/24`
- 雷达点云：`/sensing/lidar/mid360s/pointcloud`
- 雷达坐标系：`livox_front/livox_front_base_link`
- 当前外参：`base_link -> 雷达 = (0.60, 0.01, 0.57)` m，RPY 为 0
- 运行地图：`map_task5_leveled/pointcloud_map.pcd`（调平、清噪后的内部坐标）
- 地图源文件：`map/indoor_map_leveled_clean.pcd`

粗匹配、NDT、EKF 和 Nav2 都在调平后的内部坐标系运行，发布位姿不再转换回
原始 task5 坐标。`T_old_to_new.txt` 只用于转换仍保存在旧坐标中的离线数据。

## 独立模式（不使用 Autoware）

```bash
cd /home/tang/autoware_map/map_task4/indoor_localization
source /opt/ros/humble/setup.bash
source /home/tang/livox_ws/install/setup.bash
colcon build --packages-select room_global_localizer --symlink-install
./run_localizer.sh
```

`run_localizer.sh` 默认同时打开 RViz。界面中的显示含义：

- 灰色地图：调平后的内部点云地图
- 红色 `Aligned Scan Result`：本次最佳候选位姿对齐后的 MID-360S 点云
- 黄色 `Best Candidate Pose`：本次搜索的最佳候选位姿和协方差
- `Score and Decision`：独立模式显示绿色 `ACCEPTED` 或红色 `REJECTED`；组合模式进入
  精配准时显示黄色 `NDT VALIDATING`，最终状态以 `/room_global_localizer/status` 为准

若不需要 GUI：

```bash
ros2 launch room_global_localizer room_global_localizer.launch.py launch_rviz:=false
```

车辆保持静止，在另一个终端触发：

```bash
source /opt/ros/humble/setup.bash
ros2 service call /room_global_localizer/trigger std_srvs/srv/Trigger '{}'
```

结果：

- 位姿：`/room_global_localizer/pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- 状态：`/room_global_localizer/status` (`std_msgs/msg/String`，JSON)
- 最佳候选：`/room_global_localizer/candidate_pose`
- 候选对齐点云：`/room_global_localizer/aligned_cloud`
- RViz 结果文字：`/room_global_localizer/result_marker`

```bash
ros2 topic echo /room_global_localizer/pose --once
ros2 topic echo /room_global_localizer/status --once
```

如果外部程序已经启动 MID-360S 驱动，可避免重复启动：

```bash
ros2 launch room_global_localizer room_global_localizer.launch.py launch_driver:=false
```

## 上车前确认

- `auto_start` 默认关闭，只有车辆静止时才调用 trigger。
- 当前外参 pitch 为 0；文档中的约 3° 只是待验证估计值，不能直接写入。
- `min_score: 0.70`、`max_ratio: 0.85` 是仿真初值，需实车采集至少 20 个位姿标定。
- 输出是一次性的初始化位姿，不持续发布 `map -> base_link` TF；后续跟踪定位模块应订阅该位姿进行初始化。
