# indoor_localization：室内实时定位

MID-360 + NDT + 轮速/IMU + EKF。Autoware 自带的定位节点负责算法，本包负责把它们按室内需求接起来，并补上 5 个自定义节点。

## 本机 task5 推荐入口（初始化 + 实时定位）

本机不要直接运行下文的通用 `indoor_localization.launch.py`，因为完整 Autoware 已经负责
地图、MID-360S、IMU、轮速和未来的规划控制。推荐入口只启动一套定位节点：

```bash
cd /home/tang/tsy/indoor_localization
./run_task5_combined_localization.sh
```

启动后保持车辆静止，触发房间全局初始化：

```bash
ros2 service call /room_global_localizer/trigger std_srvs/srv/Trigger '{}'
```

数据链为：

```text
MID-360S / IMU / 轮速
  -> 房间 Top-1 粗定位
  -> Autoware NDT 6DoF 初始化
  -> 室内参数 NDT 实时匹配
  -> 50 Hz EKF
  -> /localization/kinematic_state + map->base_link
```

健康状态：`/indoor_localization/health`。规划控制以后可直接使用标准的
`/localization/kinematic_state` 和 TF，不需要改定位接口。需要连同规划控制启动时：

```bash
./run_task5_combined_localization.sh \
  launch_perception:=true launch_planning:=true launch_control:=true
```

当前默认不启用 `degeneracy_covariance_relay`：它的走廊区域必须先用 task5 实车轨迹核对，
否则可能在错误区域放大 EKF 协方差。

当前 task5 静止实测中，正确主候选 NVTL 为 1.467--1.473，对称错误候选为
1.22--1.25，因此 NDT 初始化阈值暂设为 1.35，健康监控 WARN/ERROR 暂设为
1.40/1.30。车辆行驶采集更多数据后必须重新统计阈值。

## 一、整体结构

```
MID-360 驱动 ─► mid360_preprocessor ─► ndt_scan_matcher ─► degeneracy_covariance_relay ─► ekf_localizer ─► stop_filter
 /sensing/lidar/   转 base_link、裁剪、      点云 vs 地图          走廊段放大纵向协方差          融合              │
 mid360s/pointcloud 去畸变、0.1m 降采样             ▲                                           ▲                 ▼
                       ▲ 去畸变用速度               │ EKF 位姿作为匹配初值 ◄─────────────────────┤      /localization/kinematic_state
                       │                            │                                           │      /localization/pose_with_covariance
底盘 VelocityReport ─► velocity_report_converter ─► gyro_odometer ◄─ GINS2000 IMU ──── 速度 ──────┘      TF map → base_link
                         （轮速比例系数在这里标定）

/room_global_localizer/pose ─► initial_pose_bridge ─► pose_initializer ─► NDT 精配准 ─► EKF 重置
                                （只在未初始化时转发一次）

localization_health_monitor ─► /indoor_localization/health（OK/WARN/ERROR + 建议速度系数，给 UI / Agent）
```

## 二、本包的 5 个节点

| 节点 | 作用 |
|---|---|
| `mid360_preprocessor` | 读 livox 点云（自动识别每点时间戳格式）→ 转到 base_link → 去掉车身、地面（z<0.15m，地图里几乎没有地面点）和远处点 → 按 gyro_odometer 的速度做平面去畸变 → 0.1m 体素降采样 → 时间戳设为扫描结束时刻 |
| `degeneracy_covariance_relay` | 插在 NDT 和 EKF 之间。车在走廊退化段（y≈−34 ~ −16）时，把 NDT 位姿沿走廊方向的标准差放大到 1m，横向和朝向不变，两端各 2m 平滑过渡 |
| `velocity_report_converter` | 底盘 VelocityReport → 带协方差的速度，**轮速比例系数 `speed_scale_factor` 在这里标定** |
| `initial_pose_bridge` | 收到 `/room_global_localizer/pose` 后调用 `/localization/initialize`（AUTO 模式，NDT 精配准后重置 EKF）。已经初始化时忽略后续位姿，避免 EKF 被反复重置；需要重新定位时先调 `~/arm` |
| `localization_health_monitor` | 汇总初始化状态、NDT 是否按时输出、NDT 得分、EKF 横向/纵向标准差，发布 JSON |

## 三、启动

```bash
colcon build --packages-select indoor_localization
ros2 launch indoor_localization indoor_localization.launch.py \
    pointcloud_map_path:=/path/to/indoor_map_leveled_clean.pcd \
    lidar_topic:=/sensing/lidar/mid360s/pointcloud \
    imu_topic:=<GINS2000 的 sensor_msgs/Imu 话题> \
    velocity_report_topic:=/vehicle/status/velocity_status
```

**本 launch 不启动、需要你们自己先启动的：**
- MID-360 驱动、GINS2000 驱动、底盘接口（chassis_driver）
- 静态 TF：`base_link → 雷达`、`base_link → IMU`（外参里要包含雷达约 3° 的安装俯仰）
- 房间全局定位节点（发布 `/room_global_localizer/pose`）
- **不要启动室外的卫星定位节点**，它和 NDT 会同时往 EKF 发位姿

**launch 参数：**

| 参数 | 默认值 | 说明 |
|---|---|---|
| `pointcloud_map_path` | 必填 | 调平后的室内 PCD |
| `lidar_topic` | `/sensing/lidar/mid360s/pointcloud` | |
| `imu_topic` | `/sensing/imu/imu_data` | GINS2000 IMU，frame_id 必须能通过 TF 转到 base_link |
| `velocity_report_topic` | `/vehicle/status/velocity_status` | |
| `use_velocity_converter` | `true` | 如果已经在跑 Autoware 的 vehicle_velocity_converter，设为 false |
| `autoware_pkg_prefix` | `autoware_` | 2024 年以后的 Autoware 版本用 `autoware_`；更早的版本包名没有前缀，填空字符串 |
| `ndt_param_path` / `ekf_param_path` | 本包 config | 见第五节 |

## 四、主要话题

| 话题 | 说明 |
|---|---|
| `/localization/util/downsample/pointcloud` | 预处理后的点云（base_link） |
| `/localization/pose_estimator/ndt_pose_with_covariance_raw` | NDT 原始输出 |
| `/localization/pose_estimator/pose_with_covariance` | 经过走廊协方差处理后，送给 EKF |
| `/localization/pose_estimator/nearest_voxel_transformation_likelihood` | NDT 匹配得分 NVTL |
| `/localization/twist_estimator/twist_with_covariance` | 轮速 + IMU 速度 |
| `/localization/kinematic_state` | **最终定位结果**（和室外相同，UI、遥测不用改） |
| `/localization/pose_with_covariance` | EKF 位姿 + 协方差 |
| `/localization/initialization_state` | 初始化状态 |
| `/indoor_localization/health` | 定位健康状态 JSON |

## 五、参数文件

`config/ndt_scan_matcher.indoor.param.yaml` 和 `config/ekf_localizer.indoor.param.yaml` 是**从 autoware_core 最新版复制后修改**的，改动处都标了 `# INDOOR`：

| 文件 | 参数 | 默认 → 室内 | 原因 |
|---|---|---|---|
| NDT | `sensor_points.required_distance` | 10.0 → **4.0** | 房间只有 9×5.6m，默认值会导致 NDT 在房间里直接不匹配 |
| NDT | `ndt.resolution` | 2.0 → 1.0 | 窄走廊和小房间需要更细的体素 |
| NDT | `validation.initial_to_result_distance_tolerance_m` | 3.0 → 1.5 | 室内低速 |
| NDT | `dynamic_map_loading.*` | 缩小 | 整栋楼的地图很小 |
| EKF | `diagnostics.error/warn_ellipse_size` | 1.5/1.2 → 3.0/2.4 | 走廊段纵向不确定度会增大，避免误报 |

**如果你们的 Autoware 版本较旧**，参数结构可能不同，节点会因为缺参数或多参数启动失败。这时请拿你们版本自带的 param.yaml，按上表手动改，再通过 `ndt_param_path` / `ekf_param_path` 传入。

## 六、调试顺序（每步通过再进入下一步）

1. **TF 和话题**：`ros2 topic hz` 确认点云约 10Hz、IMU、VelocityReport 都在；`ros2 run tf2_ros tf2_echo base_link <雷达frame>` 确认外参。
2. **速度链路**：看 `/localization/twist_estimator/twist_with_covariance`。车静止时速度为 0；原地转向时角速度方向正确（左转为正）。
3. **点云预处理**：RViz 里看 `/localization/util/downsample/pointcloud`（frame base_link）。车身点被去掉；转弯时墙面清晰不发虚。日志里每 5 秒打印一次频率和处理耗时。
4. **初始化**：车停在房间里，启动房间全局定位。看 `initial_pose_bridge` 的状态变为 DONE，`/localization/initialization_state` 变为 INITIALIZED，RViz 里点云和地图重合。
5. **房间内低速行驶**：NVTL 稳定（记录正常范围），`/indoor_localization/health` 为 OK。
6. **走廊来回行驶**：看 `degeneracy_covariance_relay/debug/zone_weight` 在退化段为 1；横向标准差保持很小；出退化段时位置没有明显跳变。

## 七、必须标定 / 填写的参数（代码里标了 TODO）

| 参数 | 位置 | 方法 |
|---|---|---|
| `self_box` | mid360_preprocessor | 车身在 base_link 下的范围 [xmin,xmax,ymin,ymax,zmin,zmax] |
| `speed_scale_factor` | velocity_report_converter | 在房间或大厅直线行驶 20m 以上，用 NDT 测得的距离除以轮速积分的距离，前进和倒车各测几次，目标误差 < 0.5% |
| `nvtl_warn` / `nvtl_error` | health_monitor | 跑几圈记录房间、走廊、大厅的 NVTL 分布，阈值设在正常值下方 |
| 走廊区域 | relay | 已按地图测好，如果地图更新需要重新确认 |

## 八、测试情况

- `test/test_core.py`：去畸变（与真值对比误差 < 1e-9 m）、点时间格式识别、裁剪、降采样、走廊协方差放大（结果保持正定）全部通过。
- 5 个节点在模拟的 ROS 环境下跑过：livox 格式点云（26 字节/点、绝对 ns 时间戳、雷达俯仰 3°）、走廊内外的协方差变化、初始化只触发一次、健康状态判断。
- launch 中 Autoware 节点的话题、参数名、launch 参数，按 autoware_core 最新源码（2026-11）核对过。
- **还没在真实 ROS / Autoware 环境中运行过**，第一次启动请按第六节逐步检查。
