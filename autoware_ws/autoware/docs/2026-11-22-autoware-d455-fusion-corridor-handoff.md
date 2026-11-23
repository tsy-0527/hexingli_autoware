# 2026-11-22 Autoware D455 相机雷达融合与走廊避障交接文档

> 完成时间：2026-11-22 20:55（Asia/Shanghai）  
> 适用工作区：`/home/tang/autoware`  
> 适用车辆配置：`yunle_vehicle + livox_sensor_kit`  
> 文档目的：让没有参与今天调试的人或新的 AI Agent 能理解系统、复核现状、继续测试和排障。

## 1. 先看结论

今天完成的工作可以分成五条主线：

1. 把原本的纯 LiDAR 感知切换为 Autoware 原生 `camera_lidar_fusion`，并接入前视 Intel RealSense D455。
2. 保留 Autoware 内置 7 类 YOLOX，同时接入自行训练的三类 YOLO26s，在 ROI 层合并后交给 Autoware 原生 C++ 融合节点。
3. 修复相机雷达时间同步、ROI 与 LiDAR 聚类匹配、跟踪器首路无发布者等断链问题，使检测结果能进入 tracking 和 prediction。
4. 针对走廊中的地面、墙面、石墩误检和漏检，调整地面分割后的聚类输入、聚类参数、低矮小障碍物支路和 RViz 可视化。
5. 针对 2.3 m 宽走廊的行人绕行反复调参，并从日志确认一次紧急停车不是 AEB 或碰撞检测触发，而是行为规划轨迹停止更新约 5 秒后，`/autoware/planning/topic_rate_check/trajectory` 超时，MRM 执行 `EMERGENCY_STOP`。

当前最重要的判断是：

- 感知链路已经实车打通多次，但最终这组走廊规划参数是在当天最后一次修改后静态校验的，**没有完成同等强度的最终实车复测**。
- 2.3 m 走廊里的长墙仍可能形成约 10 m 级大簇和凸包，当前参数只能缓解，不能从几何上保证墙面轮廓不会侵入可行驶区域。
- D455 当前用于融合的是 RGB、`CameraInfo` 和 TF，深度流在最终配置中关闭。早上实现过的 RGB-D 低矮障碍物补盲和 D455 深度点合并支路，在当前正式源码中已不存在，不能按旧对话中的路径继续操作。
- 当前工作区非常脏，且大量文件只是权限位变化。不要执行 `git reset --hard`、`git clean` 或大范围 `git checkout`。

## 2. 当前系统组成

### 2.1 硬件和基础环境

| 项目 | 当前值 |
|---|---|
| 车辆模型 | `yunle_vehicle` |
| 车辆实际参数文件 | `/home/tang/autoware/src/launcher/autoware_launch/vehicle/yunle_vehicle_launch/yunle_vehicle_description/config/vehicle_info.param.yaml` |
| 车身尺寸 | 长 0.852 m、宽 0.572 m、高 0.327 m |
| 轴距 | 0.475 m |
| 轮距 | 0.463 m |
| 最大前轮转角 | 0.366519 rad，约 21° |
| 雷达 | Livox Mid-360 |
| 雷达 IP | `192.168.1.149` |
| 相机 | Intel RealSense D455 |
| 相机序列号 | `262822304227` |
| 相机固件 | `5.15.1.55` |
| 计算平台 | NVIDIA RTX 3060 Laptop，12 GB |
| GPU 计算能力 | compute 6.1 |
| 地图 | `$HOME/autoware_map/map_task4` |
| 地图大小 | 约 1.3 GB |
| 定位 | NDT，手动初始化 |
| 速度来源 | GINS2000 gyro odometry |

注意：`obstacle_stop.param.yaml` 的文件头注释仍写着 Hunter SE 的 0.55 m/0.52 m 轴距轮距，这是旧注释，与当前实际 launch 的 `yunle_vehicle` 参数不一致。计算和现场判断应以 `vehicle_info.param.yaml` 为准，并尽快修正这段误导性注释。

### 2.2 当前启动命令

必须按以下顺序 source 工作空间，D455 工作空间要放在 Autoware 之后：

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/gins2000_ws/install/setup.bash
source ~/yunle_chassis-main/install/setup.bash
source ~/autoware/install/setup.bash
source ~/tsy/D455_ROS2_Humble/install/setup.bash
```

当前推荐启动命令：

```bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_task4 \
  vehicle_model:=yunle_vehicle \
  sensor_model:=livox_sensor_kit \
  pose_source:=ndt \
  twist_source:=gyro_odom \
  gnss_enabled:=false \
  automatic_pose_initializer_enabled:=false \
  use_sim_time:=false \
  system_run_mode:=online \
  perception_mode:=camera_lidar_fusion \
  launch_camera:=false \
  use_traffic_light_recognition:=false \
  use_custom_d455_yolo:=true \
  custom_d455_yolo_device:=cuda:0 \
  custom_d455_yolo_imgsz:=640 \
  custom_d455_yolo_confidence:=0.4
```

这些参数虽然已经有默认值，现场启动时仍建议显式写出，便于日志和复现。

`launch_camera` 当前默认是 `false`。如果需要由 Autoware 自己启动并占用 D455，启动时显式传入 `launch_camera:=true`；如果需要独立相机/视觉进程持有 D455，则保持 `false`。

启动后不会自动恢复定位位姿，因为：

```text
automatic_pose_initializer_enabled=false
```

如果 RViz 中没有 `map -> base_link`，需要在 RViz 使用 `2D Pose Estimate` 手动初始化。定位未完成时，跟踪器和 lanelet 过滤无法稳定输出 map 坐标系下的目标，不能据此判断融合失效。

### 2.3 修改后什么时候需要编译

只修改本仓库中的 YAML 参数时，通常不需要编译，因为 `install` 中对应的参数文件是源码软链接。停止并重新启动 Autoware 即可加载新值。

修改 D455 工作区中的 `.py.in`、CMake 或 launch 资源后，需要重新生成并安装 D455 包：

```bash
cd ~/tsy/D455_ROS2_Humble
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
colcon build --symlink-install --packages-select d455_perception
source install/setup.bash
```

修改 Autoware 的 XML 或 Python launch 文件通常重启即可，因为 launch 目录也是软链接。若修改了 `CMakeLists.txt`、`package.xml`、C++ 源码或新增文件，则重新编译对应包：

```bash
cd ~/autoware
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src \
  --packages-select autoware_launch tier4_perception_launch
```

工作区根目录存在历史备份目录，直接执行不带 `--base-paths src` 的全工作区扫描时可能报告“重复包”。这不是当前源码错误。

### 2.4 纯 LiDAR 回退命令

相机节点异常或需要做 LiDAR 基线对比时，可在原命令上覆盖：

```bash
perception_mode:=lidar \
launch_camera:=false \
use_custom_d455_yolo:=false
```

不要同时单独启动 `vision_detection_ws/run_camera_stream.sh`、`web_start_camera.sh` 或另一个 RealSense launch。同一个 D455 被重复打开时会出现：

```text
xioctl(VIDIOC_S_FMT) failed, errno=16
Device or resource busy
```

## 3. 当前数据链路

### 3.1 总图

```text
                        D455 彩色图像
                       /sensing/camera/camera0/color/image_raw
                                  │
                 ┌────────────────┴─────────────────┐
                 │                                  │
      内置 Autoware YOLOX 7 类            自定义 YOLO26s 3 类
           /.../rois0                    /.../custom/rois0
                 │                                  │
                 └────────── ROI Merger ────────────┘
                                  │
                    /.../camera_lidar_fusion/rois0
                                  │
Mid-360 点云 → 地面分割 → 低矮裁剪 → 体素欧式聚类
                                  │
                         roi_cluster_fusion
                                  │
                shape estimation → 目标过滤 → tracking
                                  │
                    multi_object_tracker → prediction
                                  │
                        behavior/static avoidance
                                  │
                  motion velocity planner → control

旁路：低矮小障碍物
Mid-360 原始点云 → irregular crop/ground segmentation
自定义 3 类 ROI → roi_pointcloud_fusion
           → /.../irregular_object/objects
           → camera_lidar_fusion_irregular 跟踪通道
```

### 3.2 D455 相机接口

配置文件：

- [camera.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/sensor_kit/sample_sensor_kit_launch/livox_sensor_kit_launch/launch/camera.launch.xml)

当前配置：

- 分辨率与帧率：`640x480x15`
- 只启用彩色流
- 关闭深度、对齐、点云、红外流和滤波后处理
- 发布 TF
- 将 RealSense 原生话题 relay 为 Autoware `camera0` 接口：

```text
/sensing/camera/camera0/color/image_raw
  → /sensing/camera/camera0/image_rect_color

/sensing/camera/camera0/color/camera_info
  → /sensing/camera/camera0/camera_info
```

`image_rect_color` 在这里只是话题兼容 relay，不是真正的去畸变图像。因此融合参数中启用了：

```yaml
point_project_to_unrectified_image: [true, false, false, false, false, false]
```

让投影过程使用 `CameraInfo` 的畸变系数。

### 3.3 内置 7 类 YOLOX

配置文件：

- [tier4_perception_component.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml)
- [autoware.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml)

当前模型：

```text
/home/tang/autoware_data/ml_models/tensorrt_yolox/
  yolox-sPlus-T4-960x960-pseudo-finetune.onnx
  yolox-sPlus-T4-960x960-pseudo-finetune.engine
  yolox-sPlus-T4-960x960-pseudo-finetune.json
  label.txt
```

当前使用的是只做检测、不做语义分割的 Autoware 标准 7 类模型。

TensorRT 关键点：

- 当前 `autoware.launch.xml` 显式将 TensorRT 8.5.2 目录加入 `LD_LIBRARY_PATH`。
- 原先现有的 `yolox-sPlus-opt.engine` 是在 compute 8.6 设备上生成的，当前 RTX 3060 是 compute 6.1，TensorRT 明确拒绝反序列化。
- 本机已为当前 GPU 重新生成 `yolox-sPlus-T4-960x960-pseudo-finetune.engine`，大小约 17.3 MB。
- 不要从其他机器直接拷贝 TensorRT engine。优先在同机重新生成，或至少确保 TensorRT 版本和 GPU compute capability 完全一致。

今天还尝试过迁移到 TensorRT 10/CUDA 12.8，过程中 GPU 进入 `NVRM Xid 62` 状态，需要重启系统。最终当前运行方案是 TensorRT 8.5.2 加本机缓存，不要在没有理解背景时回退或覆盖这个环境配置。

### 3.4 自定义 3 类 YOLO

自定义工作区：

```text
/home/tang/tsy/D455_ROS2_Humble
```

该目录当前没有 `.git` 元数据，修改不受 Autoware 超级工程版本管理保护。改代码前应手工备份。

模型：

```text
/home/tang/tsy/D455_ROS2_Humble/src/d455_perception/runtime/models/best.pt
SHA256: 6e40b1e26cee2ef2c8a695f798ac8863c60696692408f80fac8eb59cea1f13f0
```

类别映射：

| 自定义类别 | Autoware 类别 |
|---|---|
| `car` | `CAR` |
| `xingren` | `PEDESTRIAN` |
| `shidunzi` | `UNKNOWN` |

关键文件：

- [custom_roi_detector.py.in](/home/tang/tsy/D455_ROS2_Humble/src/d455_perception/scripts/custom_roi_detector.py.in)
- [roi_merger.py.in](/home/tang/tsy/D455_ROS2_Humble/src/d455_perception/scripts/roi_merger.py.in)
- [autoware_fusion.launch.xml](/home/tang/tsy/D455_ROS2_Humble/src/d455_perception/launch/autoware_fusion.launch.xml)

`custom_roi_detector` 的实现原则：

- 使用单独后台线程做 PyTorch/Ultralytics 推理。
- 只保留最新待处理帧，不积压图像。模型慢时旧帧会被丢弃，不容易因为帧队列增长而卡死。
- 输出标准 `tier4_perception_msgs/msg/DetectedObjectsWithFeature`，不是自定义 JSON。
- 输出话题：

```text
/perception/object_recognition/detection/custom/rois0
/perception/object_recognition/detection/custom_yolo_roi_detector/debug/image_annotated
```

实测性能约为 `6.2 Hz`，初始加载和 warm-up 会短时占用 GPU。

### 3.5 ROI Merger

ROI Merger 输入：

```text
内置: /perception/object_recognition/detection/rois0
自定义: /perception/object_recognition/detection/custom/rois0
```

输出：

```text
/perception/object_recognition/detection/camera_lidar_fusion/rois0
```

实现原则：

1. 内置 YOLOX ROI 始终作为兜底。
2. 两个模型框做 IoU/containment 匹配，默认匹配阈值为 `0.45`。
3. 不重叠框取并集。
4. 同类别重叠时自定义框优先。
5. 自定义 `shidunzi → UNKNOWN` 只优先覆盖内置 `UNKNOWN`，不随意覆盖 CAR、PEDESTRIAN 等已知类别。
6. 自定义 CAR/PEDESTRIAN 要覆盖冲突类别，需要同时达到“原始高置信阈值”和“加权后冲突阈值”。
7. 行人/车辆在短暂低置信或短暂漏检时，采用有限语义保持，避免标签逐帧跳变。
8. 自定义 ROI 超过 `0.35 s` 未更新，或与内置消息时间差超过 `0.2 s`，合并器自动退回内置 ROI。

有效分数计算：

```text
effective_score = clamp(raw_confidence × source_weight × class_weight, 0, 1)
```

当前权重：

| 参数 | 当前值 |
|---|---:|
| 来源权重 | 1.15 |
| CAR 类权重 | 1.15 |
| PEDESTRIAN 类权重 | 1.30 |
| UNKNOWN/石墩类权重 | 1.05 |

当前置信门槛：

| 类别 | 接入阈值 | 高置信冲突覆盖阈值 |
|---|---:|---:|
| PEDESTRIAN | 0.50 | 0.75 |
| CAR | 0.45 | 0.80 |
| UNKNOWN/石墩 | 0.40 | 0.80 |

另外，为过滤走廊灯具被误识别为车辆的问题，自定义检测器对 `car` 单独增加了 `0.60` 的最低阈值。该阈值高于统一模型接入参数 `0.4`。

语义保持：

```text
最多 3 帧
最多 0.30 s
```

冲突覆盖：

```text
自定义行人覆盖冲突类别最低有效分：0.55
自定义车辆覆盖 TRUCK/BUS 等具体车辆类别最低有效分：0.70
```

### 3.6 主 LiDAR 聚类与 ROI 融合

主规则聚类输入已从原始拼接点云改为：

```text
/perception/obstacle_segmentation/single_frame/pointcloud
```

原因：Autoware 的体素欧氏聚类在这个分支中实际按二维距离连接。直接把原始点云送入聚类，会把地面、墙脚和附近障碍物通过连续点带粘成一个大簇。

当前聚类参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `tolerance` | 0.30 m | 减小墙角和相邻结构粘连 |
| `voxel_leaf_size` | 0.15 m | 保留窄走廊细小结构 |
| `min_points_number_per_voxel` | 2 | 去除单点体素噪声 |
| `min_cluster_size` | 8 | 降低短寿命小簇闪烁 |
| `max_points_per_voxel_in_large_cluster` | 20 | 保留更多点以稳定大簇轮廓 |
| `use_height` | false | 该实现仍按二维连通域聚类 |
| 低矮裁剪 Z | `-0.2 ~ 2.5 m` | 去地面并保留车身以上目标 |

当前融合参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| ROI 时间偏移 | `-0.075 s` | D455 ROI 比 Mid-360 点云平均早约 74 ms |
| ROI 噪声窗口 | `0.08 s` | 吸收相机检测、传输和推理时间抖动 |
| 3D 点云噪声窗口 | `0.05 s` | 吸收雷达时间抖动 |
| ROI 超时 | `0.8 s` | 允许低帧率视觉结果 |
| 3D 超时 | `1.0 s` | 允许聚类和传输等待 |
| `only_allow_inside_cluster` | `false` | 当前允许部分重叠，优先恢复稀疏行人关联 |
| `roi_scale_factor` | `1.3` | 放大投影 ROI |
| CAR IoU | 0.30 | 过滤弱匹配 |
| PEDESTRIAN IoU | 0.15 | 容忍稀疏点云 |
| UNKNOWN IoU | 0.10 | 保留石墩和未知障碍 |
| `remove_unknown` | `false` | 不允许删除所有没被相机命中的障碍 |
| 行人尺寸范围 | 0.15 ~ 0.85 m | 防止大货架或墙面被标成行人 |

时间偏移调试方法：

1. 同时采集融合 ROI 和点云消息的 `header.stamp`。
2. 计算：

```text
offset = roi_timestamp - pointcloud_timestamp
```

3. 取稳定样本的中位数，不要用单帧。
4. 修改 `rois_timestamp_offsets` 后再次采样，残差应接近 0。

今天实测原始偏移约为 `-73.7 ms`，设置 `-0.075 s` 后残差约 `-1.4 ms`。

### 3.7 低矮小障碍物支路

主地面分割会把低矮石墩的大部分点删掉。实测左侧石墩：

```text
原始 LiDAR：约 46 ~ 51 点
地面分割后：约 1 ~ 2 点
普通主聚类最低点数：8
```

因此主聚类无法稳定形成石墩 3D 簇。当前保留 Autoware 原生 `irregular_object` 支路：

- 输入仍为 Mid-360 原始点云
- 支路内部先做 crop box
- 再做独立的地面分割
- 使用自定义石墩 ROI，而不是内置 7 类 ROI：

```text
/perception/object_recognition/detection/custom/rois0
```

- 使用真实存在的图像：

```text
/sensing/camera/camera0/image_rect_color
```

当前 crop 和融合参数：

| 参数 | 当前值 |
|---|---:|
| 前方范围 | 0.5 ~ 60.0 m |
| 横向范围 | -10.0 ~ 10.0 m |
| 高度范围 | -0.5 ~ 1.0 m |
| `roi_pointcloud_fusion.min_cluster_size` | 2 |
| `cluster_2d_tolerance` | 0.4 m |
| `max_object_size` | 2.0 m |

石墩实测结果：

```text
39 帧中 34 帧有目标
累计 74 个低矮 UNKNOWN 检测
高度约 0.03 ~ 0.40 m
典型位置 x=4.46 m,y=2.36 m 与 x=5.30 m,y=2.37 m
```

这些结果与左侧石墩位置和尺寸吻合，说明石墩已经进入 Autoware 原生 `roi_pointcloud_fusion`。

### 3.8 跟踪链路

此前多目标跟踪器一直显示：

```text
Detection status: Not initialized
Measurement time is not set
```

根因是 tracker 第 1 路输入指向：

```text
/perception/object_recognition/detection/clustering/validation/objects
```

但该话题在当前规则聚类模式下没有发布者。tracker 输入管理器等待第 1 路触发，形成死锁。

当前修改：

- `detection01` 改为持续有效的：

```text
/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects
```

- `detection03` 与新的 `detection01` 重复，设为 `none`
- `detection04` 继续接收 `irregular_object` 支路
- tracker 参数启用 tentative objects

修复后实测：

```text
tracked: 79 帧中 60 帧非空
tentative: 78 帧中 75 帧非空
predicted: 78 帧中 59 帧非空
```

跟踪输出恢复，但部分目标仍可能保持 `UNKNOWN`，这是语义匹配和点云形状问题，不等于运动目标 3D 几何完全丢失。

### 3.9 RViz 可视化

当前主要图层：

| 图层 | 话题 |
|---|---|
| 自定义三类 YOLO 图像框 | `/perception/object_recognition/detection/custom_yolo_roi_detector/debug/image_annotated` |
| 相机雷达融合叠加图 | `/perception/object_recognition/detection/clustering/camera_lidar_fusion/roi_cluster_fusion/debug/image_raw0` |
| 原始融合 3D 框 | `/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects` |
| tentative 跟踪框 | 多目标跟踪器 `debug/tentative_objects` |
| 稳定跟踪框 | `/perception/object_recognition/tracking/objects` |
| 最终预测对象 | `/perception/object_recognition/objects` |

原始融合框已恢复默认显示，并改为黄色，避免白色 UNKNOWN 框与白色点云混在一起。稳定跟踪尚无目标时，可先看原始融合框和 tentative 框区分“检测存在”与“跟踪成熟”。

## 4. 走廊规划与 MRM 急停

### 4.1 一次紧急停车的真实原因

现场约 19:52:08 触发 `EMERGENCY_STOP`。

日志证据：

- AEB、碰撞检测、规划校验在事件时刻没有报告检测到碰撞。
- 行为路径规划器约在 `19:52:03.47` 后停止产生新结果。
- 约 `19:52:08.15`，MRM 切换为 `EMERGENCY_STOP`。
- 直接原因是 `/autoware/planning/topic_rate_check/trajectory ERROR`，即规划轨迹超时。

因此这次急停首先应按“规划器卡顿/轨迹不连续”处理，而不是继续缩小避障距离。

当时的高计算风险：

- 300 m 路径按 `0.2 m` 输出重采样，约 1500 个点。
- 速度规划碰撞检查也按 `0.2 m` 步长。
- 静态避让同时启用 `shift_line` 和优化路径生成，窄走廊下单侧静止行人场景没有必要同时计算两套路径。

已做的针对性修改：

- 静态避让路径输出重采样从 `0.2 m` 改为 `0.5 m`
- 避让路径生成从 `both` 改为确定性的 `shift_line_base`
- 最大横移从 `0.7 m` 改为 `0.6 m`
- 道路边界硬余量改为 `0.10 m`
- 速度规划碰撞检查步长改为 `0.5 m`

之后用户要求参考 `autoware_1122` 的 UNKNOWN 参数，并让 PEDESTRIAN 与 UNKNOWN 完全一致，因此当前静态避让目标参数是：

```text
th_moving_speed: 0.50
th_moving_time: 0.0
longitudinal_margin: 0.0
soft_margin: 0.25
hard_margin: 0.15
hard_margin_for_parked_vehicle: 0.12
max_expand_ratio: 0.0
envelope_buffer_margin: 0.1
th_error_eclipse_long_radius: 0.6
```

`UNKNOWN` 与 `PEDESTRIAN` 在上面的字段和数值上完全一致。

### 4.2 当前静态避让参数

关键文件：

- [static_obstacle_avoidance.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml)

| 参数 | 当前值 |
|---|---:|
| 规划重采样 | 0.15 m |
| 输出重采样 | 0.50 m |
| 静态检测前向范围 | 1.5 ~ 4.5 m |
| 检测后向范围 | 2.0 m |
| 最大左右横移 | 0.60 m |
| 最大离参考线距离 | 0.60 m |
| 软道路边界余量 | 0.12 m |
| 硬道路边界余量 | 0.10 m |
| 最小绕行准备距离 | 0.50 m |
| 最小准备时间 | 0.05 s |
| 最大准备时间 | 0.30 s |
| 标称绕行速度 | 1.4 m/s |
| 路径生成方式 | `shift_line_base` |

行人 `safety_check.target_type.pedestrian` 当前为 `false`，与 `unknown: false` 一致。这意味着规划层不再保留行人独立的额外安全检查；感知层仍会把行人识别为 PEDESTRIAN。这个修改降低了行人额外保护，仅应按现场需求谨慎保留。

### 4.3 当前 obstacle_stop 参数

关键文件：

- [obstacle_stop.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml)

| 参数 | 当前值 |
|---|---:|
| 前向停车余量 | 0.30 m |
| 终端停车余量 | 0.30 m |
| 最小行为停车余量 | 0.30 m |
| UNKNOWN 侧向停车余量 | 0.08 m |
| PEDESTRIAN 侧向停车余量 | 0.08 m |
| default/pointcloud 侧向停车余量 | 0.15 m |
| UNKNOWN/PEDESTRIAN 检测高度上限 | 1.0 m |
| 检测高度下限 | -0.5 m |
| `check_outside.unknown` | false |
| `check_outside.pedestrian` | false |
| 点云轨迹裁剪最短长度 | 5.0 m |

设计意图是让停车侧向余量 `0.08 m` 小于绕行硬余量 `0.15 m`，避免车辆已经在绕行时被更宽的停车边界拦住。实际仍要验证车角扫掠、跟踪误差和地图边界误差。

### 4.4 2.3 m 走廊的几何判断

车辆宽 0.572 m，走廊半宽 1.15 m。若参考线居中并给墙保留 0.15 m：

```text
直行车身纵轴最大横移
= 1.15 - 0.572/2 - 0.15
= 0.714 m
```

当前最大横移限制为 `0.6 m`，名义上留出约 0.114 m 的额外余量。但转弯时车角扫过范围更大，且墙面错误大簇会消耗可行驶空间。因此不能只看直行几何就认为一定能安全绕行。

### 4.5 当前 motion velocity planner 参数

关键文件：

- [motion_velocity_planner.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/motion_velocity_planner.param.yaml)

```yaml
trajectory_polygon_collision_check:
  decimate_trajectory_step_length: 0.5
```

该值用于在碰撞检查精度和实时计算量之间折中。

## 5. 今天的主要问题、根因和修改

| 问题 | 根因 | 处理 | 当前状态 |
|---|---|---|---|
| D455 报 Depth stream start failure | 相机曾降到 USB 2.0，彩色和深度流带宽不足 | 改接 USB 3.x，确认 `5000M` | 已解决；最终融合不用深度 |
| D455 帧率只有约 4.3 Hz | USB 2.0 和流配置不稳定 | 修复 USB 链路，最终只开彩色流 | 彩色流恢复 |
| `errno=16 Device or resource busy` | 相机或 relay 成为孤儿进程，重复占用 `/dev/video*` | 找 PID 后先 SIGTERM，必要时 SIGKILL，再确认设备释放 | 已解决 |
| 纯 LiDAR 模式没有相机融合 | `perception_mode` 默认 lidar，相机默认关闭 | 默认改为 `camera_lidar_fusion`，并贯通 `launch_camera` 参数；相机默认保持关闭，需要时显式开启 | 融合已实车验证 |
| 内置 YOLOX 启动失败 | engine 在 compute 8.6 生成，当前 GPU 为 compute 6.1 | 切换 detection-only ONNX 并在本机生成 engine | 已解决 |
| RViz 没有相机图像框 | 曾关闭内置 7 类 YOLOX，且 RViz 订阅错误话题 | 恢复 YOLOX，增加自定义/融合图像话题 | 已修复 |
| ROI 有框但融合 3D 目标无语义 | ROI 与点云时间不匹配、IoU/尺寸匹配失败 | 偏移 `-0.075 s`、噪声窗口、IoU、尺寸校验 | 部分稳定，仍可能 UNKNOWN |
| 语义在 PEDESTRIAN/UNKNOWN 间跳变 | 低置信检测、同步漏帧、聚类过大、瞬时漏检 | 分两级阈值、权重、语义保持、限制目标尺寸 | 已缓解 |
| RViz 主视图没有障碍物 | 原始融合层被关闭，tracking 输出为空 | 恢复原始融合层，启用 tentative 输出 | 已修复 |
| tracker 始终 `Not initialized` | 首路输入对应话题没有发布者 | detection01 改为 camera_lidar_fusion 输出，关闭重复第三路 | 已修复 |
| 地面被聚类成障碍物 | 聚类读原始点云；二维连通把地面与墙脚连接 | 主聚类改读地面分割后的单帧点云，裁剪 Z | 已修复主链路 |
| 走廊墙框过大 | 融合前已有约 10 m 长墙簇，凸包包住空白 | 收紧到 0.3 m 容差、0.15 m 体素、增加点数过滤 | 改善，未根治 |
| 石墩 ROI 没有 3D 结果 | 地面分割把石墩点删到 1~2 点；小障碍支路订阅错 ROI/图像 | 启用原生 irregular 支路和自定义 ROI | 已实车验证 |
| 行人不进入 planning | tracking 首路死锁，输出全空 | 修复 tracker 输入和初始化 | 已修复 |
| 绕行一半停车/急停 | 行为规划轨迹超时触发 MRM，非 AEB 碰撞 | shift_line_base、放稀路径和碰撞检查、协调余量 | 参数已改，最终现场待复测 |
| 灯具被识别成 CAR | 自定义模型低置信误检 | car 单独提高最低置信度到 0.60 | 已改，需持续观察误检/漏检 |

## 6. 中间实验与当前不再存在的文件

今天上午曾实现过：

- D455 RGB-D 低矮障碍物三维补盲节点
- D455 ROI 深度点过滤后并入占据栅格

当时相关文件包括：

```text
d455_rgbd_low_obstacle.py
d455_occupancy_cloud_merger.py
```

这些文件和对应 launch 接线当前已经不在正式工作区中。新的 Agent 如果看到旧对话中的链接，不要误以为它们仍然存在。当前正式方案是：

```text
自定义石墩 ROI
  → Autoware 原生 irregular_object / roi_pointcloud_fusion
  → 多目标跟踪
```

如果未来要恢复 RGB-D 补盲，应重新设计并重新验证，不应直接照搬旧路径。

## 7. 当前改动文件清单

### 7.1 Autoware launch 和感知配置

- [autoware.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml)
  - 相机、感知模式、自定义模型参数、TensorRT 8.5 环境
- [tier4_perception_component.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml)
  - 内置 YOLOX 模型、ROI 合并节点、融合 ROI 话题切换
- [tier4_sensing_component.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_sensing_component.launch.xml)
  - 传递相机启动参数
- [sensing.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_sensing_launch/launch/sensing.launch.xml)
  - Livox sensor kit 才传递 `launch_camera`
- [camera.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/sensor_kit/sample_sensor_kit_launch/livox_sensor_kit_launch/launch/camera.launch.xml)
  - D455 彩色流和 Autoware 话题适配
- [camera_lidar_detector.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch/launch/object_recognition/detection/detector/camera_lidar_detector.launch.xml)
  - 聚类输入改为地面分割点云，启用内置 2D 检测
- [camera_lidar_irregular_object_detector.launch.py](/home/tang/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch/launch/object_recognition/detection/detector/camera_lidar_irregular_object_detector.launch.py)
  - 小障碍支路使用自定义 ROI 和实际图像话题
- [tracking.launch.xml](/home/tang/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch/launch/object_recognition/tracking/tracking.launch.xml)
  - 修复 tracker 第 1 路输入
- [fusion_common.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/image_projection_based_fusion/fusion_common.param.yaml)
  - 时间偏移、窗口、超时、未去畸变投影
- [roi_cluster_fusion.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/image_projection_based_fusion/roi_cluster_fusion.param.yaml)
  - IoU、ROI 缩放、行人尺寸
- [voxel_grid_based_euclidean_cluster.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml)
  - 走廊聚类参数
- [irregular_object_detector.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/irregular_object_detection/irregular_object_detector.param.yaml)
  - 低矮石墩支路
- [multi_object_tracker_node.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/tracking/multi_object_tracker/multi_object_tracker_node.param.yaml)
  - tentative objects 输出
- [autoware.rviz](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz)
  - 图像、原始融合、tentative 和 tracking 显示

### 7.2 规划配置

- [static_obstacle_avoidance.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml)
- [obstacle_stop.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml)
- [motion_velocity_planner.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/motion_velocity_planner.param.yaml)
- [obstacle_slow_down.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml)
- [obstacle_cruise.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml)
- [road_user_stop.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/road_user_stop.param.yaml)
- [run_out.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/run_out.param.yaml)
- [dynamic_obstacle_avoidance.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml)
- [avoidance_by_lane_change.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml)
- [lane_change.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml)
- [start_planner.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/start_planner/start_planner.param.yaml)
- [goal_planner.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/goal_planner/goal_planner.param.yaml)
- [crosswalk.param.yaml](/home/tang/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml)

### 7.3 外部 D455 工作区

- [custom_roi_detector.py.in](/home/tang/tsy/D455_ROS2_Humble/src/d455_perception/scripts/custom_roi_detector.py.in)
- [roi_merger.py.in](/home/tang/tsy/D455_ROS2_Humble/src/d455_perception/scripts/roi_merger.py.in)
- [autoware_fusion.launch.xml](/home/tang/tsy/D455_ROS2_Humble/src/d455_perception/launch/autoware_fusion.launch.xml)
- [CMakeLists.txt](/home/tang/tsy/D455_ROS2_Humble/src/d455_perception/CMakeLists.txt)

## 8. 备份与回滚

今天至少有以下备份目录：

```text
.backups/corridor_noise_20261122_185046
.backups/pedestrian_unknown_all_20261122_185339
.backups/corridor_margin_20261122_190636
.backups/corridor_coordinated_20261122_192801
.backups/corridor_longitudinal_20261122_193144
.backups/corridor_longer_smoother_20261122_193810
```

其中：

- `corridor_noise` 保存聚类和初次行人/UNKNOWN 调整前的配置。
- `pedestrian_unknown_all` 保存 11 个规划文件的批量修改前版本，并有 `changes.json` 记录字段。
- `corridor_margin` 保存横纵余量协调前配置。
- `corridor_coordinated` 保存再次协调前配置。
- `corridor_longitudinal` 保存纵向距离调整前配置。
- `corridor_longer_smoother` 保存 MRM 修复前配置。

回滚前先确认源文件当前内容，再把备份复制回对应路径。不要直接把整个备份目录覆盖到 `src`，因为备份只保存了部分单文件。

回滚后至少要重新检查：

```bash
git -C /home/tang/autoware/src/launcher/autoware_launch diff --check
ros2 param dump /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner
```

## 9. 验证命令

### 9.1 启动前

检查 D455 USB 链路：

```bash
lsusb -t
lsusb | grep -i "Intel.*RealSense"
```

D455 应显示 `5000M` 或 `10000M`，不能是 `480M`。

检查设备占用：

```bash
pgrep -af 'realsense2_camera_node|camera0_image_rect_color_relay|camera0_camera_info_relay'
for dev in /dev/video*; do
  [ -e "$dev" ] && fuser -v "$dev"
done
```

检查 GPU：

```bash
nvidia-smi
```

### 9.2 感知链

```bash
ros2 topic hz /sensing/camera/camera0/image_rect_color
ros2 topic hz /sensing/camera/camera0/camera_info
ros2 topic hz /sensing/lidar/concatenated/pointcloud
ros2 topic hz /perception/object_recognition/detection/rois0
ros2 topic hz /perception/object_recognition/detection/custom/rois0
ros2 topic hz /perception/object_recognition/detection/camera_lidar_fusion/rois0
ros2 topic hz /perception/object_recognition/detection/clustering/camera_lidar_fusion/objects
ros2 topic hz /perception/object_recognition/tracking/objects
ros2 topic hz /perception/object_recognition/objects
```

参考频率：

```text
D455 彩色：约 12 ~ 14 Hz
内置 YOLOX / 合并 ROI：约 12 Hz
自定义 3 类 YOLO：约 6 Hz
Mid-360：约 10 Hz
融合聚类对象：约 4 Hz
```

检查石墩支路：

```bash
ros2 topic hz /perception/object_recognition/detection/irregular_object/objects
ros2 topic echo /perception/object_recognition/detection/irregular_object/objects --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link camera0_color_optical_frame
ros2 run tf2_ros tf2_echo map base_link
```

没有 `map -> base_link` 时先完成手动定位。

### 9.3 跟踪和规划

```bash
ros2 node info /perception/object_recognition/tracking/multi_object_tracker
ros2 param get /perception/object_recognition/tracking/multi_object_tracker publish_tentative_objects
ros2 topic echo /perception/object_recognition/tracking/objects --once --field objects
ros2 topic echo /perception/object_recognition/objects --once --field objects
```

诊断中不应再看到：

```text
Detection status: Not initialized
Measurement time is not set
```

绕行测试时同时记录：

```text
/planning/trajectory
/autoware/planning/topic_rate_check/trajectory
/planning/scenario_planning/lane_driving/behavior_planning/debug/static_obstacle_avoidance/candidate_paths
/perception/object_recognition/objects
```

如果再次急停，先判断是否为轨迹 topic rate timeout，不要先假定 AEB 或 obstacle_stop 的参数是根因。

### 9.4 静态检查

今天已执行的检查包括：

```text
XML 可解析
YAML 可解析
irregular object Python launch 可编译
Autoware launch 与 D455 包多次 colcon build 通过
合成消息测试通过
```

2026-11-22 最终复核时发现：

```text
git diff --check 在 obstacle_stop.param.yaml 第 104 行报告一个行尾空格
```

该空格不影响运行，但提交前应删除。

## 10. 停止和清理

优先在启动终端按一次 `Ctrl+C`，等待 launch 输出各进程正常退出。

如果使用仓库的 tmux 启动脚本：

```bash
tmux ls
tmux kill-session -t autoware
```

检查残留：

```bash
pgrep -af 'autoware.launch.xml|component_container|rviz2|ros2 launch autoware_launch'
pgrep -af 'realsense2_camera_node|camera0_image_rect_color_relay|camera0_camera_info_relay|custom_roi_detector|custom_roi_merger'
```

按确认过的 PID 先正常终止：

```bash
kill -TERM <PID1> <PID2>
```

只有仍未退出时再做一次验证后的强制终止。不要使用过于宽泛的 `pkill -9 -f camera` 或 `pkill -9 -f autoware`，容易误杀其他 ROS 任务或终端。

清理 ROS CLI 缓存：

```bash
ros2 daemon stop
ros2 daemon start
```

关闭阶段的个别容器或 Livox 驱动出现 `-11` 已多次观察到，通常发生在统一 shutdown 阶段。关键判断是正常运行期间是否持续出数据，而不是只看关闭时的最后退出码。

## 11. 已知风险和未完成事项

### P0：下一次上车前必须确认

1. D455 仍在 USB 3.x，彩色流稳定，没有残缺帧超时。
2. `custom_roi_detector` 和 `roi_merger` 均已启动，合并 ROI 有发布。
3. tracker 已初始化，tracked/tentative/predicted 均非空。
4. 手动定位完成，`map -> base_link` 有效。
5. RViz 中原始融合框、tentative、tracking 至少一层能看到目标。
6. 走廊测试先低速，确认候选绕行路径和实际轨迹连续。
7. 检查 `/autoware/planning/topic_rate_check/trajectory` 是否再次超时。

### P1：当前最值得继续处理

1. 长墙面仍会形成大簇。当前 0.3 m/0.15 m 参数没有彻底解决墙框跨路，需要从墙面分段、静态地图差分或形状估计层继续处理。
2. 行人和 UNKNOWN 在规划层被对齐后，行人的额外安全保护减少。需要和现场安全负责人确认这是否符合预期。
3. 自定义 YOLO 的 0.60 car 阈值是用灯具误检换来的。要同时统计真实车辆召回率。
4. D455 工作区没有版本控制。建议初始化独立 git 或至少按日期备份 `custom_roi_detector.py.in`、`roi_merger.py.in` 和 launch。
5. `obstacle_stop.param.yaml` 注释与实际车辆参数不一致，应修正。
6. 最终规划参数只做了静态校验，未完成同日最终实车复测。

### P2：后续优化

1. 用 rosbag 离线复现墙、门框、石墩、行人和车辆五类场景。
2. 为行人语义增加距离相关的置信度和保持策略，不要全局降低门槛。
3. 统计自定义模型的 car/pedestrian/bollard 精确率、召回率和混淆矩阵。
4. 如需要无 LiDAR 补盲，重新设计 RGB-D 通道，并接入标准 `DetectedObjects` 或占据栅格。
5. 对长墙按局部几何分段，避免一个凸包覆盖整段墙面。

## 12. 给新 Agent 的标准排查顺序

遇到“看不见/不避障/乱避障”时，按以下顺序查，不要跳级：

```text
1. 进程和 USB
   D455、Mid-360、GINS、底盘是否都在？

2. 原始传感器
   camera image/info、LiDAR pointcloud 是否连续？

3. 视觉 ROI
   内置 rois0、custom/rois0、merged rois0 是否都有预期消息？

4. LiDAR 几何
   地面分割点云、crop 后点云、聚类 clusters 是否有目标？

5. 融合
   camera_lidar_fusion/objects 是否有非空目标，frame_id 和时间是否合理？

6. 跟踪
   tentative/tracked 是否有目标？tracker 是否 initialized？

7. 预测/规划
   prediction objects 是否出现，route 是否建立？

8. 行为与速度规划
   候选绕行路径、最终轨迹、topic rate check、MRM 条件分别是什么？

9. 控制
   mode、Autoware Control、vehicle command 是否真正进入 autonomous？
```

每一步都用 topic 的实际消息证明，不能只看节点存在。ROS 2 节点存在不等于话题有发布者，话题存在也不等于消息非空。

## 13. 术语速查

| 术语 | 解释 |
|---|---|
| ROI | 相机二维检测框，不是三维目标 |
| LiDAR cluster | 点云通过距离连通形成的聚类 |
| ROI cluster fusion | 把相机 ROI 投影到三维，并给 LiDAR 簇赋类别 |
| irregular object | Autoware 面向低矮、小尺寸或非标准障碍物的支路 |
| tracker | 在时间上关联检测，输出稳定目标 |
| prediction | 预测目标未来轨迹 |
| MRM | Minimal Risk Maneuver，系统降级时的最小风险操作 |
| tentative object | 已建立但还未达到稳定输出条件的跟踪目标 |
| frame_id | ROS 消息坐标所属的坐标系 |
| TF | 两个坐标系之间的变换 |

## 14. 文件位置总索引

当前核心源码和配置都在：

```text
/home/tang/autoware/src/launcher/autoware_launch
```

外部 D455 代码：

```text
/home/tang/tsy/D455_ROS2_Humble
```

内置模型缓存：

```text
/home/tang/autoware_data/ml_models/tensorrt_yolox
```

地图：

```text
/home/tang/autoware_map/map_task4
```

今天配置备份：

```text
/home/tang/autoware/.backups
```

今天录屏转换产物：

```text
/home/tang/autoware/录屏 2026年11月22日 20时13分04秒.mp4
/home/tang/autoware/录屏 2026年11月22日 20时13分04秒.zip
```

MP4 为 H.264、1848×970、51.316 秒、无音轨，约 27 MB。

## 15. 交接结论

今天的感知部分已经从“纯雷达、没有视觉语义”推进到“双视觉模型加原生相机雷达融合”，并且解决了几处会直接导致 3D 目标为空的实际断链。低矮石墩通过 Autoware 原生小障碍物支路能够进入环境，行人也已经能从检测进入 tracking 和 prediction。

走廊部分仍处于“参数已协调、代码可运行、现场行为需要最终复测”的阶段，尤其要区分：

- 目标没有被感知到；
- 目标被感知到但不在路径/车道；
- 已经生成绕行路径但停车模块判定不安全；
- 规划轨迹中断导致 MRM 急停。

下一次测试不要从继续盲目缩小横纵余量开始。先保存一份 rosbag 和 RViz 截图，再按“传感器 → ROI → 聚类 → 融合 → 跟踪 → 预测 → 行为规划 → 速度规划 → 控制/MRM”的顺序找到第一个输出断点，这样才能避免本次走廊调参中出现的反复试错。
