# Livox MID-360 实车探测距离排查建议

## 结论摘要

当前工程的 `sensor_model:=livox_sensor_kit` 使用 Livox MID-360。官方标称探测能力为：

- 最小探测距离：0.1 m
- 10% 反射率目标：最大约 40 m
- 80% 反射率目标：最大约 70 m
- 有效距离随垂直视场区域变化，越靠近上方视场有效距离越短

因此，实车上如果障碍物是深色、低反射率、尺寸较小、处在上方视场边缘，或距离超过约 30-40 m，点云变稀甚至消失是可能的。planning simulator 能绕行只说明规划模块在收到理想障碍物时可以工作，不能证明实车 LiDAR 感知链路正常。

## 当前工程中的实车点云链路

启动命令中使用：

```bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_6 \
  vehicle_model:=hunter_vehicle \
  sensor_model:=livox_sensor_kit
```

对应 LiDAR launch：

```text
src/launcher/autoware_launch/sensor_kit/sample_sensor_kit_launch/livox_sensor_kit_launch/launch/lidar.launch.xml
```

实际数据流为：

```text
livox_ros_driver2
  /sensing/lidar/front/pointcloud_raw
autoware_livox_tag_filter
  /sensing/lidar/front/pointcloud_raw_ex
topic_tools relay
  /sensing/lidar/top/pointcloud_raw
topic_tools relay
  /sensing/lidar/concatenated/pointcloud
perception
  /perception/obstacle_segmentation/pointcloud
  /perception/object_recognition/objects
planning
```

注意：当前 `lidar.launch.xml` 中 `use_concat_filter` 默认是 `false`，并且后面用 relay 直接把 raw 点云转发到 `/sensing/lidar/concatenated/pointcloud`。这条链路简单可用，但没有真正使用 `PointCloudConcatenateDataSynchronizerComponent` 的时间同步/运动补偿输出。如果车辆运动时远处障碍物检测不稳定，建议后续改为真正的同步/去畸变链路，而不是 relay 旁路。

## 当前没有明显软件硬裁剪的位置

感知地面分割裁剪框：

```text
src/launcher/autoware_launch/autoware_launch/config/perception/obstacle_segmentation/ground_segmentation/ground_segmentation.param.yaml
```

关键参数：

```yaml
common_crop_box_filter:
  parameters:
    min_x: -100.0
    max_x: 150.0
    min_y: -70.0
    max_y: 70.0
```

这比 MID-360 的实际有效量程大，不是距离不足的主要限制。

占据栅格地图：

```text
src/launcher/autoware_launch/autoware_launch/config/perception/occupancy_grid_map/pointcloud_based_occupancy_grid_map.param.yaml
```

关键参数：

```yaml
map_length: 150.0
map_resolution: 0.5
```

也没有把探测范围裁到很近。

## 现场实测命令

先确认原始点云是否有数据：

```bash
ros2 topic hz /sensing/lidar/front/pointcloud_raw
ros2 topic hz /sensing/lidar/front/pointcloud_raw_ex
ros2 topic hz /sensing/lidar/concatenated/pointcloud
```

检查点云头信息和 frame：

```bash
ros2 topic echo --once /sensing/lidar/concatenated/pointcloud --field header
ros2 run tf2_ros tf2_echo base_link livox_front/livox_front_base_link
```

在 RViz 中同时显示以下话题，逐层判断障碍物在哪一步消失：

```text
/sensing/lidar/front/pointcloud_raw_ex
/sensing/lidar/concatenated/pointcloud
/perception/obstacle_segmentation/pointcloud
/perception/object_recognition/objects
/planning/scenario_planning/lane_driving/behavior_planning/path
```

建议现场放置三类目标，分别在 10 m、20 m、30 m、40 m 测试：

- 白色纸板或白墙：高反射，应该最远
- 纸箱/路锥：中等反射
- 黑色衣物/黑色塑料桶：低反射，最容易提前丢失

如果 `/sensing/lidar/front/pointcloud_raw_ex` 在 30-40 m 已经没有目标点，优先怀疑硬件、安装角度、目标反射率或环境；如果 raw 有点但 `/perception/obstacle_segmentation/pointcloud` 或 `/perception/object_recognition/objects` 没有，优先调感知参数。

## 修改建议

### 1. 不建议盲目把规划参数调大

绕行路径无法生成时，先确认 `/perception/object_recognition/objects` 是否稳定输出障碍物。若感知没有输出，调静态避障参数通常不会解决问题。

### 2. 建议改用真正的同步/预处理链路

当前 `livox_sensor_kit_launch/launch/lidar.launch.xml` 中使用 relay 直接发布 `/sensing/lidar/concatenated/pointcloud`。建议整理为：

- MID-360 输出 `/sensing/lidar/top/pointcloud_raw`
- Livox tag filter 输出 `/sensing/lidar/top/pointcloud_raw_ex`
- 进入 pointcloud preprocessor，输出 `/sensing/lidar/concatenated/pointcloud`

若只使用单个 MID-360，需要同步配置中的 `input_topics` 只保留一个输入，不要保留 right/top/left 三个默认输入。

### 3. 检查 Livox tag filter 是否过度过滤

当前配置：

```xml
<param name="ignore_tags" value="[1, 2]"/>
```

这会过滤低质量点。若远距离点云本来就稀疏，可以临时做 A/B 测试：

- 保持 `[1, 2]`，记录各距离点云数量
- 改为空列表 `[]`，只用于测试，不建议直接长期上车

如果关闭过滤后远距离障碍物明显出现，说明问题主要是远距离点质量不足，需要权衡安全性和误检。

### 4. 优先使用聚类检测验证 Livox

当前感知默认：

```xml
<arg name="lidar_detection_model" default="clustering"/>
```

这对验证 MID-360 更直接。若切换到 CenterPoint/TransFusion，要确认模型训练范围、模型输入范围和 MID-360 点云分布匹配，否则远处或小目标可能被 DNN 漏检。

### 5. 调整聚类参数时要保守

文件：

```text
src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml
```

当前：

```yaml
tolerance: 0.7
voxel_leaf_size: 0.3
min_points_number_per_voxel: 1
min_cluster_size: 10
max_cluster_size: 3000
```

实车 `perception_mode:=lidar` 时，规则检测链路走的是 `voxel_grid_based_euclidean_cluster`。如果远处障碍物在 obstacle segmentation 点云中存在，但无法形成 object，可以尝试：

```yaml
tolerance: 0.9
voxel_leaf_size: 0.4
min_cluster_size: 4
```

这会增加远距离稀疏点成簇概率，但也会增加误检风险。建议只在低速封闭场地测试。

同时检查 validator：

```text
src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/detection/detected_object_validation/obstacle_pointcloud_based_validator.param.yaml
```

当前每类 `min_points_num` 都是 10：

```yaml
min_points_num:
  [10, 10, 10, 10, 10, 10, 10, 10]
```

如果 clustering 已输出目标，但 `/perception/object_recognition/objects` 仍然没有目标，可临时把 UNKNOWN/CAR 等目标类别降低到 5 做对比测试：

```yaml
min_points_num:
  [5, 5, 5, 5, 5, 5, 5, 5]
```

不要一次性大幅降低所有过滤阈值。建议按顺序测试：先改 `min_cluster_size`，再改 `tolerance`，最后才改 validator。

### 6. 检查安装俯仰角和高度

MID-360 垂直 FOV 为 59 度，但不同垂直区域有效距离不同。若雷达安装俯仰角偏上，远处地面障碍物可能落在有效距离较差区域；若偏下，则远处前方障碍物可能被地面点和车身遮挡影响。建议在 RViz 里确认 20-40 m 前方障碍物是否落在点云主覆盖区。

## 推荐判断标准

- 高反射目标 40 m 左右仍有明显点云：雷达基本正常，重点查感知过滤和目标输出。
- 高反射目标 20-30 m 已明显缺失：重点查 MID-360 电源、网络丢包、安装遮挡、镜面污染、驱动配置。
- raw 点云可见，segmentation 不可见：查地面分割、crop box、height filter。
- segmentation 可见，objects 不可见：查聚类、object validation、lanelet/object filter。
- objects 可见，规划不绕行：再查 static obstacle avoidance 和行为路径规划参数。
