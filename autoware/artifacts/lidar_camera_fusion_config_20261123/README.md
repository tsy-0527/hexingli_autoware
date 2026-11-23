# D455 + Livox Mid-360 Camera–LiDAR 融合配置包

生成日期：2026-11-23

本包整理了当前车辆上 Autoware `camera_lidar_fusion` 的关键配置、定向 Git 差异和 D455 自定义 ROI 节点。用途是让接手者快速回答以下问题：

1. 融合模式从哪里开启；
2. 相机图像、CameraInfo、ROI 和点云如何接入；
3. ROI 与点云如何做时间同步和空间匹配；
4. D455 与 Mid-360 的 TF 外参在哪里；
5. 当前车辆针对走廊、行人和低矮障碍物做过哪些调优。

## 目录结构

```text
README.md                       本说明
MANIFEST.sha256                 文件完整性校验
diff/                           相对于 autoware_launch 当前 HEAD 的定向差异
files/src/launcher/...          保留 Autoware 原始相对路径的配置快照
files/d455_perception/...       外部 D455 自定义检测/ROI 合并节点快照
reference/                      完整现场交接文档
```

## 建议阅读顺序

### 1. 启动入口和参数传递

- `files/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`
  - 默认感知模式为 `camera_lidar_fusion`；
  - 暴露 `launch_camera`、`use_custom_d455_yolo` 及自定义检测阈值；
  - 向 sensing 和 perception component 继续传参。
- `files/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml`
  - 选择 `clustering` 检测模式；
  - 设置 D455 单相机 ROI；
  - 启动自定义 D455 ROI 检测与合并；
  - 将融合参数路径传给底层 perception launch。

### 2. Autoware 原生融合链路

- `files/src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch/launch/perception.launch.xml`
  - `mode=camera_lidar_fusion`；
  - 当前车辆仅有一个前视相机，因此 `image_number=1`；
  - camera0 的 image、CameraInfo、ROI 和点云继续传入 detection。
- `files/src/launcher/autoware_launch/tier4_universe_launch/tier4_perception_launch/launch/object_recognition/detection/detector/camera_lidar_detector.launch.xml`
  - 开启本机 2D YOLOX；
  - 聚类点云使用地面分割后的单帧点云；
  - 启动聚类、`roi_cluster_fusion`、形状估计并输出 3D objects。

### 3. 融合核心参数

- `fusion_common.param.yaml`
  - D455 ROI 相对 Mid-360 点云的时间偏移：`-0.075 s`；
  - ROI 超时：`0.8 s`；点云超时：`1.0 s`；
  - camera0 ROI 时间噪声窗口：`0.08 s`；
  - camera0 按未校正图像方式投影；
  - 开启融合 debug image。
- `roi_cluster_fusion.param.yaml`
  - `only_allow_inside_cluster=false`；
  - `roi_scale_factor=1.3`；
  - CAR IoU 阈值 `0.3`；PEDESTRIAN IoU 阈值 `0.15`；
  - 收紧行人三维宽度范围。

以上两个文件位于：

```text
files/src/launcher/autoware_launch/autoware_launch/config/perception/
  object_recognition/detection/image_projection_based_fusion/
```

### 4. 相机、话题和外参

- `livox_sensor_kit_launch/launch/camera.launch.xml`
  - D455 RGB 配置为 `640x480x15`；
  - 融合只使用 RGB、CameraInfo 和 TF，不使用深度；
  - 将 RealSense 原生话题适配为 Autoware camera0 接口。
- `livox_sensor_kit_description/config/sensor_kit_calibration.yaml`
  - 保存 `sensor_kit_base_link -> camera0_link` 和 Mid-360 外参。
- `livox_sensor_kit_description/urdf/sensor_kit.xacro`
  - 从上述 YAML 读取外参并发布静态 TF 链。

注意：外参是当前车辆的标定结果，移植到另一辆车时必须重新标定，不能原样照搬。

### 5. 扩展链路

- `camera_lidar_irregular_object_detector.launch.py` 和 `irregular_object_detector.param.yaml`
  - 低矮石墩等 irregular object 的融合支路。
- `tracking.launch.xml` 和 `multi_object_tracker_node.param.yaml`
  - 将有效的 camera–LiDAR 输出接入 tracker，并允许发布 tentative objects。
- `voxel_grid_based_euclidean_cluster.param.yaml`
  - 走廊环境的聚类容差、体素大小、点数和 Z 范围调优。
- `files/d455_perception/`
  - 自定义三分类检测和 Autoware ROI 合并实现；该目录来自外部 D455 工作区，没有可用的 Git 基线，因此以完整源码快照提供。

## Git diff 说明

`diff/autoware_launch_tracked_changes.diff` 只包含本融合方案相关的已跟踪文件，基线是：

```text
/home/tang/autoware/src/launcher/autoware_launch 当前 HEAD
```

以下文件在该仓库中当前是未跟踪的新文件，因此分别以 `/dev/null -> 文件` 的新增文件 diff 提供：

```text
diff/new_camera.launch.xml.diff
diff/new_livox_sensing.launch.xml.diff
diff/new_sensor_kit_calibration.yaml.diff
diff/new_sensor_kit.xacro.diff
```

当前工作区存在批量文件权限位变化，所以 tracked diff 中可能看到 `100644 -> 100755`。评审融合逻辑时以文本内容变化为主，不应把权限位变化当作功能配置。

查看差异：

```bash
less diff/autoware_launch_tracked_changes.diff
```

不要在未知版本上直接执行 `git apply`。建议先核对 Autoware 版本和目录结构，再逐文件移植；尤其应重新确认话题名、相机数量、时间偏移和外参。

## 当前推荐启动参数

```bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=$HOME/autoware_map/map_task4 \
  vehicle_model:=yunle_vehicle \
  sensor_model:=livox_sensor_kit \
  perception_mode:=camera_lidar_fusion \
  launch_camera:=false \
  use_traffic_light_recognition:=false \
  use_custom_d455_yolo:=true \
  custom_d455_yolo_device:=cuda:0 \
  custom_d455_yolo_imgsz:=640 \
  custom_d455_yolo_confidence:=0.4
```

`launch_camera:=true` 表示由 Autoware 启动并占用 D455；如果相机由独立视觉进程启动，则保持 `false`。

## 主要运行时接口

```text
图像       /sensing/camera/camera0/image_rect_color
CameraInfo /sensing/camera/camera0/camera_info
融合 ROI   /perception/object_recognition/detection/camera_lidar_fusion/rois0
融合目标   /perception/object_recognition/detection/clustering/camera_lidar_fusion/objects
调试图像   /perception/object_recognition/detection/clustering/camera_lidar_fusion/roi_cluster_fusion/debug/image_raw0
```

更完整的调试记录、问题根因和验证命令见：

```text
reference/2026-11-22-autoware-d455-fusion-corridor-handoff.md
```
