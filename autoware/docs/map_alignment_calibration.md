# map_task1 地图坐标对齐实验

本文说明如何用一次同步录制判断 `map_task1` 的偏差来自坐标变换、参考点杆臂，还是 FAST-LIO/GNSS 的随程误差。脚本为 `scripts/record_map_alignment.sh`。

## 现有地图文件已确认的内容

- `map_projector_info.yaml` 的原点为 `31.9339324°N, 118.6332468°E`，属于 UTM 50N，中央经线为 `117°E`。
- `lanelet2_map.osm` 每个节点的 `local_x/local_y` 与用上述原点投影节点经纬度所得的 UTM 局部坐标吻合到约 `0.04 mm`。所以车道地图与投影配置本身一致。
- 原点处网格收敛角约为 `+0.864°`，点尺度约为 `0.999894`。若真东被直接当成 UTM 网格东，直行 300 m 会出现约 `4.52 m` 的横向差；点尺度造成的距离差约为 `0.03 m`。因此，2 m 误差更像航向/参考系问题，不能由此处的 UTM 尺度解释。
- `pointcloud_map.pcd` 有约 197 万点，XYZ 是局部米制量级；PCD 头没有记录 CRS、UTM 分区或地理原点。它与车道线的平面范围大体重合，但单凭 PCD 文件不能证明其绝对地理原点和朝向。

## 录制前准备

优先录制**生成当前 PCD 的同一次建图运行**。FAST-LIO 每次初始化都会建立自己的局部坐标系；另一趟新运行的局部坐标不能直接套用到旧 PCD。若原始建图 rosbag 还在，最好从该 rosbag 重放并同步录制 FAST-LIO 里程计和 GNSS/INS。

脚本已经将你们当前 FAST-LIO 的 `/Odometry` 设为默认里程计话题；该话题由 `laser_mapping` 发布，类型为 `nav_msgs/msg/Odometry`。因此，确认相关节点已经启动后可以直接运行：

```bash
ros2 topic echo /Odometry --once
```

确认输出持续发布且 `header.frame_id`、`child_frame_id` 正确即可。不要用只在录制结束时发布的单帧地图点云代替轨迹。如果以后 FAST-LIO 话题名改变，可用下面的命令查询：

```bash
ros2 topic list -t | grep -E 'nav_msgs/msg/Odometry|geometry_msgs/msg/PoseStamped'
```

脚本默认 GNSS 话题为 `/sensing/gnss/fix`，INS 姿态话题为 `/sensing/gins2000`，并会自动附带 `/tf`、`/tf_static` 及当前存在的 Autoware GNSS/定位话题。话题名称不同可通过参数覆盖：

```bash
./scripts/record_map_alignment.sh \
  --output-dir "$HOME/autoware_bag" \
  --name map_task1_alignment
```

也就是说，点云建图运行期间只需额外启动这个脚本即可。FAST-LIO 继续按原配置保存 `pointcloud_map.pcd`；脚本保存与这次建图同步的 GNSS/INS、`/Odometry` 和 TF。两者应使用同一次 FAST-LIO 运行，录制结束后把 rosbag 与 PCD 放在一起分析。

也可以这样指定不同的 GNSS 或 INS 话题：

```bash
./scripts/record_map_alignment.sh \
  --fastlio-odom /实际的/fastlio/odometry话题 \
  --gnss-fix /实际的/gnss/fix话题 \
  --ins /实际的/ins/姿态话题
```

录制建议：

1. 车辆停在容易重复找到的位置，等组合导航进入稳定 RTK Fixed 状态；静止记录约 30 秒。
2. 行驶一段 100–300 m 的直线，再掉头沿相同道路返回起点。路线中包含转弯更利于检查杆臂和时间偏差。
3. 保持 GNSS、INS、FAST-LIO 使用同一 ROS 时间源。结束后回到同一物理点并再静止记录一会儿，然后按 Ctrl+C 停止。
4. 尽量选择天空视野较好的路段。若 GNSS 状态不稳定或处于 RTK Float，米级偏差不能用来标定厘米级杆臂或亚度级航向。

## 数据配对和拟合

从 rosbag 中读取 FAST-LIO odometry 和 GNSS/INS。按消息 `header.stamp` 时间对齐，并在相邻 GNSS 采样间插值；不要用 rosbag 收包顺序直接逐条匹配。检查 odometry 的 `header.frame_id`、`child_frame_id` 和 TF，确定位置对应 FAST-LIO 的 IMU、雷达还是车体参考点。

把每个 GNSS 经纬度用 `map_task1/map_projector_info.yaml` 的同一个投影原点换成局部 UTM 坐标 (g_i=(E_i,N_i))。把 FAST-LIO 轨迹中的同一参考点记为 (p_i=(x_i,y_i))。若 GNSS 使用的是天线位置，应通过准确的静态 TF 和同一时刻姿态换算到 FAST-LIO 对应的参考点；不要把“天线到雷达约 20 cm”直接当成固定地图 X 平移。

先只拟合二维刚体变换：

```text
g_i ≈ R(θ) p_i + t
```

对所有有效配对点用最小二乘或带异常值剔除的鲁棒拟合，得到旋转 `θ` 和平移 `t`。不要先用自由尺度拟合去掩盖里程漂移。尺度可单独作为诊断项计算；该 UTM 原点附近预期尺度约为 `0.999894`，300 m 上只对应约 3 cm。

## 如何判断误差来源

- **拟合前几乎是固定偏移，拟合平移后各处都变好**：检查 map origin 与 FAST-LIO 原点是否是同一个物理点，以及天线到 IMU/雷达的杆臂。20 cm 杆臂主要造成近似固定的起点偏移，不应单独产生随距离累积的 2 m。
- **横向误差随距离近似线性增长，拟合角度后显著变好**：检查 FAST-LIO 起始航向、雷达/IMU 安装角和真北/网格北换算。2 m 横向差除以 300 m，对应约 `0.382°`。如果拟合角接近 `0.864°`（注意 INS 与 ROS yaw 的符号约定），强烈支持收敛角没有正确处理。
- **直线段能对齐，但转弯时残差随车辆朝向改变**：检查天线杆臂是否按车身姿态旋转，以及 GNSS、INS、雷达时间戳是否有延迟。
- **刚体变换后残差仍随路程增长，或回到起点时 FAST-LIO 轨迹没有闭合**：更像 FAST-LIO 累积漂移、GNSS 异常或同步问题。固定的地图旋转和平移无法修复这种非刚体误差。
- **误差方向随机跳动，并与 GNSS 状态/协方差变差同时发生**：优先检查组合导航质量和遮挡多路径。

## 需要保留的结果

分析时保留拟合出的 `θ`、`t`、可选诊断尺度、每个配对点的二维残差，以及残差随累计行驶距离的曲线；另外单独比较出发点与返回点的 FAST-LIO 局部坐标差。这样可以区分一次性坐标变换误差和随路程变化的定位漂移。
