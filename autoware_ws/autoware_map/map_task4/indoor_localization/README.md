# MID-360S 室内自主初始化与实时定位

当前推荐链路是：房间全局粗定位 → Autoware NDT 6DoF 精配准 → EKF 实时定位。
统一入口只启动一套 NDT/EKF；Autoware 负责地图、传感器、车辆接口和实时定位，
`room_global_localizer` 负责自动给出并验证初始位姿。整条链路统一使用调平后的内部
`map` 坐标，NDT 地图为 `map_task5_leveled/pointcloud_map.pcd`，不再换回原始 task5 坐标。

## 推荐启动方式

```bash
cd /home/tang/autoware_map/map_task4/indoor_localization
./run_task5_combined_localization.sh
```

启动后 RViz 默认打开，并在 8 秒后自动开始初始化。运行视图显示调平后的静态点云地图
`/map/pointcloud_map`、当前位置和规划轨迹，不显示实时雷达点云、障碍物、候选位姿、
评分标记、TF 和历史定位轨迹。组合模式关闭额外的调试地图发布器，点云地图只由
Autoware map loader 发布一次。障碍物感知留给后续规划模块，当前默认
`launch_perception:=false`。若需手动重试，可调用：

```bash
ros2 service call /room_global_localizer/trigger std_srvs/srv/Trigger '{}'
```

初始化成功后，NDT 和 EKF 会持续发布实时位姿；健康状态位于
`/indoor_localization/health`。未来接入规划控制时可使用：

```bash
./run_task5_combined_localization.sh \
  launch_perception:=true launch_planning:=true launch_control:=true
```

独立（不使用 Autoware）初始化模式仍保留在 `LOCAL_SETUP.md`，但不应与上述组合入口同时启动。

## 目录

```
map/
  indoor_map_leveled_clean.pcd   调平 + 清理后的点云地图（x y z intensity，二进制）
  indoor_loc_2d.pgm / .yaml      全局初始化用 2D 层：离地 0.60–2.85 m 的点
  indoor_nav_2d.pgm / .yaml      Nav2 静态层：离地 0.08–2.00 m 的点
  T_old_to_new.txt               旧地图 → 新地图的 4x4 变换
  preview.png                    预览
room_global_localizer/           ROS 2 (ament_python) 初始化节点
  room_global_localizer/core.py  算法（不依赖 ROS，可离线测试）
  room_global_localizer/node.py  ROS 2 节点（订阅点云并发布初始位姿）
  config/…param.yaml             参数
  launch/…launch.py
  test/offline_sim_test.py       离线仿真测试：python3 test/offline_sim_test.py ../map 30
```

## 地图做了什么

1. **调平**：原图坐标系与重力方向偏 3.08°（绕 y 轴），已旋转修正。修正后地面拟合残余倾角 0.08°。
2. **z 平移 +0.51 m**：调平地图地面 z = 0，与 `base_link` 地面原点一致；初始化、实时定位和规划始终保留在该坐标系。
3. **裁剪**：建筑外的点（透过门窗/玻璃）、地面以下的镜面反射点、天花板以上的点。
4. **去动态残影**：建图时跟车行人留下的点团（约 2500 点）和孤立噪点（约 1700 点）。
5. 点数 202,702 → 173,440，地图坐标系是本地坐标系，还没和室外卫星坐标对齐。

**需要人工核对的地方**
- 左上区域有两处墙面扫描不完整，2D 图中边缘笔直的白色区域是按手工框定的范围填充的，不代表真实可通行。
- 房间内的散点（家具、残余噪点）都在 nav 层里。Nav2 用之前请用 GIMP 等工具在 pgm 上把明显的噪点涂成白色，并确认房间门口是通的。
- 当前 Nav2 地图已使用 `indoor_navigation/tools/filter_nav_map.py` 做垂直结构过滤：保留高度跨度至少 0.60 m 的墙体/门框，删除小于 6 格的孤立点，并清除室内封闭的 unknown 小岛。原始未过滤 PGM 以 `*_unfiltered.pgm` 保留。
- 旧站点坐标必须先用 `T_old_to_new.txt` 转换；当前 `indoor_navigation/config/stations.yaml` 已使用转换后的内部坐标。

## 自主初始化的流程

```
开机 → 车静止 1 s → 累积 1 s MID-360S 点云（转到 base_link）
     → 截取离地 0.60–2.85 m → 2D 投影
     → 房间范围内 360° 全局搜索（FFT 相关，0.1 m / 2°）→ Top-6 候选
     → 每个候选精修（0.025 m / 0.25°）+ 显式评估 180° 翻转位姿
     → 独立模式：验证 score ≥ min_score 且 次优/最优 ≤ max_ratio
     → Autoware 模式：Top-3 内部坐标候选逐个进行 6DoF NDT 验证
     → 主候选 NDT reliable：正式初始化并持续跟踪；否则重试或拒绝
     → 不通过：重试 1 次 → 仍失败：发布 FAILED，不发布错误位姿
```

## 接入步骤

1. **地图关系**：粗匹配、Autoware NDT、EKF、Nav2 和 RViz 都使用调平后的内部 `map` 坐标；不要再加载原始 `map_task5/pointcloud_map.pcd`。
2. **校正雷达外参（重要）**：当前 `base_link → 雷达` 为 `(0.60, 0.01, 0.57)` m，RPY 为 0。安装俯仰必须实测标定，不能直接使用约 3° 的估计值。
3. **改参数** `config/room_global_localizer.param.yaml`：
   - `loc_map_yaml` / `nav_map_yaml`：地图路径
   - `pointcloud_topic`：任意能通过 TF 转到 base_link 的点云
   - `self_box`：车身在 base_link 下的范围，用来去掉打到车身的点
   - `internal_to_output_transform`：生产配置必须保持单位矩阵
4. **编译运行**
   ```bash
   colcon build --packages-select room_global_localizer
   ros2 launch room_global_localizer room_global_localizer.launch.py param_file:=/path/to/param.yaml
   ```
5. **接口接入**
   - 触发：调用服务 `/room_global_localizer/trigger`（std_srvs/Trigger）
   - 状态：订阅 `/room_global_localizer/status`（std_msgs/String，JSON，transient local）
     - `state` 取值：IDLE / WAIT_STOP / ACCUMULATE / SEARCH / DONE / FAILED
     - `result`：内部匹配 x、y、yaw_deg、score、ratio，以及同一内部坐标中的 output_x/y/z
   - 位姿：订阅 `/room_global_localizer/pose`（PoseWithCovarianceStamped，transient local）

## 阈值标定（上线前必须做）

`min_score = 0.70`、`max_ratio = 0.85` 是仿真得到的初值。实车上：
1. 把车停在房间里 20 个以上不同位置和朝向（包括靠墙、角落、旁边站人、门开着）。
2. 每次调用 trigger，记录 status 中的 `score` 和 `ratio`，并人工确认位姿是否正确（可以在 RViz 里看点云是否和地图重合）。
3. `min_score` 设为正确结果最低分再往下留余量；`max_ratio` 设为正确结果最大 ratio 再往上留一点余量，但不要超过 0.9。
4. 原则：**宁可拒绝，也不能错误接受**。

## 离线仿真结果（本地图）

| 场景 | 位姿正确 | 被接受 | 错误接受 | 次优/最优（均值 / 最大） | 耗时 |
|---|---|---|---|---|---|
| 房间内，正常 | 30/30 | 30/30 | 0 | 0.61 / 0.69 | ~0.34 s |
| 房间内，60° 遮挡 + 丢一半点 | 30/30 | 30/30 | 0 | 0.64 / 0.75 | ~0.31 s |
| 房间内，家具全部挪走 | 25/25 | 25/25 | 0 | 0.78 / 0.82 | — |
| 车实际不在房间（走廊/大厅） | — | 0/30 | 0 | 0.95 / 1.00 | ~1.1 s |

注：仿真是理想的 2D 光线投射，实车的得分余量会更小，所以第 4 步的阈值标定不能省。
