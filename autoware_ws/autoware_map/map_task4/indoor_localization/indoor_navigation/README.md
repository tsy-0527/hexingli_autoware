# indoor_navigation：室内规划与控制（ROS 2 Humble + Nav2）

适配云乐阿克曼小车：轴距 0.475 m、最大转角 21°（规划用最小转弯半径 1.3 m）、车长 0.852 m、车宽 0.572 m，base_link 在后轴中心正下方的地面上。

定位依赖 `indoor_localization` 包（提供 TF `map → base_link`、`/localization/kinematic_state`、预处理点云和定位健康状态）。

`route_overview.png` 画出了路网节点、速度分区，以及在真实地图上规划出的去程和回程路径。

## 一、整体结构

```
UI / Agent ──JSON──► route_navigator ──NavigateThroughPoses──► bt_navigator（行为树，无原地旋转）
  (rosbridge)          站点路网 → 途经点                          ├─► planner_server：Smac Hybrid-A*（Reeds-Shepp，可倒车）
                                                                  └─► controller_server：MPPI（阿克曼）或 RPP
                                                                              │ /cmd_vel_nav（behavior_server 的后退也走这里）
                                                                              ▼
                                                           collision_monitor（按当前指令预测车身碰撞）
                                                                              │ /cmd_vel
                                                                              ▼
                     定位健康 speed_factor ──► ackermann_cmd_converter ──► hybrid_mode_manager ──► chassis_driver
                                               (曲率换算、转向速率、换向停稳、加减速)   (室内/室外停车仲裁)
```

## 二、文件说明

| 文件 | 内容 |
|---|---|
| `config/nav2_indoor.param.yaml` | Nav2 全部参数：Smac Hybrid-A*、MPPI、代价地图、速度过滤器、Collision Monitor、行为服务器 |
| `config/controller_rpp.param.yaml` | 备用控制器 RPP，`controller:=rpp` 时加载 |
| `config/indoor_navigation_nodes.param.yaml` | 本包两个节点的参数，**底盘接口在这里改** |
| `config/stations.yaml` | 站点路网：节点、连线、站点（终点位置 + 车头朝向） |
| `config/speed_zones.yaml` | 速度分区（门口 0.2 m/s、房间 0.3 m/s，其余 0.5 m/s） |
| `behavior_trees/*.xml` | 去掉原地旋转的行为树；恢复动作：清代价地图 → 等待 → 后退 0.2 m → 再等待 |
| `maps/indoor_nav_2d.*` | 导航地图（与定位用的地图同一坐标系） |
| `maps/speed_mask.*` | 速度分区掩码，由 `tools/make_speed_mask.py` 生成 |

## 三、本包的两个节点

**`ackermann_cmd_converter`**：把 Nav2 的 (v, ω) 转换成阿克曼指令（车速 + 前轮转角）
- 转角 δ = atan(L·ω / v)，限幅 ±21°；车速接近 0 时保持上一次的曲率
- **转向速率限制**：车轮转到位需要时间；转角没跟上时自动降速，静止时先把车轮打到位再起步
- **换向必须先停稳**：前进和倒车切换前，要完全停住 0.5 s
- 加减速限制（只限车速，转角已定，所以曲率不变，车不会偏离规划的弧线）
- 定位健康 WARN 时车速减半，ERROR 时停车；0.5 s 收不到指令就刹停

**`route_navigator`**：给 UI 和 Agent 用的站点级接口（通过 rosbridge 访问，不需要自定义消息）
- 下发：话题 `/route_navigator/command`（std_msgs/String，JSON）
  - `{"cmd": "goto", "station": "hall_station_1"}`
  - `{"cmd": "cancel"}`
- 一键取消：服务 `/route_navigator/cancel`（std_srvs/Trigger）
- 状态：话题 `/route_navigator/status`（JSON）。state 取值：IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELED / REJECTED，还包括剩余距离、恢复次数、拒绝原因
- 定位健康为 ERROR 时拒绝出发

## 四、需要你们修改的地方（代码中搜索 TODO）

| 标记 | 位置 | 内容 |
|---|---|---|
| **TODO(chassis interface)** | `indoor_navigation_nodes.param.yaml` 的 `output_type` / `output_topic`；`ackermann_cmd_converter.py` 顶部说明 | chassis_driver 订阅的话题和消息类型。已支持三种：`autoware_control`（autoware_control_msgs/Control，旧版自动退回 AckermannControlCommand）、`ackermann_drive`、`debug_only`。CAN 0x121 的映射保留在 chassis_driver 里 |
| **TODO(chassis)** | `max_steer_rate` | 实测转向 0→21° 的时间，换算成速率 |
| **TODO(sensor)** | `nav2_indoor.param.yaml` 里的 D455 话题、`min_obstacle_height` | D455 点云话题；如果地面被当成障碍物，调高最小高度 |
| **TODO(sensor)** | collision_monitor | 车尾没有低矮障碍物检测，建议加超声波后作为新的数据源 |
| **TODO(site)** | `stations.yaml` | 大厅站点的实际位置和车头朝向 |

**另外必须改**：`indoor_localization` 里 `mid360_preprocessor` 的 **`obstacle_self_box`**，改成车身外扩约 3 cm，并包含雷达支架。它只用来去掉打到车身上的点，NDT 用的 `self_box` 可以保持宽松。

室内外融合时不要让本节点直接占用底盘最终话题。当前配置已将输出改为
`/hybrid/indoor/control_cmd` 和 `/hybrid/indoor/gear_cmd`，由
`hybrid_mode_manager` 停车仲裁后发布最终命令。完整启动方法见
`../HYBRID_NAVIGATION.md`。

## 感知：3D 体素栅格

没有聚类，障碍物直接从点云进入栅格。

```
MID-360 ─► mid360_preprocessor ─► 障碍物点云（离地 ≥ 5 cm，紧贴车身去掉自身点，5 cm 降采样）─┐
       └─► 原始点云（含地面，只用于清除）───────────────────────────────────────────────────┤
D455 深度点云 ─► 标记（离地 ≥ 5 cm）+ 清除（含地面）───────────────────────────────────────┤
                                                                                             ▼
                          3D 体素层：0~0.6 m 高，每层 5 cm（12 层）→ 投影成 2D 代价地图
                          （局部代价地图给 MPPI，全局代价地图给重规划）
障碍物点云 + D455 ─► Collision Monitor（不经过栅格，直接按车身预测碰撞）
```

- **标记**：只在 0~0.6 m 的体素格内标记（车高 0.327 m），更高的东西（桌面、门框上沿）不挡车。
- **清除**：从传感器到每个点做射线，射线穿过的体素清空。包括打到墙、天花板（射线在 0.6 m 处截断）和地面的点，所以人走开后不会留下残影。
- 配置位置：`config/nav2_indoor.param.yaml` 里 `local_costmap` / `global_costmap` 的 `obstacle_layer`，以及 `collision_monitor` 的 `observation_sources`。

**低矮障碍物（< 15 cm）的覆盖范围**

MID-360 装在离地约 0.51 m，向下视场只有 7°，所以近处的低矮物体它看不到：

| 物体高度 | MID-360 能看到的最近距离 |
|---|---|
| 5 cm | 约 3.7 m |
| 10 cm | 约 3.3 m |
| 15 cm | 约 2.9 m |
| 30 cm | 约 1.7 m |

- **车前近处**（约 0.5~2.5 m）：靠 D455。D455 本身离镜头约 0.5 m 以内看不到。
- **车后**：没有传感器能看到低矮物体。体素层有"记忆"：车前进时 D455 看到的低矮障碍物，离开视野后**不会被清掉**（没有射线经过那里），所以倒车经过刚走过的区域时，代价地图里还保留着它们。真正要可靠，还是建议加车尾超声波。
- 这种记忆也可能留下"已经被拿走的东西"。行为树的恢复动作会清空代价地图，所以只会造成短暂停车，不会一直卡住。

**D455 外参检查（必须做）**：车停在平地上，RViz 里看 D455 点云中的地面点，离地高度应该在 ±2 cm 以内。D455 的俯仰外参差 1°，3 m 处地面就会偏 5 cm，要么把地面当成障碍物，要么漏掉低矮物体。

**RViz 里检查**：`/local_costmap/voxel_marked_cloud`（被标记的体素）、`/local_costmap/costmap`（投影后的 2D 栅格）、`/indoor_perception/mid360/obstacle_pointcloud`（障碍物点云）。

## 五、启动

```bash
colcon build --packages-select indoor_navigation
# 先启动 indoor_localization，并完成初始化
ros2 launch indoor_navigation indoor_navigation.launch.py controller:=rpp    # 第一次：用 RPP 跑通链路
ros2 launch indoor_navigation indoor_navigation.launch.py                    # 之后：默认 MPPI
```

launch 参数：`controller`（mppi / rpp）、`nav_map_yaml`、`speed_mask_yaml`、`stations_file`，后三个不填就用包里自带的文件。

修改了 `speed_zones.yaml` 后，重新生成掩码：
```bash
python3 tools/make_speed_mask.py maps/indoor_nav_2d.yaml config/speed_zones.yaml maps
```

## 六、调试顺序

| 步骤 | 内容 | 通过标准 |
|---|---|---|
| 0 | **底盘特性实测**（车架空或低速）：转向 0→21° 的时间；0.1/0.2/0.3 m/s 能否稳住；倒车方式 | 填好 `max_steer_rate`；确认最低稳定车速 |
| 1 | `output_type: debug_only`，在 RViz 里用 2D Goal Pose 发一个近处目标，看 `/ackermann_cmd_converter/debug` | 转角、车速的符号和大小合理，换向时先停再走 |
| 2 | 接上底盘，**RPP**，大厅里短距离前进、倒车 | 能跟上路径，一键停车有效 |
| 3 | 感知与 Collision Monitor：人站在车前、车后；地上放一个 10 cm 高的盒子 | 代价地图出现对应障碍物，人走开后几秒内清除；静止时地面不被标成障碍物；车前有人或盒子会减速、停车，倒车时车后有人同样 |
| 4 | 换 **MPPI**，大厅 → 走廊直线 | 走廊里横向偏差小于 5 cm |
| 5 | **门口专项**：进门、出门各 20 次 | 记录车身离门框的最小距离，没有刮碰 |
| 6 | 整条路线：`goto hall_station_1`，再 `goto room_home` | 状态为 SUCCEEDED，终点误差 < 0.1 m、朝向误差 < 6° |

**MPPI 调参提示**
- 走廊里左右摆动：增大 `PathFollowCritic.cost_weight`，或减小 `wz_std`
- 门口跟不紧：先降速；再增大 `PathAlignCritic.cost_weight`。如果实测转向很慢，再考虑别的方案
- 终点前反复微调：适当放宽 `goal_checker` 的容差

## 七、验证情况

- 转换节点核心逻辑的单元测试全部通过（`test/test_nav_core.py`）：曲率换算、转角限幅、静止时先打方向（以 30°/s 为例约 0.5 s）、换向前停稳 0.6 s、命令超时刹停、健康状态降速、路网路由。
- 两个节点在模拟的 ROS 环境里跑过：输出 Autoware Control 消息、定位 ERROR 时拒绝出发、导航状态流转。
- 体素层的参数名和射线截断行为对照 Humble 源码核对过；预处理节点的障碍物点云用模拟点云测过：10 cm 高的物体保留，地面和车身被去掉，而 NDT 点云不受影响。
- Nav2 参数名、插件名、行为树节点名，对照 Humble 分支源码核对过；速度掩码的编码按 map_server 的解码方式做了往返校验。
- 用一个简化的 Hybrid-A*（同样的车身尺寸和 1.3 m 转弯半径）在真实地图上验证了整条路线：去程 45.6 m 无需倒车（在大厅里绕弯掉头），回程 42.6 m（进房间后倒车 1.2 m，车头朝门停好）。实际路径由 Nav2 Smac 规划，形状会更平滑。
- **还没在真实的 Nav2 / 车辆上运行过。**
