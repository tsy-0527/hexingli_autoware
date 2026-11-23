# 室内栅格 / 室外车道融合启动说明

## 架构

- 全程只有一套 Autoware、一个 `map` 坐标系和一条定位输出（MID-360 NDT + `gyro_odom` + EKF）。
  室内外**不切换定位**，只切换"谁控制底盘"。
- 室外规划/控制：Lanelet2 + Autoware。
- 室内规划/控制：二维栅格 + Nav2 → collision_monitor → 阿克曼转换。
- 两个控制器只写私有话题 `/hybrid/outdoor/*`、`/hybrid/indoor/*`。只有 `hybrid_mode_manager`
  发布底盘实际订阅的 `/control/command/control_cmd` 与 `/control/command/gear_cmd`。
- 室内栅格只覆盖室内：栅格外是"未知"，Nav2 `allow_unknown: false`，并且有禁行掩膜（KeepoutFilter）兜底。

## 交权规则（全部满足才交权）

| # | 条件 | 不满足时 |
|---|---|---|
| 1 | 一发起切换就输出停车（v=0，a=−0.5） | — |
| 2 | 车速数据新鲜（底盘 VelocityReport 优先，其次定位速度），且持续 < 0.03 m/s 达 1 s | 继续停车，重新计时；车速断流视为"可能在动" |
| 3 | 目标控制器命令新鲜（节点活着） | 继续停车 |
| 4 | **目标控制器此刻输出的是停车**（\|v\| ≤ 0.05） | 继续停车。所以必须**先交权、再下任务** |
| 5 | 定位健康（level 不是 ERROR，健康数据 1 s 内有更新）。**两个方向都要求** | 继续停车 |
| 6 | 配了围栏时，手动请求必须在门口交接带（进室内：在 exit 围栏内；出室外：在 enter 围栏外） | 服务直接返回 `success: false` |

`/hybrid_mode_manager/state` 里的 `block_reason` 会说明当前卡在哪一条。

另外 `route_navigator` 只有在 `mode=indoor, phase=active` 时才接受站点任务；室内权限被收回
（开始切换、或状态断流）时会自动取消正在执行的 Nav2 任务。

## 编译与启动

```bash
cd /home/tang/autoware_map/map_task4/indoor_localization
source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash
colcon build --packages-select indoor_localization indoor_navigation room_global_localizer --symlink-install

./run_hybrid_navigation.sh
```

默认以室外模式启动，GNSS 自动初始化默认关闭（原因见文末），启动后在 RViz 用 2D Pose Estimate
给初值，或在室内用房间全局初始化。

## 交接流程（停车式手动交接）

**室外 → 室内**

```bash
# 1. Autoware 把车开到门口交接点 (29.000, -76.049) 并停下（车道终点 = 交接点）
# 2. 请求室内权限
ros2 service call /hybrid_mode_manager/set_indoor std_srvs/srv/SetBool "{data: true}"
# 3. 等到 mode=indoor, phase=active
ros2 topic echo /hybrid_mode_manager/state
# 4. 再下发室内站点任务
ros2 topic pub --once /route_navigator/command std_msgs/String '{data: "{\"cmd\": \"goto\", \"station\": \"room_home\"}"}'
```

**室内 → 室外**

```bash
# 1. 室内站点任务把车开回门口交接点并停下（站点 south_door_out，经坡道、掉头后车头朝东）
# 2. 请求室外权限（此时 Autoware 必须处于停车状态，不要提前设路线）
ros2 service call /hybrid_mode_manager/set_indoor std_srvs/srv/SetBool "{data: false}"
# 3. 等到 mode=outdoor, phase=active
# 4. 再在 Autoware 设置室外路线并 engage
```

> 和原文档相反：**不要**在交权之前给 Nav2 下目标。原来的写法会让 Nav2 在没有底盘权限时空转，
> 进度检查失败后触发倒车等恢复行为；交权瞬间车会被一条已经"着急"的路径直接带走。

## 自动切换（待录入实测匹配点）

把实测点录入 `config/hybrid_switch_zones.yaml`（统一 `map` 坐标，不能直接填经纬度）：

- `indoor_enter_polygon`：室内区域（较小）；
- `indoor_exit_polygon`：包住它的较大区域。两者之间的带状区域就是门口交接带，交接点要放在这条带里。

配好之后手动请求也会做位置检查。在 RViz 核对边界后再启动：

```bash
./run_hybrid_navigation.sh automatic_switching:=true
```

> 现在的 `hybrid_switch_zones.yaml` 已按南门录好，但**自动切换不适合这个布局**：Autoware 不会开进建筑轮廓，
> Nav2 也被禁行掩膜限制在门前区域，围栏触发条件不会满足。保持 `automatic_switching: false`，
> 由任务层在"Autoware 到达交接点"之后调用 `set_indoor`；围栏只用于手动请求的位置检查。

围栏只负责发起请求，仍然要满足上面全部交权条件。注意自动进门后只是拿到权限，
还需要任务层下发室内站点任务。

## 室内栅格：map_20261117_legacy_indoor

`grid_map_path` 默认是 `map_20261117_legacy_indoor`：旧室内地图的栅格按
`legacy_room_registration.yaml`（yaw 3.422°，平移 11.185, −18.210；墙体 ICP 中位误差 0.05 m）变换到统一坐标系，
再拼上从 20261117 点云生成的门前区域（见该目录 README.txt）。

> **更正**：之前的配准（yaw −0.111°，平移 −11.169, 55.407）是错的，把室内图放到了北侧停车场上；
> 当时"20261117 点云没录到这栋楼、走廊被树冠堵死"的说法也是错的。这栋楼就是走廊在 x≈29、y −58 ~ −19
> 的那栋，点云里有它。`stations.yaml`、房间全局初始化的 `room_polygon` 都已按新配准换算。

重新生成：`bash indoor_navigation/tools/rebuild_maps.sh`（在仓库根目录）。

## 南门：交接点、坎和坡道

```
                 door_inside (26.9, -68.8)  ← 门内
      ─────────── 立面 y≈-70.8，门洞 x≈26.9，宽约 1.2 m ───────────
         平台                                        平台
  ┌栏杆──────────┐ ↗
  │  坡道 →→→→  │      ← 路沿坎 0.15–0.22 m（x 19–42），不能直接上
  └栏杆──────────┘
   ↑ 坡道口 ramp_approach (18.2, -76.3)       south_door_out / 交接点 (29.0, -76.049)
   └──── 主路上掉头 ←←←←←←←←←←←←←←←←←←←←←←←←← Autoware 西行到此停车
```

- **坎**：门前 x 19–42 整排路沿，比路面高 0.15–0.22 m，东头在 x≈41.8 拐向北接到楼边。
- **坡道**：在门的西边，x 17.7–24、y −75.4 ~ −74.0，两侧栏杆，净宽约 1.2 m（车宽加余量 0.63 m）。
  西头和路面齐平，往东升高约 0.14 m 后接上门前平台（坡度约 3%）。车只能从西南角约 1.5 m 宽的坡道口拐进去。
- 栅格里坎、栏杆、坡道西侧平台边、主路南侧路沿都画成了障碍，所以 Nav2 只能走坡道。
- 路线节点（`stations.yaml`）：`south_door_out` → `ramp_approach`（坡道口，南北向）→ `door_ramp`（坡道中线，东西向）→ `door_inside`。
  进门在坡道口朝北、坡道上朝东上坡；出门反过来。只放一个坡道点不够：按"前一点 → 后一点"判断方向时，
  从交接点出发会让车朝西经过坡道。
- 离线 Hybrid-A*（R_min 1.3 m、车身轮廓 + 0.03 m）校验：进门约 25 m、出门约 25 m，全程前进、不倒车，
  掉头在交接点西侧主路上完成，车身始终在 exit 围栏内。坡道和掉头区限速 0.2 m/s。
- 交接点和 Autoware 路线终点都没变：仍是 (29.000, −76.049)，进门车头朝西，出门车头朝东。

**禁行掩膜为什么必须有**：`allow_unknown: false` 只挡静态图里的未知格。全局代价地图的 VoxelLayer
开着 clearing，`combination_method: 1` 取最大值时，雷达射线扫过的未知格会被改写成空闲，门外几米到
十米的室外区域又能规划了。KeepoutFilter 在所有图层之后把掩膜外的格子强制设为禁行。

## 上车前检查

```bash
# 最终底盘命令必须只有 hybrid_mode_manager 一个发布者
ros2 topic info /control/command/control_cmd --verbose
# 两个源都应存在
ros2 topic hz /hybrid/outdoor/control_cmd
ros2 topic hz /hybrid/indoor/control_cmd
# 交权用的车速和健康
ros2 topic hz /vehicle/status/velocity_status
ros2 topic echo /indoor_localization/health --once
# 禁行掩膜生效
ros2 topic echo /keepout_costmap_filter_info --once
```

先架空车轮或断开驱动电机验证：切换阶段始终先输出零速；室外时下发室内站点任务会被拒绝；
交还室外时 Autoware 若在输出速度，状态会一直停在 stopping 并给出原因。

## 仍需你确认的问题（代码改不了）

1. **NDT 地图。** `map_path` 默认 `/home/tang/autoware_map/map_task4`，请确认其中的 `pointcloud_map.pcd`
   就是 20261117 点云（或其清理版）。门前那个录制时站着的人只从规划栅格里删了，点云里还在，对 NDT 影响很小。
2. **车道图。** 用 `outdoor_lanelet/lanelet2_map.osm`（修正版）替换 `map_path` 下的车道图，修正内容和剩余的路网问题
   见 `outdoor_lanelet/README.md`。
3. **门前尺寸现场核对。** 坡道口宽度（约 1.5 m）、栏杆间净宽（约 1.2 m）、门洞宽度（约 1.2 m）都是从点云量的，
   请用卷尺核对；如有出入，改 `indoor_navigation/tools/rebuild_maps.sh` 里的折线和矩形后重新生成。
4. **门口一带是 20261117 建图的漂移区**（开头和结尾两次经过，两层相差约 1–1.3 m），门前 NDT 可能不稳，
   交接前先看 `/indoor_localization/health`。
5. **GNSS 自动初始化。** 用 `gnss_init:=true` 才打开，默认关闭：地图 `projector_type: Local`，
   还没有和经纬度配准，GNSS 位姿不能直接当这张图里的初值。
6. **站点航向。** `hall_station_1`、`hall_station_2` 的位置和航向仍是 TODO。
