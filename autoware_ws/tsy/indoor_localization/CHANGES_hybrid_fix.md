# 融合导航逻辑修正清单

解压到 /home/tang/autoware_map/map_task4/indoor_localization 覆盖同名文件即可（只包含改动和新增的文件），
然后重新 colcon build。代码改动的 diff 见 hybrid_fix.patch。
`outdoor_lanelet/lanelet2_map.osm` 需要另外拷到 `map_path` 下替换车道图（见 outdoor_lanelet/README.md）。

## 逻辑错误（已修正，离线仿真复现过原版的问题）

| 问题 | 后果 | 修正位置 |
|---|---|---|
| 室外时 route_navigator 也接受室内站点任务 | Nav2 没有底盘权限却在规划控制，进度检查失败后触发倒车等恢复行为；交权瞬间车被直接带走 | route_navigator.py：只有室内拿到权限才接受任务，权限收回时取消任务 |
| 交权时不检查目标控制器是否在输出速度 | 交还室外时 Autoware 挂着旧路线在输出 0.8 m/s，交权瞬间底盘直接收到 0.8 m/s | hybrid_core.py：目标控制器必须在输出停车（先交权、再下任务） |
| 车速取自定位，且没有超时 | 车速数据断流后一直用最后一个值（0），车可能在动也交权 | hybrid_mode_manager.py：底盘 VelocityReport 优先，断流视为"可能在动" |
| 定位健康没有超时 | 健康数据停发后一直认为 OK | hybrid_mode_manager.py：1 s 超时 |
| 只在进室内时检查定位健康 | NDT 是全程唯一定位，出室外交权时定位坏了也照交 | 两个方向都检查 |
| 手动请求在任何位置都执行 | 在路中间切换，车停在不能继续的地方 | 配了围栏时必须在门口交接带，否则服务返回 success=false |
| GNSS 初始化参数没传到定位栈；组件里 GNSS 话题写死 | hybrid launch 里写的 gnss_enabled=true 实际不生效 | 两个 launch 补传参；默认用 gnss_init:=false 显式关闭（原因见 HYBRID_NAVIGATION.md） |
| 室内栅格默认用整个校园的 denoised 栅格 | Nav2 在室外也能规划 | 默认改为 map_20261117_legacy_indoor |
| allow_unknown: false 被全局代价地图的雷达清除绕过 | 门外被雷达扫过的未知格变成空闲，可以规划出去 | 新增 KeepoutFilter 禁行掩膜 |

## 地图与配准（已处理）

| 问题 | 处理 |
|---|---|
| 旧室内图配准错误（yaw −0.111°，平移 −11.169, 55.407），室内被放到北侧停车场上；stations.yaml 和房间多边形都跟着错 | 墙体 2D ICP 重新配准：yaw 3.422°，平移 (11.185, −18.210)，中位误差 0.05 m。站点、房间多边形全部重新换算 |
| 室内栅格没有门前区域，交接点到门之间无法规划 | 从 20261117 点云生成门前区域（door_apron），并入 nav_map；门洞强制可通行 |
| 门前 0.15–0.22 m 路沿坎在栅格里是空的，Nav2 会直接翻坎进门 | 坎、坡道栏杆、平台边、南侧路沿画成障碍；坡道通道强制可通行；新增路线节点 ramp_approach、door_ramp，经坡道进出门 |
| 录制时站在主路上的人在栅格里是障碍 | 从 door_apron 清掉 |
| 室外车道图：14 条车道左右边界反了、6 条整段车道与分段车道重叠、最南侧车道缺一段 | outdoor_lanelet/fix_lanelet.py 修正（不移动任何点） |
| 没有室内外切换围栏和交接位姿 | hybrid_switch_zones.yaml 按南门生成；交接点 (29.000, −76.049) |

## 新增文件

- indoor_navigation/tools/register_legacy_grids.py   旧栅格 → 统一坐标系，合并门前区域、强制可通行、障碍折线、限速区
- indoor_navigation/tools/make_keepout_mask.py       生成禁行掩膜
- indoor_navigation/tools/rebuild_maps.sh            一条命令重建 map_20261117_legacy_indoor
- map_20261117_legacy_indoor/                        室内 + 门前栅格、限速、禁行掩膜、配准参数
- outdoor_lanelet/                                   修正后的车道图、检查和修正脚本
- offline_sim/                                       不需要 ROS 的交接流程仿真（9 个场景）

## 验证

- indoor_navigation 单元测试 14 项通过（含经坡道进出门的路线断言），room_global_localizer 测试 3 项通过
- offline_sim 9 个场景（29 项检查）全部通过；场景 9 用真实围栏和站点：交接点进门、路线先到坡道口再上坡道
- 门前路线用离线 Hybrid-A*（R_min 1.3 m，车身 + 0.03 m）在新 nav_map 上校验：进门、出门都能走通，不倒车
- 车道图检查：边界反向 0、同向重叠 0
- 没有在实车和真实 ROS 环境里运行过，按 HYBRID_NAVIGATION.md 的"上车前检查"从架空车轮开始

## 仍需确认（详见 HYBRID_NAVIGATION.md 末尾）

1. map_task4 的 NDT 点云是不是 20261117 点云
2. 用修正后的车道图替换；路网剩余问题（北侧环路不连通、路口缺转向等）
3. 坡道口、栏杆间距、门洞宽度现场核对
4. 门口一带是建图漂移区，交接前看定位健康
5. GNSS 与地图的经纬度配准
6. hall_station_1 / hall_station_2 的位置和航向
