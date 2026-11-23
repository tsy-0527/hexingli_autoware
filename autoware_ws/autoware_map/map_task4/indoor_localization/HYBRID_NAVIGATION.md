# 室内栅格 / 室外车道融合启动说明

## 架构

- 全程只有一套 Autoware、一个 `map` 坐标系和一条定位输出。
- 定位使用 MID-360 NDT + `gyro_odom` + EKF；室外由 GINS2000/gnss_poser 生成
  `/sensing/gnss/gnss_poser/pose_with_covariance`，用于自动初始定位。
- 室外规划/控制使用 Lanelet2 + Autoware。
- 室内规划使用二维栅格 + Nav2，输出经过碰撞监测和阿克曼转换。
- 两个控制器只写 `/hybrid/outdoor/*` 和 `/hybrid/indoor/*`。只有
  `hybrid_mode_manager` 可以发布底盘实际订阅的
  `/control/command/control_cmd` 与 `/control/command/gear_cmd`。
- 每次切换先输出停车指令；实测车速持续低于 0.03 m/s 达 1 秒，并且目标控制器
  命令新鲜、室内定位健康时才交权。

不要再同时运行原来的室内和室外启动命令；融合启动脚本已经包含两者需要的组件。

## 编译与启动

```bash
cd /home/tang/autoware_map/map_task4/indoor_localization
source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash
colcon build --packages-select indoor_localization indoor_navigation --symlink-install

./run_hybrid_navigation.sh
```

默认以室外模式启动。第一次实车联调保持自动切换关闭，使用停车式手动交接：

```bash
# 室外 -> 室内（先给 Nav2 下发室内目标，使室内控制源开始输出）
ros2 service call /hybrid_mode_manager/set_indoor std_srvs/srv/SetBool "{data: true}"

# 室内 -> 室外（先在 Autoware 设置室外车道路线）
ros2 service call /hybrid_mode_manager/set_indoor std_srvs/srv/SetBool "{data: false}"

# 查看当前权属、切换阶段、源命令新鲜度和定位健康状态
ros2 topic echo /hybrid_mode_manager/state
```

## 自动切换（待录入实测匹配点）

当前工作区没有收到用户所述的“经纬度匹配点”附件，因此不能安全猜门口围栏。
把实测点录入 `config/hybrid_switch_zones.yaml`：

- `indoor_enter_polygon`：较小的进入围栏；
- `indoor_exit_polygon`：包住进入围栏的较大退出围栏，用于防抖；
- 坐标必须是当前统一 Autoware `map` 坐标，不能直接填经纬度。

在 RViz 核对边界后再启动：

```bash
./run_hybrid_navigation.sh automatic_switching:=true
```

围栏只负责请求模式，仍然不能越过停车、命令超时和室内定位健康检查。

## 上车前检查

```bash
# 最终底盘命令必须只有 hybrid_mode_manager 一个发布者
ros2 topic info /control/command/control_cmd --verbose

# 两个源都应存在；未规划的一侧允许暂时没有数据
ros2 topic hz /hybrid/outdoor/control_cmd
ros2 topic hz /hybrid/indoor/control_cmd

# 统一定位与 GNSS 初始位姿
ros2 topic hz /localization/kinematic_state
ros2 topic hz /sensing/gnss/gnss_poser/pose_with_covariance
```

先架空车轮或断开驱动电机验证模式切换。确认切换阶段始终先输出零速、最终命令只有一个
发布者后，再以低速测试门口往返。

## 已修正的坐标问题

原 `stations.yaml` 仍是旧室内地图坐标，而当前点云/栅格已经注册到 20261117 校园地图。
站点和路径节点现已按 `legacy_room_registration.yaml` 转换到统一 `map` 坐标；否则 Nav2
目标会整体错位约 55 m。
