# 室内建图工具链

> 本目录已适配当前机器的 MID360S PointCloud2 + FAST-LIO 环境。第一次使用请先阅读
> [本机建图操作流程.md](本机建图操作流程.md)，并运行 `./check_setup.sh`。
> 本次 task4_floor1 失败的完整分析、证据和后续方案见
> [建图失败分析与处理方案.md](建图失败分析与处理方案.md)。

```
record_mapping.sh ─► postprocess.sh ─────────────────────────────► 人工修图 ─► build_map_package.py
   录包+预检          bag2dufo → DUFOMap → mapclean → 栅格预览     CloudCompare    地图包 v1.x + 验证
```

| 文件 | 作用 |
|---|---|
| `record_mapping.sh` | 预检（话题、数据、磁盘、mcap）→ 录包 → 录后频率检查 |
| `postprocess.sh` | 一键跑自动后处理 |
| `bag2dufo.py` | bag → 逐关键帧 PCD（世界系，VIEWPOINT=雷达位姿），去车身点 |
| `mapclean.py` | 调平（地面 z=0，输出 T_old_to_new）、去地面下反射、离群点、车道残影 route_check |
| `make_grids.py` | 导航栅格（0.05 m，切层 0.05–2.0 m）、定位栅格（墙体轮廓 1.2–2.5 m） |
| `build_map_package.py` | 修图后打包：重新生成栅格、限速掩膜、manifest（sha256），自动验证 |
| `validate_map.py` | 轨迹可通行、站点在空闲格、原始扫描与地图一致性 |
| `clean_map_pipeline.sh` | **离线分段一键流程**：重跑 FAST-LIO → bag2dufo → DUFOMap → mapclean → 栅格 → 0.1 m 图 → 质量门控 |
| `check_map_quality.py` | 质量门控：无有效匹配比例、轨迹高度、跳变、地面残差、栅格可通行比例 |
| `export_clean_map.py` | 导出 0.1 m NDT 图和俯视/栅格预览图 |
| `config/postprocess.yaml` | 全部参数，**先改 `vehicle` 段** |
| `config/fastlio_mid360s_mapping_indoor.yaml` | 室内/台阶用 FAST-LIO 配置（blind 0.9 去车体、0.25 m 体素、4 次迭代） |
| `config/zones_template.yaml` | 房间框 / 走廊区 / 站点 / 限速区 / 交接点 |

## 安装（工控机 / 上位机，ROS 2 Humble）

```bash
sudo apt install ros-humble-rosbag2-storage-mcap python3-scipy python3-yaml
pip install python-lzf          # 可选：读 CloudCompare 存的 binary_compressed PCD 更快
# DUFOMap: https://github.com/KTH-RPL/dufomap  编译后得到 build/dufomap_run
```

## 流程

```bash
# 1. 起本机 PointCloud2 驱动和建图专用 FAST-LIO，然后录制
./00_open_mapping_terminals.sh
./record_mapping.sh floor3_east "3F东侧 门全开 起点=东门"

# 2. 自动后处理（上位机跑即可）
DUFOMAP_BIN=~/dufomap/build/dufomap_run ./postprocess.sh \
  /home/tang/fastlio_ws/bags/mapping/floor3_east_*/bag \
  /home/tang/fastlio_ws/bags/mapping/floor3_east_work

# 3. CloudCompare 修 clean/map_leveled.pcd，另存为 PCD

# 4. 打包 + 验证
python3 build_map_package.py edited.pcd ~/mapping_work/floor3_east/clean floor3_east v1.0 \
    --zones zones.yaml --bag /home/tang/fastlio_ws/bags/mapping/floor3_east_*/bag
```

## 分段离线清理（长走廊 / 台阶 / 现场发散）

FAST-LIO2 没有回环；长走廊、台阶、坡道和人员走动容易让扫描匹配失去约束。
本机实测：现场开 RViz + dense 发布 + 录包时，`/cloud_registered` 只有约 4.2 Hz
（期望 10 Hz），长走廊里会先发散；离线 0.4x、关 RViz 重跑可以稳定得多。
推荐把“录包”和“建图”分开，并在台阶/楼层变化处分段：

```bash
# 只录原始 /livox/lidar + /livox/imu 的 bag；录包时关掉 RViz
# 一层主体：从起点到台阶前（这里假设 660 s）
./clean_map_pipeline.sh /path/to/raw_bag task4_floor1 --duration 660 --rate 0.4

# 下层/室外：从台阶后的静止点开始，单独一段
./clean_map_pipeline.sh /path/to/raw_bag task4_lower --start 680 --duration 300 --rate 0.4
```

每段结束先看 `work/clean/quality_report.md`，建议门控：

- 扫描匹配有效帧 ≥ 90%（即 `No Effective Points!` < 10%）；
- 单层轨迹高度波动 < 0.10 m，最大单帧跳变 < 1 m；
- 雷达离地中位数 0.57 ± 0.05 m；
- 地面平面残差 < 2 cm；
- 轨迹在导航栅格上空闲比例 > 95%。

不通过时：缩短段长、在台阶前后各加 10 s 静止、降速到 0.3–0.5 m/s，
或改用 `config/fastlio_mid360s_mapping_indoor.yaml`（`blind: 0.9` 去车体自点、
0.25 m 体素、4 次迭代）。`No Effective Points!` 的含义是当前帧没有任何点
找到有效地图平面，EKF 更新被跳过、位姿只靠 IMU 递推；比例高就说明场景退化。

## 注意事项

**录之前**
- 复测 `config/postprocess.yaml` 的 `vehicle`：雷达离地高度、车身包围盒（雷达系）；外参已与本目录 FAST-LIO 配置对齐。
- 导航要过的门全部打开，固定状态；可移动的东西（推车、椅子）放到平时的位置。清场，人员别在车附近跟着走。
- 只跑驱动 + FAST-LIO，Nav2、D455 等别开，工控机别被抢 CPU / 磁盘带宽导致丢帧。录到本机 SSD，别用 U 盘。
- 起点选在**固定、可复现的位置**（门口交接点 / 控制点），后面室内外对齐要用。

**录制中**
- 开始先静止 10 s 让 IMU 初始化，结束回到起点再静止 10 s。
- 速度 ≤ 0.5 m/s，转弯慢（≤ 30°/s）。阿克曼不能原地转，掉头要用大回环或三点掉头，三点掉头时更要慢。
- 走闭环：起点 → 全部区域 → 回起点。FAST-LIO2 没有回环，终点回起点的偏差就是漂移量，可以在报告里看。
- 长走廊（纵向退化）慢一点，尽量让走廊两端或门框、柱子在视野里；走廊来回各走一趟。
- 门口来回走两次，每个房间转一圈，站点、限速区、交接点都要到。
- 跟车的人站在车后方，别进入车前方视野；就算残影被 DUFOMap / route_check 删了，地图上也会缺一块。

**录完**
- 看脚本输出的频率检查：雷达 ≈ 10 Hz，IMU ≈ 200 Hz，如果丢帧要当场重录。
- 一张地图对应一个 bag。原始 `/livox/lidar`（PointCloud2）+ `/livox/imu` 一定要保留，改参数后可用同一 `lidar_type=4` 配置离线重跑 FAST-LIO。

**后处理**
- 看 `clean/mapclean_report.md` 的警告：雷达实测离地高度与配置差 > 5 cm，说明地面找错了；轨迹高度波动 > 3 cm，或者回到起点 z 对不上，说明有漂移，要考虑重录或分段建图。
- `suspicious.pcd` 用 label 着色复核（1 地面下反射，2 离群，3 车道残影）。玻璃和镜子周围要重点看。
- 玻璃墙、玻璃门雷达会穿透，栅格上容易漏。人工修图时在 3D 地图里补墙，或者在 nav_grid.pgm 上画线，然后打包。
- 3D 地图只做“删”，不要移动、旋转点云，否则 T_old_to_new 和轨迹就对不上了。
- 换地图 = 整包升版本号（v1.0 → v1.1），`zones.yaml` 的坐标也跟着换；脚本会拒绝覆盖已有版本。
- 限速掩膜按 `speed_filter_info.yaml` 配置 costmap_filter_info_server：type 2，multiplier 0.01，掩膜值 30 对应 0.30 m/s，0 表示不限速。
- `validate_map.py` 只检查地图本身。初始化仿真、门口规划、NDT 回放还是要按“上车验证顺序”在实车上做。
