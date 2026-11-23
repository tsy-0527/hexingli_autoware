# 汇入区域修正说明（lanelet2_map(2).osm）

## 1. 交付物

| 文件 | 说明 |
|---|---|
| `lanelet2_map_merge_fixed.osm` | 修正后的地图（本目录，231 KB） |
| `/home/tang/下载/lanelet2_map(2)_汇入修正.osm` | 同一文件的副本，放在原图旁边 |
| `build_fixed_map.py` | 重建脚本（可重复生成，参数可调） |
| `verify_fixed_map.py` | lanelet2 拓扑 / 可达性 / 重叠 / 路径校验 |
| `validate_osm_file.py` | 文件级校验（ID、引用、宽度、中心线、坐标） |
| `analyze_lanelets.py` / `plot_map.py` | 几何分析与绘图工具 |
| `map_before_*.png` / `map_after_*.png` | 修改前后对比图 |

原始文件 `lanelet2_map(2).osm` 未做任何修改。

## 2. 原图的问题（诊断结果）

用 lanelet2 加载原图后，**7 个 lanelet 之间没有任何一条路由关系**：
`following / previous / lefts / rights` 全部为空，也就是整张图不可路由。

原因：

1. 相邻 lanelet 只是"坐标上看起来接上了"，**没有共享节点，更没有共享 way**。
   lanelet2 判定前后连通要求连接处是同一个节点；判定左右邻接（可变道）
   要求是同一条 way。
2. 原图的接点端面互相**交叉**而不是重合，例如：
   - `conn` 终点端面 `(65.04,-75.80)-(64.96,-78.30)` 与 `east_in` 起点端面
     `(65.01,-75.80)-(64.99,-78.30)` 只是接近；
   - `uturn` 两端与 `east_out`/`east_in` 的端面也是斜交。
3. 汇入区内 `east_in` 比 `east_out` 左边界向北偏了约 **1.7~2.5 m**，
   两车道之间是一条空白带；匝道 `conn` 只接到这条"悬空"车道，
   既不能汇入主路，也没有任何连通关系。

## 3. 修改内容

### 3.1 汇入区域（本次的核心修改）

汇入区位于 `x ≈ 64 ~ 70 m`、主路 `y ≈ -80 m` 处：

- **直行车道贯通**：`main_w` 延伸到汇入断面 `x = 70.05`，
  与 `east_out` 在同一个断面共享节点，直行方向完全没有被截断。
- **匝道平滑并入**：`conn` 末段（原图中最后一段"平直向东"的部分）
  用三次 Bézier 重新放样，从 `(63.93,-77.01)` 平滑下压到
  `(70.02,-78.64)`，终点端面与 `east_in` 起点端面**完全重合**（共享节点）。
- **两条车道并排、可汇入**：`east_in` 重建为主路北侧的左车道，
  右边界直接使用 `east_out` 的左边界（**同一条 way**，`subtype=dashed`），
  因此：
  - `east_in` 与 `east_out` 互为左右邻接；
  - `canChangeLane` 双向为 `True`，匝道车辆可正常汇入直行车道；
  - 两条车道宽度均为 2.5 m，贯穿到东端掉头口。
- 汇入断面处只保留一个公共节点，`conn/east_in` 与 `main_w/east_out`
  分成两对端面，**互不重叠**（汇入区内部 0 重叠）。

### 3.2 为形成可路由拓扑所做的拆分（不改道路外形）

lanelet2 要求每个 lanelet 只在端点接岔，不能在"半路"分叉，
所以原图中被 T 型接点穿过的两个 lanelet 必须拆分：

| 原 lanelet | 拆分后 | 原因 |
|---|---|---|
| `start` | `start`（2000，南段）+ `start_north`（2007，北段） | `park_loop` 的回环端正好接在 `start` 中段 |
| `park` | `park`（2001，接入段）+ `park_loop`（2008，回环段） | `conn` 匝道从 `park` 东侧中段分支 |

拆分点都落在原图本来就存在的接点断面上，**道路的可见几何没有变化**，
只是把内部 lanelet 边界放到了接点上。

### 3.3 其它接点

- `start ↔ park`：`start_north` 末端与 `park` 起点的两个端点改为共享节点。
- `park ↔ conn`：`conn` 起点端面即 `park` 的分支断面，共享节点。
- `park_loop ↔ start/start_north`：`park_loop` 末端端面与 `start` 中段断面
  共享节点（东西两侧端点分别对齐）。
- `east_out ↔ uturn ↔ east_in`：`uturn` 两端端面改为与新
  `east_out`/`east_in` 端面完全一致；掉头口是绕中央分隔带端头的
  标准发卡形，`uturn` 左边界为闭合环、右边界为内弧。
- 所有 lanelet 的 `one_way=no`、`location=urban`、`participant:vehicle=yes`
  等标签保持不变；新增/修改 lanelet 的 `lane_id` 见下表。

## 4. 修正后的 lanelet 清单

长度/宽度由 lanelet2 几何计算（宽度为左右边界逐点最近距离）：

| relation | lane_id | 长度 [m] | 宽度 min/max [m] | 作用 |
|---|---|---|---|---|
| 2000 | `start` | 20.4 | 2.49 / 2.53 | 起步直道南段 |
| 2007 | `start_north` | 7.6 | 2.49 / 2.53 | 起步直道北段（接 park） |
| 2001 | `park` | 28.2 | 2.42 / 2.50 | 环线接入段（park 起点 → 匝道岔口） |
| 2008 | `park_loop` | 104.2 | 2.42 / 2.53 | 环线回环段（匝道岔口 → 回到 start） |
| 2002 | `conn` | 111.9 | 2.49 / 2.50 | 汇入匝道（含新的平滑末段） |
| 2003 | `main_w` | 51.9 | 2.49 / 2.50 | 主路西段（延伸到汇入断面） |
| 2004 | `east_out` | 105.4 | 2.49 / 2.50 | 主路直行车道 |
| 2005 | `east_in` | 106.8 | 2.49 / 2.50 | 汇入车道（与 east_out 共用虚线边界） |
| 2006 | `uturn` | 66.6 | 2.20 / 2.50 | 东端掉头 |

拓扑关系（双向，`⇄` 表示前后连通，`↔` 表示相邻可双向变道）：

```
start ⇄ start_north ⇄ park ⇄ park_loop ⇄ start / start_north
park ⇄ conn ⇄ east_in ⇄ uturn ⇄ east_out ⇄ main_w
east_in ↔ east_out          （共用 dashed 边界，可汇入/变道）
```

## 5. 校验结果（lanelet2 1.2.3，ROS 2 Humble）

复现命令：

```bash
cd /home/tang/autoware_map/map_task4/merge_fix
python3 verify_fixed_map.py lanelet2_map_merge_fixed.osm
python3 validate_osm_file.py lanelet2_map_merge_fixed.osm
```

主要结果：

- `lanelet2.io.load` 成功，9 个 lanelet，标签完整。
- 路由图：
  - `east_out.lefts = [east_in]`，`east_in.rights = [east_out]`；
  - `canChangeLane(east_out, east_in) = True`，反向 `True`；
  - `RoutingGraph.checkValidity()` → `OK`。
- 全连通：从任意 lanelet 出发都能沿 `following/previous` 到达其余全部 lanelet。
- 实际路径（`RoutingGraph.getRoute`）：
  - `main_w → east_out`：`main_w -> east_out`
  - `conn → east_in`：`conn -> east_in`
  - `start → uturn`：`start -> park_loop -> conn -> east_in -> east_out -> uturn`
  - 反向：`east_out -> main_w`、`uturn -> east_out -> main_w`（用 `invert()` 起点/终点）
- 文件级校验：节点/way 引用完整、ID 唯一、无重复相邻节点、
  每个 lanelet 恰好一条 left/right/centerline、端面宽度 2.499~2.531 m、
  中心线全部位于车道多边形内、经纬度与 `local_x/local_y` 换算一致。
- 重叠检查：仅剩两处**岔口固有**的楔形重叠（lanelet2 单车道岔口的
  几何特性，不在汇入区内）：
  - `park ↔ conn`：约 6.51 m²
  - `start_north ↔ park_loop`：约 6.21 m²

  作为对照，用户现有 `lanelet2_map.osm` 中有 17 对 lanelet 重叠，
  最大约 537 m²；`lanelet2_map_测好.osm` 有 128 对重叠。
  因此本图的两处岔口重叠属于同级别建模方式中的正常范围。

## 6. 坐标系

保持原图坐标系不变（`local_x/local_y` 为设计坐标，单位 m）：

```
lat = 35.0        + local_y * 8.983111665353559e-06
lon = 139.0       + local_x * 1.0966354197886395e-05
```

（该线性关系由原图 640 个节点的经纬度拟合得到，残差 < 1e-9°。）

## 7. 可调参数

在 `build_fixed_map.py` 的 `build_geometry()` 中：

| 参数 | 默认 | 含义 |
|---|---|---|
| `LANE_WIDTH` | 2.5 m | 车道宽度 |
| `MERGE_I` | 3 | 汇入断面取 `east_out` 左边界第 3 个点（x≈70.05） |
| `END_T` | 0.4 | 东端共用边界截止位置（x≈175.60） |
| `TAIL_I` | 42 | 匝道开始重放样的中心线点序号（y≈-77.01） |

修改后重新执行 `python3 build_fixed_map.py` 即可生成新图。
