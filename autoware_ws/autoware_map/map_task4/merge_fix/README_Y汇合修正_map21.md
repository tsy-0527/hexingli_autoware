# Y 型汇合/驶出区修正说明（lanelet2_map(21).osm）

## 1. 交付物

| 文件 | 说明 |
|---|---|
| `lanelet2_map21_merge_fixed.osm` | 修正后的地图（本目录） |
| `/home/tang/下载/lanelet2_map(21)_Y汇合修正.osm` | 同一文件的副本，放在原图旁边 |
| `build_fixed_map21.py` | 重建脚本（可重复生成） |
| `verify_map21.py` | lanelet2 拓扑 / 双向路径 / 重叠校验 |
| `validate_osm_file.py` | 文件级校验（ID、引用、宽度、中心线、坐标） |
| `map21_y_before_after.png` | Y 区修改前后对比图 |
| `map21_after_*.png` | 修改后全图/Y 区/北侧/掉头口视图 |

原始文件 `lanelet2_map(21).osm` 未做任何修改。

## 2. 原图的问题（诊断结果）

- 用 lanelet2 加载原图后 **7 个 lanelet 之间没有任何路由关系**
  （`following / previous / lefts / rights` 全空）。
- Y 型汇合处（约 `(60.7, -74)`）：
  - 北侧匝道 `conn` 的端面是**东西向**（`(61.95,-73.96)-(59.45,-74.04)`）；
  - 西侧上行支路 `main_w` 的端面是**西北—东南斜向**
    （`(59.88,-73.06)-(61.52,-74.94)`）；
  - 东侧车道 `east_in` 的起点端面又是**东北—西南斜向**
    （`(61.39,-72.69)-(59.67,-74.51)`）；
  - 三个端面只是在一个中心点附近相交，既没有共享节点，
    端面方向也互不匹配，汇入/驶出都无法通行。
- `east_out` 在南侧 `x ≈ 50.6` 处是一个**孤立死端**，
  与 `main_w` 只是坐标重叠，没有拓扑连接。

## 3. 修改内容

### 3.1 Y 型汇合 / 驶出区（本次核心）

- 把 **汇入/驶出断面移到 `east_in` 的第 13 个点**
  `x ≈ 65.66, y ≈ -76.87`。该处 `east_in` 的切向为东南
  `(0.913, -0.409)`，断面为东北—西南向，长 2.5 m。
- **`conn`（北侧匝道）**：保留北侧原有几何，末端用三次 Bézier
  重新放样，从 `(60.5,-67.99)` 平滑左转，最终以东南切向
  结束在汇合断面；左/右端点直接使用 `east_in` 起点断面的两个节点。
- **`main_link`（西侧上行支路，新增 relation 2009）**：从
  `x ≈ 50` 的分岔断面 `(50.02,-78.73)/(49.94,-81.23)` 出发，
  以 15° 起始切向平缓爬升，再以东南切向并入同一汇合断面。
  车道中心线为一条三次 Bézier，左右边界由 ±1.25 m 法向生成，
  端面分别钉在分岔断面和汇合断面节点上。
- 于是 **`conn` 和 `main_link` 都以完全相同的端面节点接入
  `east_in`**：
  - 汇入：`conn → east_in`、`main_link → east_in`；
  - 驶出：`east_in → conn`、`east_in → main_link`（借助双向车道）。

### 3.2 x ≈ 50 的分岔（驶出/直行）

- 把 `main_w` 在 `x ≈ 50` 的横断面处拆成：
  - `main_w`（2003）：西侧直行段；
  - `main_link`（2009）：通往 Y 区的上行支路。
- `east_out`（2004）西端起点整体吸附到同一个横断面，
  与 `main_w` 共享节点，因此：
  - `main_w → east_out`（直行）、`east_out → main_w`（反向直行）；
  - `main_w → main_link`（上匝道）、`main_link → main_w`（下匝道）。

### 3.3 其它接点（保持原道路外形，仅补拓扑）

- `start` 在 `park` 回环接入的横断面处拆为
  `start`（2000，南段）+ `start_north`（2007，北段），共享节点；
- `park` 在 `conn` 分支处拆为
  `park`（2001，接入段）+ `park_loop`（2008，回环段），共享节点；
- `uturn` 两端端面吸附到新的 `east_out` / `east_in` 端面节点，
  东端掉头口保持发卡形；
- **所有 lanelet 保持 `one_way=no`（双向行驶）**，
  `location=urban`、`participant:vehicle=yes` 等标签保留；
- 原图没有 `lane_id` 标签，本次为便于识别补上了 `lane_id`
  （见下表），relation id 2000–2006 保持不变。

## 4. 修正后的 lanelet 清单

长度/宽度由 lanelet2 几何计算：

| relation | lane_id | 长度 [m] | 宽度 min/max [m] | 作用 |
|---|---|---|---|---|
| 2000 | `start` | 20.4 | 2.50 / 2.53 | 起步直道南段 |
| 2007 | `start_north` | 7.6 | 2.49 / 2.53 | 起步直道北段 |
| 2001 | `park` | 28.2 | 2.42 / 2.50 | 环线接入段 |
| 2008 | `park_loop` | 104.2 | 2.41 / 2.53 | 环线回环段 |
| 2002 | `conn` | 106.0 | 2.49 / 2.50 | 北侧汇入/驶出匝道 |
| 2003 | `main_w` | 31.7 | 2.50 / 2.50 | 西侧直行段（至 x≈50 分岔） |
| 2009 | `main_link` | 16.4 | 2.43 / 2.50 | 西侧上行/下行支路（至 Y） |
| 2004 | `east_out` | 127.9 | 2.50 / 2.50 | 南侧东行/西行车道 |
| 2005 | `east_in` | 111.0 | 2.50 / 2.50 | 北侧东行/西行车道（Y 的出口/入口） |
| 2006 | `uturn` | 65.1 | 2.48 / 2.50 | 东端掉头 |

拓扑关系（全部双向，`⇄` 表示前后连通）：

```
start ⇄ start_north ⇄ park ⇄ park_loop ⇄ start / start_north
park ⇄ conn ⇄ east_in
main_w ⇄ east_out ⇄ uturn ⇄ east_in
main_w ⇄ main_link ⇄ east_in
```

## 5. 校验结果

复现命令：

```bash
cd /home/tang/autoware_map/map_task4/merge_fix
python3 verify_map21.py lanelet2_map21_merge_fixed.osm
python3 validate_osm_file.py lanelet2_map21_merge_fixed.osm
```

主要结果：

- `lanelet2.io.load` 成功，10 个 lanelet，全部 `one_way=no`。
- 路由图关键关系：
  - `east_in.previous = [conn, main_link]`（两个方向都能汇入）；
  - `main_w.following = [east_out, main_link]`；
  - `uturn.next/previous = east_in/east_out`；
  - `RoutingGraph.checkValidity()` → `OK`。
- 全连通：从任意 lanelet 出发都能沿前后关系到达其余全部 lanelet。
- 双向路径实测（`getRoute`，必要时对起点/终点取 `invert()`）：
  - 汇入：`main_link → east_in`、`conn → east_in`；
  - 驶出：`east_in → main_link`、`east_in → conn`；
  - 直行：`main_w → east_out`、`east_out → main_w`；
  - 环线：`main_w → main_link`、`east_in → uturn → east_out`、
    `start → park_loop → conn → east_in`。
- 文件级校验：节点/way 引用完整、ID 唯一、无重复相邻节点、
  每个 lanelet 恰好一条 left/right/centerline、端面宽度 2.50 m、
  中心线全部位于车道多边形内、经纬度与 `local_x/local_y` 换算一致。
- 重叠检查：没有“非接点”重叠；仅剩 4 处岔口固有楔形重叠
  （lanelet2 单车道分/合流的几何特性）：
  - `park ↔ conn`：6.54 m²
  - `conn ↔ main_link`：7.79 m²（Y 汇合楔形）
  - `east_out ↔ main_link`：8.15 m²（x≈50 分岔楔形）
  - `start_north ↔ park_loop`：6.06 m²

  作为对照，本机 `lanelet2_map.osm` 有 17 对 lanelet 重叠，
  最大约 537 m²；因此本图的重叠属于同级别建模方式中的正常范围。

## 6. 坐标系

保持原图坐标系不变（`local_x/local_y` 为设计坐标，单位 m）：

```
lat = 35.0  + local_y * 8.983111665353559e-06
lon = 139.0 + local_x * 1.0966354197886395e-05
```

## 7. 可调参数

在 `build_fixed_map21.py` 中：

| 参数 | 默认 | 含义 |
|---|---|---|
| `MERGE_I` | 13 | Y 汇入/驶出断面取 `east_in` 第 13 个点（x≈65.66） |
| `PARK_I` | 25 | `park` 在 `conn` 分支处的拆分点序号 |
| `MAIN_I` | 3 | `main_w` 在 x≈50 分岔处的拆分点序号 |
| `CONN_TAIL` | 42 | `conn` 开始 Bézier 放样的中心线点序号 |
| `LANE_WIDTH` | 2.5 m | 车道宽度 |
| 15° | — | `main_link` 起始切向（北偏东 15°） |
