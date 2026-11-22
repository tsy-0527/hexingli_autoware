import { useState, useRef, useEffect, useCallback, useMemo } from "react";
import * as THREE from "three";
import { PCDLoader } from "three/examples/jsm/loaders/PCDLoader.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { DRACOLoader } from "three/examples/jsm/loaders/DRACOLoader.js";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer.js";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass.js";
import { UnrealBloomPass } from "three/examples/jsm/postprocessing/UnrealBloomPass.js";
import { OutputPass } from "three/examples/jsm/postprocessing/OutputPass.js";

// ════════════════════════════════════════════════════════════════════════════
// 现场常改的配置写在 public/config.json 里（改完刷新网页即可，不用重新打包）：
//   车辆通道 vehicleChannels、相机通道 cameraChannels、底盘直连 chassisGateway、地图原点 mapOrigin、地图投影 mapProjection、
//   原点高程 mapOriginAlt、天地图密钥 tiandituKey。
// 下面带“默认值”字样的几项是 config.json 缺失或写错时使用的默认值。
// ════════════════════════════════════════════════════════════════════════════
const CONFIG_URL = "/config.json";
// 关闭后以 public/config.json 中的 vehicleChannels / cameraChannels 为准（改完刷新网页即可，不用重新打包）。
const LAN_ONLY_MODE = false;

// 天地图密钥 (tk)（默认值，可被 config.json 的 tiandituKey 覆盖）
let TIANDITU_TK = "a23db9e9e77571917b2663926e1636e3";

// ── 地图经纬度原点 ──
// 必须与 Autoware 地图原点一致（ros2 topic echo /map/map_projector_info --once 中的 map_origin）
// （默认值，可被 config.json 的 mapOrigin 覆盖）
let MAP_ORIGIN = {
  lat: 31.933957,
  lng: 118.6332695,
};


// ── 网页订阅节流(ms)：rosbridge 同一话题每个节流窗口只推“最新一帧” ──
// 定位按 20 Hz 全速接收（与工控机原始发布频率一致）：实车远程驾驶实时性优先；定位帧很小（约 1 KB），
// 20 Hz 也只有约 20 KB/s，不会打满窄带链路。轨迹约 3.3 Hz、感知 5 Hz、行为路径/GNSS 1 Hz：消息体大，降频避免堆积。
// 高频大消息整体降频防堆积；3D 中车辆位置另有插值补帧，帧到达不匀也不会停顿。
// 自车定位接收间隔：0 = 原始 20 Hz；100 = 10 Hz；200 = 5 Hz。
// 降频后中间由插值补帧（按速度做曲线插值），画面一样连续；代价是显示延迟随之增加（约 2.5 个接收周期）。
const SUB_POSE_MS = 100;
const POSE_PERIOD_MS = SUB_POSE_MS > 0 ? SUB_POSE_MS : 50;
const SUB_CONTROL_MS = 200; // 控制指令回显（5 Hz，只用于系统状态页显示）
const SUB_PLAN_MS = 300;  // 局部轨迹（约 3.3 Hz，仅用于画面引导带）
const SUB_SLOW_MS = 1000; // 行为规划路径，只用于卫星图（1 Hz）
const SUB_GNSS_MS = 1000; // GNSS 经纬度（1 Hz）

// ── 卫星定位话题 ──
// 默认读取 GINS2000 组合导航输出（自定义消息 Gins2000.msg：latitude / longitude / altitude / gnss_fix_status 等）；
// 也兼容标准 sensor_msgs/NavSatFix，按字段自动识别。网址加 ?gnss=/话题名 可临时指定其他话题。
const GNSS_TOPIC = (() => {
  try {
    const q = new URLSearchParams(window.location.search).get("gnss");
    if (q && q.trim().startsWith("/")) return q.trim();
  } catch { /* 忽略 */ }
  return "/sensing/gins2000";
})();
// 消息类型留空 = 让 rosbridge 按话题自动识别（自定义消息无需写包名）。
// 注意：rosbridge 必须在 source 过 Gins2000 消息所在工作空间的终端里启动，否则无法解析该消息。
const GNSS_MSG_TYPE = "";

// 系统状态页用到的话题
const LOC_TOPIC = "/localization/kinematic_state";
const TRAJ_TOPIC = "/planning/scenario_planning/trajectory";
const BEHAVIOR_PATH_TOPIC = "/planning/scenario_planning/lane_driving/behavior_planning/path";
const CONTROL_TOPIC = "/control/command/control_cmd";
const ROUTE_STATE_TOPIC = "/api/routing/state"; // autoware_adapi_v1_msgs/RouteState
const ROUTE_TOPIC = "/api/routing/route";       // autoware_adapi_v1_msgs/Route（含终点位姿）
const WEB_ROUTE_TOPIC = "/web/route";            // 旧版兼容解析；当前不主动订阅
const LOCAL_GOAL_GRACE_MS = 3000;                 // 网页刚设终点后，多久内忽略路线接口里的旧终点
const GEAR_TOPIC = "/control/command/gear_cmd";
const YUNLE_FEEDBACK_TOPIC = "/yunle_chassis/feedback/ccu_status";
const YUNLE_BMS_TOPIC = "/yunle_chassis/feedback/bms_status";
const BMS_STALE_MS = 5000;
const CHASSIS_STALE_MS = 3000; // 超过 3 秒没收到 0x51 底盘反馈，视为中断 // 超过 5 秒未收到 0x100，电量显示为离线，避免误用旧值

// 驱动小车位置/轨迹的关键话题（用于判断数据是否在线）
const LIVE_TOPICS = [
  "/localization/kinematic_state",
  YUNLE_FEEDBACK_TOPIC,
  GNSS_TOPIC,
  "/planning/scenario_planning/trajectory",
  "/planning/scenario_planning/lane_driving/behavior_planning/path",
];
// ── 实车真实物理尺寸与物理参数配置（单位：米） ──
const REAL_VEHICLE_PARAM = {
  length: 1.0,
  width: 0.65,
  height: 0.6,
  wheelbase: 0.632,
  trackWidth: 0.55,
  rearOverhang: 0.18,
};

// 所有贴地元素共用同一个“道路表面”基准。仅用毫米/厘米级差值避免 z-fighting，
// 不再把全局路线、局部轨迹人为悬空到不同高度，防止巡航侧视时产生明显视差。
const MAP_SURFACE_Y = 0.04;

// ── 局部引导带渲染层级与物理高度配置 ──
const GUIDE_WINDOW_LENGTH = 10;
const GUIDE_SAMPLE_COUNT = 41;
const GUIDE_SMOOTHING = 0.25;
const GUIDE_HALF_WIDTH = 0.45;
const GUIDE_TOP_Y = MAP_SURFACE_Y + 0.035;
const GUIDE_BASE_Y = MAP_SURFACE_Y + 0.015;
const GUIDE_CORNER_RADIUS = 0.7;
const ARROW_UPDATE_INTERVAL_MS = 1000 / 20;
const GUIDE_GREEN = 0x7cffb2;
const PATH_ATTACH_DISTANCE = 5;
// 仅用于 3D 显示：在附近车道中心线上查询稳定高度，但不修改路径的水平坐标。
const PATH_LANE_HEIGHT_DISTANCE = 4.0;
const PATH_LANE_HEIGHT_CELL = 8.0;

// ── 视觉风格配置（美化版新增，可按现场效果微调） ──
const SCENE_BG = 0x060a14;          // 场景背景 / 雾色
const ROAD_COLOR = 0x12304d;        // Lanelet 路面填充色
const ROAD_OPACITY = 0.55;
const LANE_EDGE_COLOR = 0x46d7ff;   // 车道边线
const ROAD_SURFACE_Y = MAP_SURFACE_Y;
const LANE_EDGE_Y = MAP_SURFACE_Y + 0.01;
const PCD_LOW_COLOR = 0x0a2f6b;     // 点云按高度渐变：低处深蓝 → 中间亮蓝 → 高处青白
const PCD_MID_COLOR = 0x19b8ff;
const PCD_HIGH_COLOR = 0x9fe9ff;
const SCAN_COLOR = 0x3fd4ff;        // 车底扫描光圈
const FX_Y = 0.1;                   // 车底特效高度
// ── 流畅模式（与 Autoware 同机运行时避免抢占显卡/CPU）──
const BLOOM = { enabled: false, strength: 0.7, radius: 0.45, threshold: 0.68 }; // 泛光后处理，较吃显卡；独立上位机可改 true
const RENDER_FPS = 60;        // 3D 画面最高帧率（60 与 60Hz 屏 1:1，不会掉到 30；仍卡可改 30）
// 60 及以上不限帧：跟随显示器刷新。高刷屏（120/144Hz）上用固定门限会出现“隔一帧画一帧”的节奏抖动
const FRAME_GATE_MS = RENDER_FPS >= 60 ? 0 : 1000 / RENDER_FPS - 2;
// 车辆平滑显示（抖动缓冲）：样本按 ROS 时间戳排到本地时间轴上，显示时刻 = 现在 - 回看时长。
// 回看时长自适应：= 接收周期 + 最近一段时间到达迟到量的高分位（POSE_DELAY_PERCENTILE）+ 余量。
// 局域网抖动小时自动缩短（更跟手），公网卡顿多时自动加长（更稳），无需手动调。
// INTERP_DELAY_MS 只是刚连上、还没统计到抖动时的初始值。
const INTERP_DELAY_MS = Math.max(120, Math.round(POSE_PERIOD_MS * 2.5));
const POSE_DELAY_MIN_MS = 80;   // 自适应回看的下限
const POSE_DELAY_MAX_MS = 600;  // 上限：链路再差也不让画面落后太多
const POSE_DELAY_MARGIN_MS = 20;
// 按到达迟到量的哪个分位数定回看时长：偶发的长卡顿交给速度外推覆盖，不必为它把整体延迟拉长
const POSE_DELAY_PERCENTILE = 0.8;
const POSE_CORRECTION_TAU_S = 0.15; // 外推误差被修正时，在多长时间内平滑吸收
const POSE_DELAY_GROW_RATE = 0.3;   // 回看加长时，每秒最多加长 300 ms
const POSE_DELAY_SHRINK_RATE = 0.05; // 回看缩短时，每秒最多缩短 50 ms
const EXTRAP_MAX_MS = 400; // 链路卡顿时最多沿圆弧外推多久（实车回放：公网偶发卡顿 0.1~0.4 s）
// 跟随相机平滑（只平滑镜头，不改车辆本身位置）：时间常数越大越稳，越小越跟手
const CAM_YAW_TAU_S = 0.45;      // 镜头航向跟随时间常数（定位航向的小幅抖动被 12 m 力臂放大，是跟随视角晃动的主因）
const CAM_YAW_SLOW_TAU_S = 2.0;  // 低速（定位航向不可靠）时的航向时间常数
const CAM_YAW_MIN_SPEED = 0.3;   // m/s，低于该速度认为航向不可靠
const CAM_POS_TAU_S = 0.12;      // 镜头位置跟随时间常数
const CAM_HEIGHT_TAU_S = 0.6;    // 镜头高度跟随时间常数（车道高度在车道间切换时不跳）
const POSE_JUMP_M = 3;         // 相邻两帧定位跳变超过该距离（设起点/重定位）时直接瞬移，不做插值
const START_TIMEOUT_MS = 8000; // 设起点服务无响应的兜底超时
// 实车定位初始化（NDT 在 GNSS 位置附近搜索匹配点云）通常要 5~20 秒，Autoware 完成前不会回复，超时要留足
const GNSS_INIT_TIMEOUT_MS = 45000;
// 全局路线（起点→终点整条路线）显示样式
const ROUTE_COLOR = 0x3fd4ff;
const ROUTE_HALF_WIDTH = 0.55;
const ROUTE_Y = MAP_SURFACE_Y + 0.012;
// 地面高度：从点云地图估算任意位置的地面高程，用于车辆、车道、路面、标牌贴地显示
const GROUND_CELL_M = 1.0;        // 网格边长（米）
const GROUND_PERCENTILE = 0.1;    // 每格取高度的第 10 百分位作为地面（排除车辆、行人、树冠等高处点和个别噪点）
const GROUND_SEARCH_M = 2.0;      // 查询半径（米），取范围内各格地面高度的中位数
const PCD_MAX_POINTS = 250000; // 实时性优先：限制点云顶点量，降低 GPU 占用和主线程等待

// ── 障碍物（欧式聚类 → PredictedObjects，全部为 UNKNOWN，按静态障碍物展示） ──
const OBJECTS_TOPIC = "/perception/object_recognition/objects";
// ── 旧版 web_relay 话题名，仅保留被动解析兼容；当前版本不再主动订阅任何 /web/* ──
const WEB_TRAJ_TOPIC = "/web/trajectory";
const WEB_OBJECTS_TOPIC = "/web/objects";
const WEB_PATH_TOPIC = "/web/behavior_path";
const WEB_TOPIC_ALIAS = { // 瘦身话题 → 对应的原始话题（系统状态页按原始话题统计）
  [WEB_TRAJ_TOPIC]: "/planning/scenario_planning/trajectory",
  [WEB_OBJECTS_TOPIC]: OBJECTS_TOPIC,
  [WEB_PATH_TOPIC]: "/planning/scenario_planning/lane_driving/behavior_planning/path",
};
const SUB_OBJ_MS = 200;        // 画面感知显示 5 Hz；Autoware 内部感知/制动不受影响
const OBJ_STALE_MS = 1500;     // 超过该时长没有新消息，隐藏全部障碍物（防止链路断开后残留“幽灵”）
const OBJ_SMOOTH = 0.25;       // 每帧向最新位置靠近的比例
const OBJ_MIN_HEIGHT = 0.15;   // 低于该高度的簇视为地面残留，不显示
const OBJ_MIN_SIZE = 0.1;      // footprint 半径小于该值不显示
const OBJ_MAX_DIST = 40;       // 只显示本车周围该距离内的障碍物
const OBJ_SHOW_AFTER = 2;      // 连续出现几帧才显示（过滤一闪而过的噪声簇）
const OBJ_LINGER_MS = 600;     // 消失后保留并淡出的时长（漏检一帧不闪烁）
const OBJ_MATCH_M = 1.0;       // ID 变了但位置相差小于该值，视为同一个障碍物
const OBJ_SHAPE_EPS = 0.15;    // 轮廓变化超过该值（米）才重建几何体
const OBJ_AHEAD_MIN_M = 8;     // “前方有障碍物”检查车前至少多远（沿规划轨迹）
const OBJ_AHEAD_TIME_S = 4;    // 车速快时按 4 秒行驶距离检查（取两者较大）
const PATH_STALE_MS = 2000;    // 规划轨迹超过该时长没更新，视为过期，不再用于挡路判断
const OBJ_NEAR_M = 5;          // 距本车小于该距离显示琥珀色 + 距离
const OBJ_COLOR = { far: 0x8fa3bf, near: 0xffb547, danger: 0xff3b4f };

// ── 系统状态页：统计频率的话题与各项的预期频率（低于预期一半显示“降频”） ──
const HEALTH_TOPICS = [...LIVE_TOPICS, OBJECTS_TOPIC, CONTROL_TOPIC];
// 预期频率 = 网页实际订阅的频率（不是 Autoware 内部的发布频率）：网页为省流量做了限频，
// 例如障碍物最多 5 Hz。以前写死 10 Hz、低于一半判“变慢”，实收 4.5~5 Hz 时就会在正常/变慢之间来回跳。
const HEALTH_EXPECT_HZ = {
  loc: 1000 / POSE_PERIOD_MS,       // 10 Hz（SUB_POSE_MS = 100）
  perception: 1000 / SUB_OBJ_MS,    // 5 Hz
  planning: 1000 / SUB_PLAN_MS,     // 约 3.3 Hz（web_relay 转发为 5 Hz，按较低者判断）
  control: 1000 / SUB_CONTROL_MS,   // 5 Hz
  chassis: 10,                      // 底盘反馈订阅上限 20 Hz（throttle 50 ms），低于 5 Hz 判“变慢”
};
const HEALTH_LOST_MS = 3000;  // 超过该时长没数据视为“中断”
const HEALTH_SLOW_MS = 1000;  // 超过该时长没数据视为“降频”

// ── 仿真障碍物（配合工控机 click_to_obstacle 节点，经 dummy_perception_publisher 送入 Autoware） ──
const OBSTACLE_CMD_TOPIC = "/web/obstacle/cmd";    // 上位机 → 节点：添加 / 删除 / 清空（std_msgs/String，JSON）
const OBSTACLE_LIST_TOPIC = "/web/obstacle/list";  // 节点 → 上位机：当前障碍物列表（含 RViz 放置的）
const OBSTACLE_NODE_TIMEOUT_MS = 4000;             // 超过该时长收不到列表，视为节点未运行
// 类别一律为 UNKNOWN（与实车欧式聚类输出一致），预设只改变尺寸：[前后, 左右, 高]（米）
const OBSTACLE_PRESETS = [
  { id: "s", name: "0.5×0.5×1.5", size: [0.5, 0.5, 1.5] }, // 与节点默认尺寸一致
  { id: "low", name: "0.4×0.4×0.7", size: [0.4, 0.4, 0.7] },
  { id: "m", name: "0.8×0.8×0.8", size: [0.8, 0.8, 0.8] },
  { id: "l", name: "4.5×1.8×1.5", size: [4.5, 1.8, 1.5] },
];
const OBSTACLE_LABEL = "UNKNOWN";
const OBSTACLE_COLOR = 0xffb547;

// ── 地图投影（卫星图与经纬度换算） ──
// MAP_PROJECTION 必须与 Autoware 地图一致，查看方法：ros2 topic echo /map/map_projector_info --once
//   projector_type → type（"MGRS" / "LocalCartesianUTM" / "LocalCartesian"）；map_origin → originLat / originLon / originAlt
// （默认值，可被 config.json 的 mapProjection / mapOriginAlt 覆盖）
let MAP_PROJECTION = { type: "LocalCartesianUTM", originLat: MAP_ORIGIN.lat, originLon: MAP_ORIGIN.lng, originAlt: 0 };

// ── 相机画面（MediaMTX WebRTC，即浏览器打开 http://<IP>:8889/cam/ 看到的那一路流） ──
// 网页直接通过 WHEP（http://<IP>:8889/cam/whep）拉流到 <video>，不经过 rosbridge，延迟约 100~300 ms。
// 相机通道：在“相机画面”标题栏上手动切换，不会自动切换（选择记在浏览器里，刷新后保留）。
// 每个通道按钮上显示是否连通和带宽：当前通道取实时播放数据；另一个通道每隔 CAMERA_REPROBE_MS 建一条临时连接测一次。
// 与顶栏的 rosbridge 地址互不影响。网址加 ?cam=地址1,地址2 可临时覆盖通道列表。
// 注意：网页需以 http 打开；若网页是 https，浏览器会拦截 http 视频地址，需给 MediaMTX 开 HTTPS。
// 地址可写 ws:// / wss://，会自动换成对应的 http:// / https://（WHEP 是 HTTP 请求）；不写路径时默认流名 /cam/
// ⚠️ frp 一个端口只能转发一个服务：若 43141 转发的是 rosbridge(9090)，相机的公网通道要给 MediaMTX(8889) 另开端口
// （默认值，可被 config.json 的 cameraChannels 覆盖）
let CAMERA_CHANNELS = [
  { name: "网络信号", url: "http://10.66.0.2:8889/cam/" }, // WireGuard 网络通道
];
const CAMERA_PROBE_MS = 2500;          // 备用通道测速：收到数据后统计多长时间的码率 / 丢包 / 延迟
const CAMERA_PROBE_TIMEOUT_MS = 8000;  // 单个通道测速的总时限（连不上 / 一直没数据就判为不通）
const CAMERA_REPROBE_MS = 30000;       // 备用通道多久测一次（测速时会短暂多占一路视频带宽）
const CAMERA_FOCUS_Y = 65;               // “铺满”时默认的纵向位置（%）：50 居中，越大越偏下（多留路面）；界面上可按住画面上下拖动调整
const CAMERA_ICE_GATHER_TIMEOUT_MS = 500;  // 等待本地 ICE 候选收集的上限（局域网候选几十毫秒内就齐了，等太久只会拖慢出图）
const CAMERA_JITTER_BUFFER_MS = 150;       // 浏览器接收缓冲（毫秒）：经 ZeroTier 等有抖动的链路时留一点缓冲，画面更稳；局域网直连可改 0 追求最低延迟
const CAMERA_CONNECT_TIMEOUT_MS = 10000;   // 发起连接后多久还没出画面就重连
const CAMERA_FREEZE_MS = 7000;             // 出画面后画面停住超过该时长才重连（短暂丢包等关键帧即可恢复，重连反而更慢）
const CAMERA_RETRY_MS = 2000;              // 失败后的首次重试间隔，连续失败时逐步加长到 CAMERA_RETRY_MAX_MS
const CAMERA_RETRY_MAX_MS = 8000;
const CAMERA_PIN_HOST = true;              // 只用相机地址那个 IP 的 ICE 候选（MediaMTX 会报出工控机所有网卡的地址，浏览器在多条路之间切换会导致断流）
const CAMERA_STATS = true;                 // 画面左下角显示传输状态（协议 / 码率 / 帧率 / 丢包 / 延迟），方便判断卡顿原因
const GNSS_STALE_MS = 3000;           // 超过该时长没有新的 GNSS 数据，视为无信号
const GNSS_VERIFY_DELAY_MS = 600;     // 工控机初始化完成后，等多久再读取小车位置
// 设起点 / 设终点发给 Autoware 的高度固定为 0
// 3D 画面中车辆贴着点云地面显示（false 则按工控机定位的 z 显示）
const VEHICLE_ON_GROUND = true;
const GNSS_STATUS_TEXT = { "-1": "无解", 0: "单点", 1: "差分", 2: "RTK" };
const MAX_PIXEL_RATIO = 1;    // 渲染分辨率倍数（高分屏上 2 会让显卡负载约增 4 倍）
const SHADOWS = false;        // 实时阴影
const UI_EGO_MS = 200;        // 界面数据刷新间隔（最多 5 次/秒重绘；3D 车辆位置另有插值，依旧平滑）
const CINEMA_SHOT_MS = 9000;        // 巡航镜头每个机位停留时长
// 点云“雷达扫描波”：以车辆为圆心向外扩散的光环（纯显示效果）
const SCAN_WAVE = { periodMs: 3200, maxRadius: 45, width: 1.6, color: [0.35, 0.9, 1.0], intensity: 1.4 };
// 开场自检动画：最短展示时间 / 等待数据的最长时间
const BOOT_MIN_MS = 2400;
const BOOT_MAX_MS = 12000;     // 超过这个时间仍未就绪的项目判为失败，并显示原因
const BOOT_AUTO_ENTER_S = 10;  // 有失败项时，再等多少秒自动进入（无人值守的大屏也能进入主界面）
// 云乐底盘手动控制：网页经 rosbridge 发布 Autoware 标准 Control + GearCommand，
// 与 ros2 topic pub /control/command/control_cmd 的已验证链路完全一致，再由 yunle_autoware_bridge 转成 0x121。
// speed 单位 m/s，steering 为 Autoware 轮胎转角(rad，左正右负)。
const TELEOP = {
  speed: 5 / 3.6, // 5 km/h；网关端同时执行 5 km/h 硬限幅
  feedbackTimeoutMs: 500,
  // 实测机械限位约为左 20.5°、右 20.9°；网关将 21°映射为底盘最大编码 ±120。
  steering: (21 * Math.PI) / 180,
  periodMs: 50,
};
// 底盘手动控制和 CCU/BMS 反馈走独立 SocketCAN 网关，不经过 ROS/rosbridge。
// url/token 可由 public/config.json 的 chassisGateway 覆盖。
let CHASSIS_GATEWAY = {
  enabled: true,
  url: `${window.location.protocol === "https:" ? "wss" : "ws"}://${window.location.hostname || "127.0.0.1"}:8765`,
  token: "",
};
// 工控机 rosbridge 通道：在“智能驾驶实况”标题栏上手动切换，不会自动切换（选择记在浏览器里）。
// 没在用的通道每隔 VEHICLE_PROBE_MS 短暂订阅同一组主要话题，显示实际接收速度。
// 支持 "IP"、"IP:端口"、"ws://..."、"wss://..."（缺省端口 9090；同一地址 ws/wss 两种协议都会试）。
// 顶栏点“连接状态”可填写其他地址；网址加 ?ip= 可临时指定。
// （默认值，可被 config.json 的 vehicleChannels 覆盖）
let VEHICLE_CHANNELS = [
  // 明确写出 ws:// 和 9090，避免部署环境把 rosbridge 误判成 WSS。
  { name: "网络信号", url: "ws://10.66.0.2:9090" }, // WireGuard 网络通道
];
// Electron 专用 UDP 实时定位通道。只承载可丢帧的定位遥测；控制、服务调用和其他 ROS 数据仍走 rosbridge。
// UDP 超时后会自动恢复独立 WebSocket 定位通道，UDP 恢复时再自动切回。
let ROS_UDP_TELEMETRY = {
  enabled: true,
  listenAddress: "0.0.0.0",
  listenPort: 15000,
  allowedSource: "", // 可填工控机/WireGuard IP；留空表示不按源地址过滤
  fallbackMs: 3000,
  objectsEnabled: true,
  objectsFallbackMs: 3000,
  trajectoryEnabled: true,
  trajectoryFallbackMs: 3000,
  pathEnabled: true,
  pathFallbackMs: 4000,
};

// UDP 诊断按“可用帧”统计频率/间隔，按 seq 统计整帧丢失，按 part/parts 统计分包不完整。
// rosbag 的 header.stamp 是录制时间，因此这里只显示到达间隔和抖动，不把它误报成绝对单向延迟。
const UDP_DIAG_SPECS = {
  pose: { label: "定位", expectedHz: 1000 / POSE_PERIOD_MS },
  objects: { label: "障碍物", expectedHz: 1000 / SUB_OBJ_MS },
  trajectory: { label: "控制轨迹", expectedHz: 1000 / SUB_PLAN_MS },
  path: { label: "行为路径", expectedHz: 1000 / SUB_SLOW_MS },
};
const UDP_SEQ_MOD = 0x100000000;

function createUdpDiagnosticsStore() {
  const now = Date.now();
  return {
    startedAt: now,
    types: Object.fromEntries(Object.keys(UDP_DIAG_SPECS).map((type) => [type, {
      packets: 0,
      bytes: 0,
      framesSeen: 0,
      framesComplete: 0,
      missingFrames: 0,
      incompleteFrames: 0,
      outOfOrderFrames: 0,
      duplicateFrames: 0,
      lastSeq: null,
      lastPacketAt: 0,
      lastCompleteAt: 0,
      completeTimes: [],
      intervals: [],
      source: "",
      sampleAt: now,
      sampleBytes: 0,
    }])),
  };
}

function udpDiagStartFrame(store, type, rawSeq, now) {
  const stat = store.types[type];
  const value = Number(rawSeq);
  if (!stat || !Number.isFinite(value)) return;
  const seq = value >>> 0;
  if (stat.lastSeq == null) {
    stat.lastSeq = seq;
    stat.framesSeen += 1;
    return;
  }
  const delta = (seq - stat.lastSeq + UDP_SEQ_MOD) % UDP_SEQ_MOD;
  if (delta === 0) {
    stat.duplicateFrames += 1;
    return;
  }
  if (delta < UDP_SEQ_MOD / 2) {
    if (delta > 1) stat.missingFrames += delta - 1;
    stat.lastSeq = seq;
    stat.framesSeen += 1;
    return;
  }
  // 发送脚本重启后 seq 会从 1 重新开始，不把整段重启误算成乱序。
  if ((stat.lastPacketAt > 0 && now - stat.lastPacketAt > 3000) || (seq < 1000 && stat.lastSeq > 1000)) {
    stat.lastSeq = seq;
    stat.framesSeen += 1;
    return;
  }
  stat.outOfOrderFrames += 1;
}

function udpDiagPacket(store, type, bytes, source, now) {
  const stat = store.types[type];
  if (!stat) return;
  stat.packets += 1;
  stat.bytes += Math.max(0, Number(bytes) || 0);
  stat.lastPacketAt = now;
  if (source) stat.source = source;
}

function udpDiagCompleteFrame(store, type, now) {
  const stat = store.types[type];
  if (!stat) return;
  stat.framesComplete += 1;
  if (stat.lastCompleteAt > 0) {
    const gap = now - stat.lastCompleteAt;
    if (gap > 0 && gap < 60000) {
      stat.intervals.push(gap);
      if (stat.intervals.length > 300) stat.intervals.shift();
    }
  }
  stat.lastCompleteAt = now;
  stat.completeTimes.push(now);
  if (stat.completeTimes.length > 300) stat.completeTimes.shift();
}

function udpDiagIncompleteFrame(store, type) {
  const stat = store.types[type];
  if (stat) stat.incompleteFrames += 1;
}

function percentile(values, ratio) {
  if (!values.length) return 0;
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.min(sorted.length - 1, Math.floor(ratio * sorted.length))];
}

function snapshotUdpDiagnostics(store, now = Date.now()) {
  const types = {};
  let totalExpected = 0;
  let totalLost = 0;
  let totalKbps = 0;
  let activeCount = 0;
  Object.entries(UDP_DIAG_SPECS).forEach(([type, spec]) => {
    const stat = store.types[type];
    const elapsedSec = Math.max(0.001, (now - stat.sampleAt) / 1000);
    const kbps = Math.max(0, ((stat.bytes - stat.sampleBytes) * 8) / elapsedSec / 1000);
    stat.sampleAt = now;
    stat.sampleBytes = stat.bytes;
    const recentCutoff = now - 5000;
    stat.completeTimes = stat.completeTimes.filter((at) => at >= recentCutoff);
    const recentSpanSec = Math.max(1, Math.min(5, (now - store.startedAt) / 1000));
    const hz = stat.completeTimes.length / recentSpanSec;
    const expectedFrames = stat.framesSeen + stat.missingFrames;
    const lostFrames = stat.missingFrames + stat.incompleteFrames;
    const lossPct = expectedFrames > 0 ? Math.min(100, (lostFrames / expectedFrames) * 100) : 0;
    const p95GapMs = percentile(stat.intervals, 0.95);
    const maxGapMs = stat.intervals.length ? Math.max(...stat.intervals) : 0;
    const ageMs = stat.lastCompleteAt > 0 ? now - stat.lastCompleteAt : null;
    if (expectedFrames > 0) {
      activeCount += 1;
      totalExpected += expectedFrames;
      totalLost += lostFrames;
    }
    totalKbps += kbps;
    types[type] = {
      ...spec,
      packets: stat.packets,
      bytes: stat.bytes,
      framesSeen: stat.framesSeen,
      framesComplete: stat.framesComplete,
      missingFrames: stat.missingFrames,
      incompleteFrames: stat.incompleteFrames,
      outOfOrderFrames: stat.outOfOrderFrames,
      duplicateFrames: stat.duplicateFrames,
      lossPct,
      hz,
      kbps,
      p95GapMs,
      maxGapMs,
      ageMs,
      source: stat.source,
    };
  });
  return {
    startedAt: store.startedAt,
    elapsedMs: Math.max(0, now - store.startedAt),
    types,
    activeCount,
    totalKbps,
    totalExpected,
    totalLost,
    lossPct: totalExpected > 0 ? Math.min(100, (totalLost / totalExpected) * 100) : 0,
  };
}
const WS_CONNECT_TIMEOUT_MS = 5000; // 握手超过该时长视为连不上（ZeroTier 不通时浏览器可能卡二三十秒才报错）
const VEHICLE_PROBE_SAMPLE_MS = 2500; // 备用车辆通道每次接收测速的采样时长
const VEHICLE_PROBE_MS = 15000;       // 没在用的车辆通道多久重测一次（测速期间会短暂接收 ROS 数据）
const TELE_IDLE = { up: false, down: false, left: false, right: false, brake: false };

const CAR_MODELS = [
  // yawOffset：把车模自身的朝向转到“车头朝前”。若车头仍反向，改为 Math.PI 即可
  { id: "white", name: "白色小车", path: "/model/white_car.glb", yawOffset: 0 },
  { id: "yellow", name: "黄色小车", path: "/model/yellow_car.glb", yawOffset: Math.PI * 1.5 },
  { id: "ice_cream", name: "冰淇淋车", path: "/model/ice_cream_car.glb", yawOffset: 0 },
];

// ── 界面文案映射：演示模式下用“人话”展示系统状态 ──

// map 坐标 ↔ 经纬度：统一按 MAP_PROJECTION 换算（与 Autoware 地图投影一致，卫星图、GNSS 初始化共用）
function cartesianToLatLon(x, y) {
  return mapToLatLon(x, y, MAP_PROJECTION);
}

function latLonToCartesian(lat, lng) {
  return projectToMap(lat, lng, 0, MAP_PROJECTION);
}

function normalizeAngle(angle) {
  while (angle > Math.PI) angle -= Math.PI * 2;
  while (angle < -Math.PI) angle += Math.PI * 2;
  return angle;
}

// 障碍物 UUID → 稳定字符串（rosbridge 会把 uint8[] 编成 base64 字符串，也可能是数组）
function objectIdKey(id, i) {
  const u = id?.uuid;
  if (typeof u === "string") return u;
  if (Array.isArray(u)) return u.map((b) => b.toString(16).padStart(2, "0")).join("");
  return `obj_${i}`;
}

// ── 网络时序记录（离线优化测试用）──
// 网址加 ?netlog=1 开启：记录每条定位 / 瘦身话题的 ROS 时间戳、到达网页的时刻和字节数，
// 顶栏出现“下载网络记录”按钮（或在控制台执行 dtNetLog.download()）。
// 与同一时段工控机录的 bag 一起交给回放工具，即可用真实链路抖动做离线测试。
const NETLOG_ENABLED = (() => {
  try { return new URLSearchParams(window.location.search).get("netlog") === "1"; } catch { return false; }
})();
const NETLOG_MAX_ROWS = 400000; // 约 10 Hz 定位 + 瘦身话题，可记录 5 小时以上
const netLog = {
  rows: [],
  startEpochMs: Date.now(),
  startPerfMs: typeof performance !== "undefined" ? performance.now() : 0,
  push(topic, channel, stampMs, arrival, bytes) {
    if (!NETLOG_ENABLED || this.rows.length >= NETLOG_MAX_ROWS) return;
    this.rows.push(`${topic},${channel},${stampMs.toFixed(3)},${arrival.toFixed(2)},${bytes}`);
  },
  download() {
    const head = `# netlog v1 start_epoch_ms=${this.startEpochMs} start_perf_ms=${this.startPerfMs.toFixed(2)} ua=${navigator.userAgent.replace(/,/g, " ")}\n` +
      "topic,channel,stamp_ms,arrival_perf_ms,bytes\n";
    const blob = new Blob([head + this.rows.join("\n") + "\n"], { type: "text/csv" });
    const a = document.createElement("a");
    const d = new Date();
    const p = (v) => String(v).padStart(2, "0");
    a.href = URL.createObjectURL(blob);
    a.download = `netlog_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.csv`;
    document.body.appendChild(a);
    a.click();
    setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
  },
};
if (NETLOG_ENABLED && typeof window !== "undefined") {
  window.dtNetLog = netLog;
  console.info("🧪 网络时序记录已开启；测试结束后点顶栏“下载网络记录”，或在控制台执行 dtNetLog.download()");
}

// ── 轨迹 / 障碍物 / 路线解析（纯函数）：原始 Autoware 消息与 web_relay 瘦身格式共用同一套下游逻辑 ──
const quatYaw = (q) => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

// 障碍物统一成 { id, px, py, pz（ROS）, yaw, H, local:[[x,y]...]（物体自身坐标系轮廓） }
function objectsFromRaw(list) {
  return (list || []).map((obj, i) => {
    const pose = obj.kinematics?.initial_pose_with_covariance?.pose || {};
    const p = pose.position || { x: 0, y: 0, z: 0 };
    const shape = obj.shape || {};
    const d = shape.dimensions || {};
    let local = (shape.footprint?.points || []).map((pt) => [Number(pt.x), Number(pt.y)]);
    if (shape.type !== 2 || local.length < 3) {
      const L = Number(d.x) || 0;
      const W = Number(d.y) || 0;
      local = shape.type === 1
        ? Array.from({ length: 12 }, (_, k) => {
            const a = (k / 12) * Math.PI * 2;
            return [(Math.cos(a) * L) / 2, (Math.sin(a) * L) / 2];
          })
        : [[L / 2, W / 2], [-L / 2, W / 2], [-L / 2, -W / 2], [L / 2, -W / 2]];
    }
    return { id: objectIdKey(obj.object_id, i), px: Number(p.x), py: Number(p.y), pz: Number(p.z), yaw: quatYaw(pose.orientation || { x: 0, y: 0, z: 0, w: 1 }), H: Number(d.z) || 0, local };
  });
}
function objectsFromWeb(data) {
  return (data?.o || []).map((o, i) => ({
    id: o.i || `obj_${i}`, px: Number(o.x), py: Number(o.y), pz: Number(o.z), yaw: Number(o.yaw) || 0, H: Number(o.h) || 0,
    local: Array.isArray(o.fp) && o.fp.length >= 3
      ? o.fp.map(([x, y]) => [Number(x), Number(y)])
      : (() => {
          const L = Number(o.l) || 0;
          const W = Number(o.w) || 0;
          return [[L / 2, W / 2], [-L / 2, W / 2], [-L / 2, -W / 2], [L / 2, -W / 2]];
        })(),
  }));
}
// footprint 换算成“世界朝向下相对中心的偏移”（Three：x = ROS Δy, z = ROS Δx），渲染时不再旋转，朝向跳变时画面不抖
function objectsToScene(items) {
  const objects = [];
  for (const it of items) {
    const cs = Math.cos(it.yaw);
    const sn = Math.sin(it.yaw);
    const fp = it.local
      .filter(([px, py]) => Number.isFinite(px) && Number.isFinite(py))
      .map(([px, py]) => ({ x: px * sn + py * cs, z: px * cs - py * sn }));
    if (fp.length < 3) continue;
    const R = Math.max(...fp.map((v) => Math.hypot(v.x, v.z)));
    if (it.H < OBJ_MIN_HEIGHT || R < OBJ_MIN_SIZE) continue;
    if (!Number.isFinite(it.px) || !Number.isFinite(it.py)) continue;
    objects.push({ id: it.id, x: it.py, y: it.pz, z: it.px, H: it.H, R, fp });
  }
  return objects;
}
// 控制器轨迹 → Three 坐标点列
function trajectoryFromRaw(points) {
  return (points || [])
    .map((pt) => {
      const position = (pt.pose || pt.point?.pose || {}).position;
      return position ? { x: Number(position.y), z: Number(position.x) } : null;
    })
    .filter((point) => point && Number.isFinite(point.x) && Number.isFinite(point.z));
}
function trajectoryFromWeb(data) {
  return (data?.p || []).map(([x, y]) => ({ x: Number(y), z: Number(x) })).filter((pt) => Number.isFinite(pt.x) && Number.isFinite(pt.z));
}
// 行为路径 → ROS 坐标点列 { x, y, z }
function routeFromRaw(points) {
  return (points || [])
    .map((pt) => {
      const position = (pt.pose || pt.point?.pose || {}).position;
      if (!position || !Number.isFinite(Number(position.x)) || !Number.isFinite(Number(position.y))) return null;
      return { x: Number(position.x), y: Number(position.y), z: Number.isFinite(Number(position.z)) ? Number(position.z) : 0 };
    })
    .filter(Boolean);
}
function routeFromWeb(data) {
  return (data?.p || []).map(([x, y, z]) => ({ x: Number(x), y: Number(y), z: Number(z) || 0 })).filter((pt) => Number.isFinite(pt.x) && Number.isFinite(pt.y));
}

// 按弧长把折线重采样为 n 个点（用于左右边界点数不一致时生成路面）
function resamplePolyline(points, n) {
  if (points.length < 2 || n < 2) return points.slice();
  const cum = [0];
  for (let i = 1; i < points.length; i++) {
    cum.push(cum[i - 1] + Math.hypot(points[i].x - points[i - 1].x, points[i].z - points[i - 1].z));
  }
  const total = cum[cum.length - 1];
  if (total < 1e-6) return Array.from({ length: n }, () => ({ ...points[0] }));
  const out = [];
  let j = 1;
  for (let k = 0; k < n; k++) {
    const d = (total * k) / (n - 1);
    while (j < points.length - 1 && cum[j] < d) j++;
    const seg = Math.max(cum[j] - cum[j - 1], 1e-6);
    const t = Math.min(1, Math.max(0, (d - cum[j - 1]) / seg));
    const a = points[j - 1];
    const b = points[j];
    out.push({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, z: a.z + (b.z - a.z) * t });
  }
  return out;
}

// 按“显示时刻 = 现在 - INTERP_DELAY_MS”在定位缓冲区中插值出车辆位姿
// ── 定位时间轴（纯函数，离线回放测试直接调用这一份代码） ──
// 把 ROS 时间戳换算到本地 performance.now() 时间轴，并根据实测到达抖动给出建议回看时长。
function createPoseTimeline() {
  return { offset: null, lastStamp: 0, excess: [], gaps: [], recentD: [], targetDelay: INTERP_DELAY_MS };
}
function poseTimelinePush(tl, stampMs, arrival) {
  if (!(stampMs > 0)) return arrival;
  if (tl.lastStamp > 0 && tl.lastStamp - stampMs >= 10000) { // ROS 时钟重置
    tl.offset = null; tl.lastStamp = 0; tl.excess.length = 0; tl.gaps.length = 0; tl.recentD.length = 0;
  }
  // 偏移取历史上最快到达的那一帧（更快的帧一来就立即采用）。
  // 只有“最近 20 帧全部变慢”（链路持续变差或两机时钟漂移）才缓慢上调：每帧上调 2%，播放速度变化不到 2%，看不出来。
  // 偶发卡顿时成批迟到的帧不动偏移——实车回放发现，那样会让显示时刻往回挪十几毫秒，画面就顿一帧。
  const d = arrival - stampMs;
  tl.recentD.push(d);
  if (tl.recentD.length > 20) tl.recentD.shift();
  if (tl.offset == null || d < tl.offset) tl.offset = d;
  else if (tl.recentD.length >= 20) {
    const persistent = Math.min(...tl.recentD) - tl.offset;
    if (persistent > 0) tl.offset += persistent * 0.02;
  }
  const sampleT = stampMs + tl.offset;
  tl.excess.push(arrival - sampleT); // 这一帧比“最快情况”多晚了多久
  if (tl.excess.length > 200) tl.excess.shift();
  const gap = tl.lastStamp > 0 ? stampMs - tl.lastStamp : 0;
  if (gap > 0 && gap < 1000) { tl.gaps.push(gap); if (tl.gaps.length > 40) tl.gaps.shift(); }
  tl.lastStamp = stampMs;
  if (tl.excess.length >= 10) {
    const q = (arr, p) => { const s = [...arr].sort((a, b) => a - b); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };
    const period = tl.gaps.length >= 5 ? q(tl.gaps, 0.9) : POSE_PERIOD_MS;
    // 显示时刻之后的下一帧，最晚在“周期 + 迟到量”之后才到；回看时长要覆盖它才不会外推
    tl.targetDelay = Math.min(POSE_DELAY_MAX_MS, Math.max(POSE_DELAY_MIN_MS, period + q(tl.excess, POSE_DELAY_PERCENTILE) + POSE_DELAY_MARGIN_MS));
  }
  return sampleT;
}

// ── 显示位姿的误差平滑吸收（纯函数，离线回放测试直接调用） ──
// 卡顿时车是沿速度外推的，迟到的定位一到就要切回真实轨迹，误差虽只有 1 cm 左右，
// 但集中在一帧里就是一下“抽动”。这里检测“偏离上一帧运动趋势”的突变，把它摊到 POSE_CORRECTION_TAU_S 内完成。
// 超过 0.5 m / 0.3 rad 的跳变视为重定位，直接瞬移，不做平滑。
function createPoseSmoother() {
  return { prev: null, vx: 0, vz: 0, vyaw: 0, cx: 0, cz: 0, cyaw: 0, prevExtrap: false, prevBaseT: null };
}
function smoothDisplayPose(sm, raw, dt) {
  const decay = Math.exp(-dt / POSE_CORRECTION_TAU_S);
  sm.cx *= decay; sm.cz *= decay; sm.cyaw *= decay;
  if (sm.prev && dt > 0) {
    const dx = raw.x - sm.prev.x;
    const dz = raw.z - sm.prev.z;
    const dyaw = normalizeAngle(raw.yaw - sm.prev.yaw);
    // 只在“上一帧还在外推、这一帧外推的起点换成了新到的定位”时吸收：
    // 这一帧的位移 = 正常运动 + 外推误差，按上一帧速度扣掉正常运动，剩下的就是要平滑掉的误差。
    const corrected = sm.prevExtrap && (!raw.extrapolated || raw.baseT !== sm.prevBaseT);
    if (corrected) {
      const jx = dx - sm.vx * dt;
      const jz = dz - sm.vz * dt;
      const jyaw = dyaw - sm.vyaw * dt;
      if (Math.hypot(jx, jz) < 0.5) { sm.cx -= jx; sm.cz -= jz; } // 超过 0.5 m 视为重定位，直接瞬移
      if (Math.abs(jyaw) < 0.3) sm.cyaw -= jyaw;
    } else {
      sm.vx = dx / dt; sm.vz = dz / dt; sm.vyaw = dyaw / dt;
    }
  }
  sm.prev = { x: raw.x, z: raw.z, yaw: raw.yaw };
  sm.prevExtrap = Boolean(raw.extrapolated);
  sm.prevBaseT = raw.baseT ?? null;
  return { ...raw, x: raw.x + sm.cx, z: raw.z + sm.cz, yaw: raw.yaw + sm.cyaw };
}

function sampleEgoPose(buf, now, fallback, delayMs = INTERP_DELAY_MS) {
  if (!buf || !buf.length) return { x: fallback.x, y: fallback.y, z: fallback.z, yaw: fallback.yaw };
  const last = buf[buf.length - 1];
  if (buf.length === 1) return last;
  const t = now - delayMs;
  if (t <= buf[0].t) return buf[0];
  if (t >= last.t) {
    // 新帧迟到（链路卡顿）：按最后一帧的速度和转弯率沿圆弧外推（最多 EXTRAP_MAX_MS），之后原地等待。
    // 车速和转弯率在零点几秒内基本不变，外推误差很小；新帧到达后自然接回真实轨迹。
    const prev = buf[buf.length - 2];
    const dt = last.t - prev.t;
    const ext = Math.min(t - last.t, EXTRAP_MAX_MS);
    const yawRate = dt >= 30 ? normalizeAngle(last.yaw - prev.yaw) / dt : 0; // rad/ms；两帧几乎同时到达时无法估计，不转
    const dYaw = yawRate * ext;
    const hasVel = Number.isFinite(last.vx) && Number.isFinite(last.vz);
    let vx = hasVel ? last.vx : (dt >= 30 ? ((last.x - prev.x) / dt) * 1000 : 0);
    let vz = hasVel ? last.vz : (dt >= 30 ? ((last.z - prev.z) / dt) * 1000 : 0);
    // 速度方向转过 dYaw/2，等价于沿圆弧走过的弦
    const c = Math.cos(dYaw / 2);
    const s = Math.sin(dYaw / 2);
    [vx, vz] = [vx * c + vz * s, -vx * s + vz * c];
    return {
      x: last.x + vx * (ext / 1000),
      y: last.y,
      z: last.z + vz * (ext / 1000),
      yaw: last.yaw + dYaw,
      extrapolated: true, // 供 smoothDisplayPose 判断“外推被新数据修正”的时刻
      baseT: last.t,
    };
  }
  for (let i = buf.length - 1; i > 0; i--) {
    const a = buf[i - 1];
    const b = buf[i];
    if (t >= a.t) {
      const hMs = Math.max(1, b.t - a.t);
      const k = (t - a.t) / hMs;
      const h = hMs / 1000;
      // 两帧之间的平均速度（弦）。速度话题不可信（为 0、方向不对）时退回用弦，等价于直线插值，不会走走停停。
      const cx = (b.x - a.x) / h;
      const cz = (b.z - a.z) / h;
      const chord = Math.hypot(cx, cz);
      const tangent = (s) => {
        if (!Number.isFinite(s.vx) || !Number.isFinite(s.vz)) return [cx, cz];
        const sp = Math.hypot(s.vx, s.vz);
        if (chord < 0.05 && sp < 0.05) return [cx, cz];
        const ratio = sp / Math.max(chord, 1e-3);
        const cosA = (s.vx * cx + s.vz * cz) / Math.max(sp * chord, 1e-6);
        return ratio > 0.5 && ratio < 2 && cosA > 0.7 ? [s.vx, s.vz] : [cx, cz];
      };
      const [m0x, m0z] = tangent(a);
      const [m1x, m1z] = tangent(b);
      // 三次 Hermite：端点经过真实定位，切线用速度，降频后转弯是圆滑曲线而不是折线
      const k2 = k * k;
      const k3 = k2 * k;
      const h00 = 2 * k3 - 3 * k2 + 1;
      const h10 = k3 - 2 * k2 + k;
      const h01 = -2 * k3 + 3 * k2;
      const h11 = k3 - k2;
      return {
        x: h00 * a.x + h10 * h * m0x + h01 * b.x + h11 * h * m1x,
        y: a.y + (b.y - a.y) * k,
        z: h00 * a.z + h10 * h * m0z + h01 * b.z + h11 * h * m1z,
        yaw: a.yaw + normalizeAngle(b.yaw - a.yaw) * k,
      };
    }
  }
  return buf[0];
}

// ── 经纬度 → Autoware map 坐标（与 map_projector_info 的 projector_type 对应） ──
const WGS84_A = 6378137;
const WGS84_F = 1 / 298.257223563;

// WGS84 → UTM（Krüger 级数，毫米级精度）
function latLonToUtm(lat, lon, forceZone) {
  const zone = forceZone ?? Math.floor((lon + 180) / 6) + 1;
  const lon0 = ((zone - 1) * 6 - 180 + 3) * Math.PI / 180;
  const k0 = 0.9996;
  const n = WGS84_F / (2 - WGS84_F);
  const A = (WGS84_A / (1 + n)) * (1 + (n * n) / 4 + (n ** 4) / 64);
  const alpha = [n / 2 - (2 * n * n) / 3 + (5 * n ** 3) / 16, (13 * n * n) / 48 - (3 * n ** 3) / 5, (61 * n ** 3) / 240];
  const phi = (lat * Math.PI) / 180;
  const lam = (lon * Math.PI) / 180 - lon0;
  const c = (2 * Math.sqrt(n)) / (1 + n);
  const t = Math.sinh(Math.atanh(Math.sin(phi)) - c * Math.atanh(c * Math.sin(phi)));
  const xi = Math.atan2(t, Math.cos(lam));
  const eta = Math.atanh(Math.sin(lam) / Math.sqrt(1 + t * t));
  let E = eta;
  let N = xi;
  for (let j = 1; j <= 3; j++) {
    E += alpha[j - 1] * Math.cos(2 * j * xi) * Math.sinh(2 * j * eta);
    N += alpha[j - 1] * Math.sin(2 * j * xi) * Math.cosh(2 * j * eta);
  }
  return { e: 500000 + k0 * A * E, n: (lat < 0 ? 10000000 : 0) + k0 * A * N, zone };
}

// WGS84 → 以原点为中心的 ENU（对应 Autoware 的 LocalCartesian）
function latLonToEnu(lat, lon, alt, lat0, lon0, alt0) {
  const e2 = WGS84_F * (2 - WGS84_F);
  const ecef = (la, lo, h) => {
    const p = (la * Math.PI) / 180, l = (lo * Math.PI) / 180;
    const Nr = WGS84_A / Math.sqrt(1 - e2 * Math.sin(p) ** 2);
    return [(Nr + h) * Math.cos(p) * Math.cos(l), (Nr + h) * Math.cos(p) * Math.sin(l), (Nr * (1 - e2) + h) * Math.sin(p)];
  };
  const [x, y, z] = ecef(lat, lon, alt);
  const [x0, y0, z0] = ecef(lat0, lon0, alt0);
  const dx = x - x0, dy = y - y0, dz = z - z0;
  const p0 = (lat0 * Math.PI) / 180, l0 = (lon0 * Math.PI) / 180;
  return {
    x: -Math.sin(l0) * dx + Math.cos(l0) * dy,
    y: -Math.sin(p0) * Math.cos(l0) * dx - Math.sin(p0) * Math.sin(l0) * dy + Math.cos(p0) * dz,
  };
}

// 按投影类型把经纬度换算成 map 坐标 (x 东, y 北)
function projectToMap(lat, lon, alt, proj) {
  switch (proj.type) {
    case "MGRS": {
      const u = latLonToUtm(lat, lon);
      return { x: ((u.e % 100000) + 100000) % 100000, y: ((u.n % 100000) + 100000) % 100000 };
    }
    case "LocalCartesianUTM": {
      const u = latLonToUtm(lat, lon);
      const o = latLonToUtm(proj.originLat, proj.originLon, u.zone);
      return { x: u.e - o.e, y: u.n - o.n };
    }
    case "LocalCartesian":
    default:
      return latLonToEnu(lat, lon, alt || 0, proj.originLat, proj.originLon, proj.originAlt || 0);
  }
}

// UTM → WGS84（Krüger 级数逆算）
function utmToLatLon(E, N, zone, south = false) {
  const k0 = 0.9996;
  const n = WGS84_F / (2 - WGS84_F);
  const A = (WGS84_A / (1 + n)) * (1 + (n * n) / 4 + (n ** 4) / 64);
  const beta = [n / 2 - (2 * n * n) / 3 + (37 * n ** 3) / 96, (n * n) / 48 + (n ** 3) / 15, (17 * n ** 3) / 480];
  const delta = [2 * n - (2 * n * n) / 3 - 2 * n ** 3, (7 * n * n) / 3 - (8 * n ** 3) / 5, (56 * n ** 3) / 15];
  const xi = (N - (south ? 10000000 : 0)) / (k0 * A);
  const eta = (E - 500000) / (k0 * A);
  let xiP = xi;
  let etaP = eta;
  for (let j = 1; j <= 3; j++) {
    xiP -= beta[j - 1] * Math.sin(2 * j * xi) * Math.cosh(2 * j * eta);
    etaP -= beta[j - 1] * Math.cos(2 * j * xi) * Math.sinh(2 * j * eta);
  }
  const chi = Math.asin(Math.sin(xiP) / Math.cosh(etaP));
  let phi = chi;
  for (let j = 1; j <= 3; j++) phi += delta[j - 1] * Math.sin(2 * j * chi);
  const lon0 = ((zone - 1) * 6 - 180 + 3) * Math.PI / 180;
  const lam = lon0 + Math.atan2(Math.sinh(etaP), Math.cos(xiP));
  return { lat: (phi * 180) / Math.PI, lng: (lam * 180) / Math.PI };
}

// ENU（以原点为中心）→ WGS84
function enuToLatLon(x, y, lat0, lon0, alt0) {
  const e2 = WGS84_F * (2 - WGS84_F);
  const p0 = (lat0 * Math.PI) / 180, l0 = (lon0 * Math.PI) / 180;
  const N0 = WGS84_A / Math.sqrt(1 - e2 * Math.sin(p0) ** 2);
  const X0 = (N0 + alt0) * Math.cos(p0) * Math.cos(l0);
  const Y0 = (N0 + alt0) * Math.cos(p0) * Math.sin(l0);
  const Z0 = (N0 * (1 - e2) + alt0) * Math.sin(p0);
  const X = X0 - Math.sin(l0) * x - Math.sin(p0) * Math.cos(l0) * y;
  const Y = Y0 + Math.cos(l0) * x - Math.sin(p0) * Math.sin(l0) * y;
  const Z = Z0 + Math.cos(p0) * y;
  const lon = Math.atan2(Y, X);
  const pr = Math.hypot(X, Y);
  let lat = Math.atan2(Z, pr * (1 - e2));
  for (let k = 0; k < 8; k++) {
    const Nr = WGS84_A / Math.sqrt(1 - e2 * Math.sin(lat) ** 2);
    const h = pr / Math.cos(lat) - Nr;
    lat = Math.atan2(Z, pr * (1 - (e2 * Nr) / (Nr + h)));
  }
  return { lat: (lat * 180) / Math.PI, lng: (lon * 180) / Math.PI };
}

// map 坐标 → 经纬度（projectToMap 的逆运算）
function mapToLatLon(x, y, proj) {
  switch (proj.type) {
    case "MGRS": {
      // MGRS 坐标是 100 km 网格内的偏移，网格取地图原点所在的那一格
      const o = latLonToUtm(proj.originLat, proj.originLon);
      return utmToLatLon(Math.floor(o.e / 100000) * 100000 + x, Math.floor(o.n / 100000) * 100000 + y, o.zone, proj.originLat < 0);
    }
    case "LocalCartesianUTM": {
      const o = latLonToUtm(proj.originLat, proj.originLon);
      return utmToLatLon(o.e + x, o.n + y, o.zone, proj.originLat < 0);
    }
    case "LocalCartesian":
    default:
      return enuToLatLon(x, y, proj.originLat, proj.originLon, proj.originAlt || 0);
  }
}

function formatDuration(totalSec) {
  const s = Math.max(0, Math.floor(totalSec));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = s % 60;
  const pad = (v) => String(v).padStart(2, "0");
  return h > 0 ? `${h}:${pad(m)}:${pad(sec)}` : `${pad(m)}:${pad(sec)}`;
}

function formatDistance(m) {
  if (!Number.isFinite(m)) return { value: "--", unit: "m" };
  if (m >= 1000) return { value: (m / 1000).toFixed(2), unit: "km" };
  return { value: m.toFixed(m < 10 ? 1 : 0), unit: "m" };
}

// rosbridge 地址 → 要尝试的 WebSocket 地址列表："IP"、"IP:端口"、"ws://..."、"wss://..."，缺省协议 ws、端口 9090。
// 先用填写的协议，再试另一种（frp 的 tcp 隧道只能 ws://，https 隧道只能 wss://；网页是 https 时浏览器只允许 wss://）
function vehicleWsUrls(raw) {
  const t = String(raw || "").trim().replace(/\/+$/, "");
  const m = t.match(/^(wss?):\/\/(.+)$/i);
  const rest = m ? m[2] : t;
  const c = rest.lastIndexOf(":");
  const scheme = m ? m[1].toLowerCase() : "ws";
  const host = (c > 0 ? rest.slice(0, c) : rest).trim();
  const port = c > 0 ? rest.slice(c + 1) : "9090";
  if (!host) return [];
  const pageIsHttps = typeof window !== "undefined" && window.location.protocol === "https:";
  // 明确填写 ws:// 或 wss:// 时必须尊重用户配置，不能因为网页是 HTTPS 就把
  // ws:// 静默改成 wss://。普通 rosbridge:9090 没有 TLS，改成 wss 只会让握手失败。
  // HTTPS 页面连接 ws:// 会被浏览器的 Mixed Content 规则拦截，交给界面给出明确提示。
  // 没写协议时约定先尝试普通 rosbridge 的 ws://，与上面的“缺省协议 ws”一致。
  // 即使页面是 HTTPS 也不偷偷改变协议，而是由 blockedByMixedContent 给出真实原因。
  const schemes = m ? [scheme] : ["ws", "wss"];
  return [...new Set(schemes)].map((sc) => ({
    url: `${sc}://${host}:${port}`,
    sc,
    scheme,
    blockedByMixedContent: pageIsHttps && sc === "ws",
  }));
}

// 工控机 ros_udp_pose_relay.py 使用的紧凑定位报文 → 现有 rosbridge 消息结构。
// 紧凑格式可把单帧控制在几百字节，避免普通 ROS JSON 在 UDP 上发生 IP 分片。
function udpPosePacketToRosMessage(packet) {
  if (!packet || packet.v !== 1 || packet.type !== "pose") return null;
  if (!Array.isArray(packet.p) || packet.p.length < 3) return null;
  if (!Array.isArray(packet.q) || packet.q.length < 4) return null;
  if (!Array.isArray(packet.linear) || packet.linear.length < 3) return null;

  const values = [...packet.p.slice(0, 3), ...packet.q.slice(0, 4), ...packet.linear.slice(0, 3)].map(Number);
  if (!values.every(Number.isFinite)) return null;
  const [px, py, pz, qx, qy, qz, qw, vx, vy, vz] = values;
  const sec = Number(packet.stamp?.[0]);
  const nanosec = Number(packet.stamp?.[1]);
  if (!Number.isFinite(sec) || !Number.isFinite(nanosec)) return null;

  return {
    op: "publish",
    topic: LOC_TOPIC,
    msg: {
      header: { stamp: { sec, nanosec }, frame_id: String(packet.frame || "map") },
      pose: { pose: { position: { x: px, y: py, z: pz }, orientation: { x: qx, y: qy, z: qz, w: qw } } },
      twist: { twist: { linear: { x: vx, y: vy, z: vz } } },
    },
  };
}

// 车辆通道测速：建立 WebSocket 后短暂订阅主要 ROS 话题，返回实际接收速度。
// 这里测到的是当前业务数据的实收速率，不是运营商线路的理论带宽。
function probeVehicleChannel(addr, signal) {
  const urls = vehicleWsUrls(addr);
  const tryOne = (url) => new Promise((resolve) => {
    const t0 = performance.now();
    let done = false;
    let opened = false;
    let rxBytes = 0;
    let sampleStartedAt = 0;
    let sampleTimer = null;
    let ws;
    let connectTimer = null;
    const onAbort = () => finish({ ok: false, url, cancelled: true, reason: "已停止" });
    const finish = (r) => {
      if (done) return;
      done = true;
      clearTimeout(connectTimer);
      clearTimeout(sampleTimer);
      signal?.removeEventListener("abort", onAbort);
      try { ws && ws.close(); } catch { /* 忽略 */ }
      resolve(r);
    };
    signal?.addEventListener("abort", onAbort, { once: true });
    if (signal?.aborted) { onAbort(); return; }
    connectTimer = setTimeout(() => finish({ ok: false, url, reason: "超时" }), WS_CONNECT_TIMEOUT_MS);
    try {
      ws = new WebSocket(url);
      ws.onmessage = (e) => {
        if (!sampleStartedAt) return;
        rxBytes += typeof e.data === "string" ? e.data.length : (e.data?.byteLength || e.data?.size || 0);
      };
      ws.onopen = () => {
        opened = true;
        clearTimeout(connectTimer);
        const connectionMs = performance.now() - t0;
        sampleStartedAt = performance.now();
        const subscriptions = [
          { topic: LOC_TOPIC, throttle_rate: SUB_POSE_MS },
          { topic: ROUTE_STATE_TOPIC, throttle_rate: 500 },
          { topic: CONTROL_TOPIC, throttle_rate: SUB_CONTROL_MS },
          { topic: GNSS_TOPIC, throttle_rate: SUB_GNSS_MS },
        ];
        try {
          subscriptions.forEach((sub, i) => ws.send(JSON.stringify({
            op: "subscribe",
            id: `vehicle-speed-probe-${i}`,
            topic: sub.topic,
            throttle_rate: sub.throttle_rate,
            queue_length: 1,
          })));
        } catch (err) {
          finish({ ok: false, url, reason: err?.message || "订阅失败" });
          return;
        }
        sampleTimer = setTimeout(() => {
          const elapsed = Math.max(0.001, (performance.now() - sampleStartedAt) / 1000);
          finish({ ok: true, url, ms: connectionMs, kbps: (rxBytes * 8) / elapsed / 1000 });
        }, VEHICLE_PROBE_SAMPLE_MS);
      };
      ws.onerror = () => finish({ ok: false, url, reason: "连接被拒绝或无法访问" });
      ws.onclose = () => finish({ ok: false, url, reason: opened ? "测速期间连接被关闭" : "连接被关闭" });
    } catch (err) {
      finish({ ok: false, url, reason: err?.message || "地址无效" });
    }
  });
  return (async () => {
    let last = { ok: false, reason: "地址无效" };
    for (const u of urls) {
      last = await tryOne(u.url);
      if (last.ok || last.cancelled) return last;
    }
    return last;
  })();
}

// 车辆通道：主通道未连上时才自动测试备用通道；主通道连上后立即取消重数据测速。
function useVehicleChannelProbe(channels, currentUrl, autoProbeEnabled) {
  const [results, setResults] = useState({});
  const [probing, setProbing] = useState({});
  const busyRef = useRef(new Set());
  const probeOne = useCallback(async (c, options = {}) => {
    const signal = options?.signal;
    if (busyRef.current.has(c.name)) return;
    busyRef.current.add(c.name);
    setProbing((p) => ({ ...p, [c.name]: true }));
    try {
      const r = await probeVehicleChannel(c.url, signal);
      if (r.cancelled || signal?.aborted) return;
      console.info(r.ok ? `🔌 车辆通道「${c.name}」接收 ${formatReceiveSpeed(r.kbps)}（${r.url}，建连 ${r.ms.toFixed(0)} ms）` : `🔌 车辆通道「${c.name}」不通：${r.reason}`);
      setResults((m) => ({ ...m, [c.name]: { ...r, at: Date.now() } }));
    } finally {
      busyRef.current.delete(c.name);
      setProbing((p) => ({ ...p, [c.name]: false }));
    }
  }, []);
  useEffect(() => {
    if (!autoProbeEnabled || channels.length < 2) return;
    const controller = new AbortController();
    const run = () => channels.filter((c) => c.url !== currentUrl).forEach((c) => probeOne(c, { signal: controller.signal }));
    run();
    const id = setInterval(run, VEHICLE_PROBE_MS);
    return () => {
      clearInterval(id);
      controller.abort();
    };
  }, [autoProbeEnabled, channels, currentUrl, probeOne]);
  return { results, probing, probeOne };
}

// ── 通道下拉菜单（车辆连接 / 相机共用）：触发按钮 + 弹出列表；点外面或按 Esc 关闭 ──
function ChannelDropdown({ className = "", trigger, triggerTitle, title, rows, onSelect, onRefresh, refreshing, align = "left", footer }) {
  const [open, setOpen] = useState(false);
  const ref = useRef(null);
  useEffect(() => {
    if (!open) return;
    const onDown = (e) => { if (ref.current && !ref.current.contains(e.target)) setOpen(false); };
    const onKey = (e) => { if (e.key === "Escape") setOpen(false); };
    document.addEventListener("mousedown", onDown);
    window.addEventListener("keydown", onKey);
    return () => {
      document.removeEventListener("mousedown", onDown);
      window.removeEventListener("keydown", onKey);
    };
  }, [open]);
  const close = () => setOpen(false);
  return (
    <div className={`dt-dd ${className}`} ref={ref}>
      <button type="button" className={`dt-dd-trigger ${open ? "open" : ""}`} title={triggerTitle} onClick={() => setOpen((v) => !v)}>
        {trigger}
        <span className="dt-dd-caret">▾</span>
      </button>
      {open && (
        <div className={`dt-dd-menu dt-glass ${align}`}>
          <div className="dt-dd-head">
            <span>{title}</span>
            <button type="button" onClick={onRefresh} disabled={refreshing} title="立即重新测试没在用的通道">{refreshing ? "测试中…" : "⟳ 重新测试"}</button>
          </div>
          {rows.map((r) => (
            <button
              type="button"
              key={r.key}
              className={`dt-dd-row ${r.current ? "cur" : ""}`}
              title={r.tip}
              onClick={() => { if (!r.current) onSelect(r.key); close(); }}
            >
              <i className={`dt-cam-dot ${r.tone}`} />
              <span className="dt-dd-main">
                <b>{r.name}{r.current && <em>使用中</em>}</b>
                <small className="dt-num">{r.url}</small>
              </span>
              <span className="dt-dd-stat">
                <b className={`dt-num ${r.tone}`}>{r.text}</b>
                {r.sub && <small>{r.sub}</small>}
              </span>
            </button>
          ))}
          {footer && footer(close)}
        </div>
      )}
    </div>
  );
}

const timeOf = (t) => new Date(t).toLocaleTimeString("zh-CN", { hour12: false, timeZone: "Asia/Shanghai" });

// 车辆通道右侧统一显示接收速度：数值越大，当前收到的数据越多。
function formatReceiveSpeed(kbps) {
  if (Number.isFinite(kbps)) {
    const safeKbps = Math.max(0, kbps);
    if (safeKbps >= 1000) return `${(safeKbps / 1000).toFixed(2)} Mbit/s`;
    return `${safeKbps.toFixed(safeKbps >= 100 ? 0 : 1)} Kbit/s`;
  }
  return "--";
}

// 相机通道继续使用原有的紧凑指标：有码率时显示码率，否则显示延迟。
function channelMetric(kbps, latencyMs) {
  if (Number.isFinite(kbps) && kbps > 1000) return `${(kbps / 1000).toFixed(1)}M`;
  if (Number.isFinite(latencyMs)) return `${latencyMs.toFixed(0)} ms`;
  return "--";
}

// ── 顶栏车辆连接（信号格）──
// 只表达“连没连上”：连接中 / 已连接（满格）/ 未连接。不给链路打分——
// 实测链路质量需要额外请求，且受 rosbridge 处理速度影响，容易误判；网速等数字放在悬停提示和下拉里。
function linkQuality({ connected, connectingLong }) {
  if (connected) return { bars: 4, tone: "ok", state: "已连接" };
  return connectingLong ? { bars: 1, tone: "bad", state: "未连接" } : { bars: 0, tone: "wait", state: "连接中…" };
}
function SignalPill({ quality, label }) {
  return (
    <span className={`dt-link ${quality.tone}`}>
      <span className="dt-sig" aria-hidden="true">
        {[0, 1, 2, 3].map((i) => <i key={i} className={i < quality.bars ? "on" : ""} />)}
      </span>
      {label}
    </span>
  );
}

// ── 车辆连接下拉：放在顶栏，信号格 + 通道名；展开后列出所有通道的速度、延迟，点击手动切换 ──
function VehicleChannelMenu({ channels, currentUrl, connected, connectingLong, activeUrl, liveKbps, liveMs, quality, jitterMs, results, probing, onSelect, onProbe, onCustom }) {
  const cur = channels.find((c) => c.url === currentUrl);
  const curName = cur ? cur.name : "自定义";
  const others = channels.filter((c) => c.url !== currentUrl);
  const liveMetric = formatReceiveSpeed(liveKbps);
  const liveReceive = `接收 ${liveMetric}`;
  const jitterText = Number.isFinite(jitterMs) ? `定位抖动 ${jitterMs.toFixed(0)} ms · ` : "";
  const liveConnection = `${jitterText}建连 ${Number.isFinite(liveMs) ? `${liveMs.toFixed(0)} ms` : "--"}`;
  const liveDetail = `接收 ${liveMetric} · ${liveConnection}`;
  const rows = channels.map((c) => {
    const isCur = c.url === currentUrl;
    if (isCur) {
      return connected
        ? { key: c.url, name: c.name, url: c.url, current: true, tone: "ok", text: liveReceive, sub: liveConnection, tip: `正在使用 · ${activeUrl} · ${liveDetail}` }
        : { key: c.url, name: c.name, url: c.url, current: true, tone: "wait", text: "连接中", sub: activeUrl ? `尝试 ${activeUrl}` : "", tip: "正在使用 · 正在连接" };
    }
    const r = results[c.name];
    const busy = probing[c.name];
    if (!r) return { key: c.url, name: c.name, url: c.url, tone: "wait", text: busy ? "测试中" : "--", sub: busy ? "" : "尚未测试", tip: "点击切换到这个通道" };
    if (r.ok) return { key: c.url, name: c.name, url: c.url, tone: "ok", text: `接收 ${formatReceiveSpeed(r.kbps)}`, sub: `建连 ${r.ms.toFixed(0)} ms${busy ? " · 重测中" : ""}`, tip: `实收速度 ${formatReceiveSpeed(r.kbps)} · 建连 ${r.ms.toFixed(0)} ms · ${timeOf(r.at)}（${r.url}）· 点击切换` };
    return { key: c.url, name: c.name, url: c.url, tone: "bad", text: "不通", sub: `${r.reason} · ${timeOf(r.at)}`, tip: `不通：${r.reason}` };
  });
  if (!cur) {
    rows.unshift(connected
      ? { key: currentUrl, name: "自定义", url: currentUrl, current: true, tone: "ok", text: liveReceive, sub: liveConnection, tip: `${activeUrl} · ${liveDetail}` }
      : { key: currentUrl, name: "自定义", url: currentUrl, current: true, tone: "wait", text: "连接中", sub: activeUrl ? `尝试 ${activeUrl}` : "", tip: "正在连接" });
  }
  return (
    <ChannelDropdown
      className="dt-dd-vehicle"
      triggerTitle={`${curName}：${quality.state}${connected ? ` · ${liveDetail}` : ""}（点击查看详情或切换通道）`}
      trigger={<SignalPill quality={quality} label={connected ? curName : quality.state} />}
      title="车辆连接通道 · 接收速度 / 建连时间"
      rows={rows}
      onSelect={onSelect}
      refreshing={others.some((c) => probing[c.name])}
      onRefresh={() => others.forEach(onProbe)}
      footer={(close) => <CustomAddress initial={currentUrl} onApply={(v) => { onCustom(v); close(); }} />}
    />
  );
}

function CustomAddress({ initial, onApply }) {
  const [v, setV] = useState(initial);
  const apply = () => { if (v.trim()) onApply(v.trim()); };
  return (
    <div className="dt-dd-foot">
      <input
        className="dt-input dt-num"
        value={v}
        placeholder="其他地址：IP / IP:端口 / ws(s)://…"
        onChange={(e) => setV(e.target.value)}
        onKeyDown={(e) => { if (e.key === "Enter") apply(); e.stopPropagation(); }}
      />
      <button type="button" className="dt-btn ok" onClick={apply}>连接</button>
    </div>
  );
}

// ── 云乐底盘直连：浏览器 WebSocket ↔ 本机 yunle_can_gateway ↔ can0/can1 ──
// 网关负责 CAN 编解码、10 ms 周期、独占控制权和全部安全联锁；这里仅发送驾驶意图。
function useDirectCanGateway(config) {
  const wsRef = useRef(null);
  const ccuRef = useRef({ value: null, at: 0 });
  const [state, setState] = useState({
    connected: false,
    connectionError: "",
    ccuStatus: null,
    ccuFeedbackAt: 0,
    bmsStatus: null,
    bmsFeedbackAt: 0,
    status: null,
    manualError: "",
  });

  useEffect(() => {
    if (!config.enabled) return undefined;
    let disposed = false;
    let reconnectTimer = null;
    let ws = null;
    let lastCcuUiAt = 0;

    const connect = () => {
      if (disposed) return;
      try {
        ws = new WebSocket(config.url);
      } catch (err) {
        setState((s) => ({ ...s, connected: false, connectionError: err?.message || "CAN 网关地址无效" }));
        reconnectTimer = setTimeout(connect, 2000);
        return;
      }
      wsRef.current = ws;
      ws.onopen = () => {
        setState((s) => ({ ...s, connectionError: "" }));
        if (config.token) ws.send(JSON.stringify({ type: "auth", token: config.token }));
      };
      ws.onmessage = (event) => {
        let msg;
        try { msg = JSON.parse(event.data); } catch { return; }
        if (msg.type === "hello") {
          if (!msg.authentication_required) setState((s) => ({ ...s, connected: true, connectionError: "" }));
        } else if (msg.type === "auth") {
          setState((s) => ({ ...s, connected: Boolean(msg.ok), connectionError: msg.ok ? "" : "CAN 网关认证失败" }));
        } else if (msg.type === "ccu_status") {
          const at = Date.now();
          ccuRef.current = { value: msg, at };
          if (at - lastCcuUiAt >= UI_EGO_MS) {
            lastCcuUiAt = at;
            setState((s) => ({ ...s, ccuStatus: msg, ccuFeedbackAt: at }));
          }
        } else if (msg.type === "bms_status") {
          setState((s) => ({ ...s, bmsStatus: msg, bmsFeedbackAt: Date.now() }));
        } else if (msg.type === "gateway_status") {
          setState((s) => ({ ...s, status: msg }));
        } else if (msg.type === "control_claim" && !msg.ok) {
          setState((s) => ({ ...s, manualError: msg.message || "底盘正在被另一客户端控制" }));
        } else if (msg.type === "error") {
          const text = msg.code === "unauthorized" ? "CAN 网关认证失败"
            : msg.code === "not_control_owner" ? "尚未取得底盘控制权"
            : msg.message || `CAN 网关错误：${msg.code || "unknown"}`;
          setState((s) => ({ ...s, manualError: text }));
        }
      };
      ws.onerror = () => {
        if (!disposed) setState((s) => ({ ...s, connected: false, connectionError: `无法连接 CAN 网关 ${config.url}` }));
      };
      ws.onclose = () => {
        if (wsRef.current === ws) wsRef.current = null;
        if (disposed) return;
        setState((s) => ({ ...s, connected: false, manualError: s.connected ? "底盘 CAN 网关连接中断" : s.manualError }));
        reconnectTimer = setTimeout(connect, 1000);
      };
    };
    connect();
    return () => {
      disposed = true;
      clearTimeout(reconnectTimer);
      if (wsRef.current === ws) wsRef.current = null;
      if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) ws.close();
    };
  }, [config.enabled, config.url, config.token]);

  const requestRemoteMode = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN || !state.connected) {
      setState((s) => ({ ...s, manualError: s.connectionError || "未连接底盘 CAN 网关" }));
      return false;
    }
    const feedback = ccuRef.current;
    if (!feedback.value || Date.now() - feedback.at > TELEOP.feedbackTimeoutMs) {
      setState((s) => ({ ...s, manualError: "没有收到 0x51 底盘反馈，请检查 can0/can1 与网关" }));
      return false;
    }
    if (!feedback.value.ccu_drive_mode_shift_button) {
      setState((s) => ({ ...s, manualError: "请先闭合底盘自动驾驶切换按钮" }));
      return false;
    }
    ws.send(JSON.stringify({ type: "claim_control" }));
    setState((s) => ({ ...s, manualError: "" }));
    return true;
  }, [state.connected]);

  const publishCmdVel = useCallback((linearX, angularZ, forceNeutral = false) => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN || !state.connected) return false;
    ws.send(JSON.stringify({
      type: "command",
      speed_mps: Number(linearX) || 0,
      steering_rad: Number(angularZ) || 0,
      force_neutral: Boolean(forceNeutral),
    }));
    return true;
  }, [state.connected]);

  const releaseControl = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify({ type: "command", speed_mps: 0, steering_rad: 0, force_neutral: true }));
    ws.send(JSON.stringify({ type: "release_control" }));
  }, []);

  return { ...state, requestRemoteMode, publishCmdVel, releaseControl };
}

function useSimulation(serverIp) {
  const wsRef = useRef(null);
  const remoteGearRef = useRef(1); // Autoware GearCommand：N=1, D=2, R=20
  const chassisStatusRef = useRef({ value: null, at: 0 });
  const lastDataAtRef = useRef(0);
  const [state, setState] = useState({
    ego: { x: 0, y: 0, z: 0, yaw: 0, speed: 0, steering: 0, lat: MAP_ORIGIN.lat, lng: MAP_ORIGIN.lng },
    objects: [],      // 障碍物（Three 坐标，footprint 已转为世界朝向偏移）
    objectsAt: 0,     // 最近一次收到障碍物消息的时间
    path: [],         // 局部规划路径 (3D 笛卡尔坐标，供 3D 场景绘制局部引导带)
    globalPath: [],   // 全局/行为规划路径（经纬度，仅供右侧卫星地图绘制）
    startPoint: null, // 起点 { lat, lng }
    goalPoint: null,  // 终点 { lat, lng }
    startPose: null,  // 起点 Three.js 位姿 { x, z, yaw }（3D 起点标牌）
    globalPath3d: [], // Autoware 规划路径的 3D 点（Three 坐标，含高程），用于 3D 路线条带
    startStatus: { state: "idle", msg: "" }, // 设起点进度：idle / pending / ok / sent / failed
    goalPose: null,   // 终点 Three.js 位姿 { x, z, yaw }
    operationMode: null,
    chassisStatus: null,
    chassisFeedbackAt: 0,
    bmsStatus: null,
    bmsFeedbackAt: 0,
    manualError: "",
    placedObstacles: [], // 已放置的仿真障碍物（来自工控机节点的列表，ROS 坐标）
    autoRequestStatus: "idle",
    autoFail: null,   // 自动驾驶启动失败：{ msg, at }
    stopStatus: { state: "idle", msg: "", at: 0 },  // 停车：idle / pending / ok / failed
    routeClear: { state: "idle", msg: "", at: 0 },  // 取消路线：idle / pending / ok / failed
    routeState: null, // Autoware 路线状态：1 未设置 / 2 已设置 / 3 已到达 / 4 切换中
    pathAt: 0,        // 最近一次收到控制轨迹的时间（判断轨迹是否过期）
  });

  const [connected, setConnected] = useState(false);
  const [connectionMs, setConnectionMs] = useState(null);
  const [connectionError, setConnectionError] = useState("");
  const trafficRef = useRef({ rxBytes: 0, lastRx: 0, lastAt: performance.now() });
  const [rxKbps, setRxKbps] = useState(0);
  // 当前正在尝试 / 已连上的 rosbridge 地址（顶栏悬停可见）
  const [activeUrl, setActiveUrl] = useState("");
  const [dataAgeSec, setDataAgeSec] = useState(null);
  // 各关键话题最近到达时间（仅用于界面显示在线状态）
  const topicStatsRef = useRef({});
  const udpDiagnosticsRef = useRef(createUdpDiagnosticsStore());
  const [udpDiagnostics, setUdpDiagnostics] = useState(() => snapshotUdpDiagnostics(udpDiagnosticsRef.current));
  const resetUdpDiagnostics = useCallback(() => {
    const fresh = createUdpDiagnosticsStore();
    udpDiagnosticsRef.current = fresh;
    setUdpDiagnostics(snapshotUdpDiagnostics(fresh));
  }, []);
  // 原始定位样本（不经过 React），供 3D 画面平滑插值
  const poseBufferRef = useRef([]);
  // 定位时间轴：时钟偏移、到达抖动统计、建议回看时长
  const poseTimelineRef = useRef(createPoseTimeline());
  // 网页本地设终点的时刻：之后 LOCAL_GOAL_GRACE_MS 内不让路线接口里的旧终点覆盖新终点
  const localGoalAtRef = useRef(0);
  // 设起点成功后，下一帧定位直接瞬移到位（不做平滑过渡）
  const teleportNextRef = useRef(false);
  // 设起点：服务不存在时回退发布 /initialpose 的备用消息
  const pendingInitRef = useRef(null);
  const startStatusTimerRef = useRef(null);
  // Autoware 定位初始化状态（/api/localization/initialization_state）
  const locInitStateRef = useRef(null);
  // 当前设起点/定位初始化进度（给消息回调读取，不触发渲染）
  const startStatusRef = useRef({ state: "idle", kind: "start" });
  const setStartStatus = useCallback((state, msg = "", kind = "start") => {
    clearTimeout(startStatusTimerRef.current);
    startStatusRef.current = { state, kind, at: Date.now() };
    setState((s) => ({ ...s, startStatus: { state, msg, kind } }));
    if (state === "pending" || state === "verifying") {
      const timeoutMs = kind === "gnss" ? GNSS_INIT_TIMEOUT_MS : START_TIMEOUT_MS;
      const failMsg = kind === "gnss"
        ? "超时"
        : "车辆无响应";
      if (kind === "gnss") console.warn(`⚠️ ${Math.round(timeoutMs / 1000)} 秒内未完成定位初始化：请检查 NDT 是否运行、点云地图是否加载，并确认车辆在卫星位置附近`);
      startStatusTimerRef.current = setTimeout(() => {
        startStatusRef.current = { state: "failed", kind, at: Date.now() };
        setState((s) => (s.startStatus.state === "pending" || s.startStatus.state === "verifying" ? { ...s, startStatus: { state: "failed", msg: failMsg, kind } } : s));
        startStatusTimerRef.current = setTimeout(() => setState((s) => ({ ...s, startStatus: { state: "idle", msg: "" } })), 10000);
      }, timeoutMs);
    } else if (state !== "idle") {
      startStatusTimerRef.current = setTimeout(() => setState((s) => ({ ...s, startStatus: { state: "idle", msg: "" } })), state === "failed" || state === "warn" ? 10000 : kind === "gnss" ? 5000 : 2500);
    }
  }, []);
  const [topicHealth, setTopicHealth] = useState({});
  // 定位初始化状态（1 未初始化 / 2 初始化中 / 3 已初始化），系统状态页显示
  const [locInitState, setLocInitState] = useState(null);
  // 最近一次控制指令（转向角 rad、目标速度 m/s）
  const [controlCmd, setControlCmd] = useState(null);
  // 最新一帧 GNSS（顶栏卫星状态）与定位初始化后读取小车位置的计时
  const gnssRef = useRef(null);
  const gnssCheckRef = useRef(null);
  const [gnss, setGnss] = useState(null);
  // 仿真障碍物节点：最近一次收到列表的时间、列表内容（用于去重）
  const obstacleListAtRef = useRef(0);
  const obstacleListKeyRef = useRef("");
  const [obstacleNodeOnline, setObstacleNodeOnline] = useState(false);

  useEffect(() => {
    const t = setInterval(() => {
      const now = Date.now();
      const perfNow = performance.now();
      const traffic = trafficRef.current;
      const elapsed = Math.max(0.001, (perfNow - traffic.lastAt) / 1000);
      setRxKbps(((traffic.rxBytes - traffic.lastRx) * 8) / elapsed / 1000);
      traffic.lastRx = traffic.rxBytes;
      traffic.lastAt = perfNow;
      if (lastDataAtRef.current > 0) {
        setDataAgeSec(Math.max(0, Math.round((now - lastDataAtRef.current) / 1000)));
      }
      const health = {};
      HEALTH_TOPICS.forEach((topic) => {
        const st = topicStatsRef.current[topic];
        if (!st || !st.lastAt) {
          health[topic] = { ageMs: null, hz: 0 };
        } else {
          st.stamps = st.stamps.filter((ts) => now - ts <= 2000);
          health[topic] = { ageMs: now - st.lastAt, hz: Math.round((st.stamps.length / 2) * 10) / 10 };
        }
      });
      setTopicHealth(health);
      setUdpDiagnostics(snapshotUdpDiagnostics(udpDiagnosticsRef.current, now));
      setObstacleNodeOnline(obstacleListAtRef.current > 0 && now - obstacleListAtRef.current < OBSTACLE_NODE_TIMEOUT_MS);
    }, 1000);
    return () => clearInterval(t);
  }, []);

  useEffect(() => {
    const freshUdpDiagnostics = createUdpDiagnosticsStore();
    udpDiagnosticsRef.current = freshUdpDiagnostics;
    setUdpDiagnostics(snapshotUdpDiagnostics(freshUdpDiagnostics));
    let ego = { x: 0, y: 0, z: 0, yaw: 0, speed: 0, steering: 0, lat: MAP_ORIGIN.lat, lng: MAP_ORIGIN.lng };
    // 流畅模式：定位消息只保留最新一帧，界面最多每 UI_EGO_MS 刷新一次（3D 车辆另有插值，依旧平滑）
    let egoFlushTimer = null;
    let egoFlushAt = 0;
    const flushEgo = () => {
      egoFlushTimer = null;
      egoFlushAt = Date.now();
      const e = ego;
      setState((s) => ({ ...s, ego: e, startPoint: s.startPoint || { lat: e.lat, lng: e.lng } }));
    };
    const scheduleEgoFlush = () => {
      if (egoFlushTimer) return;
      egoFlushTimer = setTimeout(flushEgo, Math.max(0, UI_EGO_MS - (Date.now() - egoFlushAt)));
    };

    // 只连当前选中的这一个地址（不会自动换到其他通道）；同一地址先用填写的协议，连不上再试另一种
    const attempts = vehicleWsUrls(serverIp).map((u) => ({ ...u, addr: serverIp.trim() }));
    let attemptIdx = 0;
    let wsUrl = attempts[0]?.url || "";
    let disposed = false;
    let reconnectTimer = null;
    let openTimer = null;
    let watchdogTimer = null;
    let gotAnyData = false;
    let lastDataAt = 0;
    let idleCloseCount = 0;
    let poseWs = null;
    let poseReconnectTimer = null;
    let udpFallbackTimer = null;
    let udpMessageUnsubscribe = null;
    let udpErrorUnsubscribe = null;
    let lastUdpPoseAt = 0;
    let udpEverReceived = false;
    const udpHeavyAt = { objects: 0, trajectory: 0, path: 0 };
    const udpHeavyStampMs = { objects: 0, trajectory: 0, path: 0 };
    const udpHeavyEverReceived = { objects: false, trajectory: false, path: false };
    const udpHeavyFrames = new Map();
    let lastPoseStampMs = 0;
    poseTimelineRef.current = createPoseTimeline(); // 换车辆地址时重新统计时钟偏移与抖动
    // 控制回显 / 底盘反馈：存在闭包变量里，不每帧 setState，避免整页 React 高频重绘抢 3D 渲染的主线程
    let lastControlUiAt = 0;
    let lastWebRouteKey = "";
    // ── 路线 / 终点同步（工控机、RViz、命令行、网页设的终点都走这里） ──
    const applyRouteGoal = (x, y, yaw) => {
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      // 网页刚设了新终点：Autoware 换上新路线前，路线接口还在报旧终点，这段时间不用旧终点覆盖
      if (Date.now() - localGoalAtRef.current < LOCAL_GOAL_GRACE_MS) return;
      const gps = cartesianToLatLon(x, y, MAP_ORIGIN.lat, MAP_ORIGIN.lng);
      setState((s) => {
        const g = s.goalPose;
        if (g && Math.abs(g.z - x) < 0.01 && Math.abs(g.x - y) < 0.01 && Math.abs(normalizeAngle(g.yaw - yaw)) < 0.01) return s;
        return { ...s, goalPoint: { lat: gps.lat, lng: gps.lng }, goalPose: { x: y, z: x, yaw } };
      });
    };
    const applyRouteCleared = () => {
      if (Date.now() - localGoalAtRef.current < LOCAL_GOAL_GRACE_MS) return;
      setState((s) => ({ ...s, goalPoint: null, goalPose: null, globalPath: [], globalPath3d: [], path: [] }));
    };
    const applyRouteState = (st) => {
      if (!Number.isFinite(st)) return;
      setState((s) => {
        const prev = s.routeState;
        if (prev === st) return s;
        // 从“有路线”变成“未设置”（在别处取消了路线）：清掉终点和路线显示
        if (st === 1 && (prev === 2 || prev === 3 || prev === 4) && Date.now() - localGoalAtRef.current >= LOCAL_GOAL_GRACE_MS) {
          return { ...s, routeState: st, goalPoint: null, goalPose: null, globalPath: [], globalPath3d: [], path: [] };
        }
        return { ...s, routeState: st };
      });
    };
    let lastChassisUiAt = 0;
    let chassisFreshAt = 0;

    // 定位走独立 WebSocket，避免感知/轨迹的大 JSON 消息在同一 TCP 流中阻塞小车位姿。
    const handleLocalizationMessage = (msg, channel = "main", bytes = 0) => {
      const pose = msg?.msg?.pose?.pose;
      const twist = msg?.msg?.twist?.twist;
      if (!pose?.position || !pose?.orientation || !twist?.linear) return;

      const now = Date.now();
      gotAnyData = true;
      lastDataAt = now;
      lastDataAtRef.current = now;
      idleCloseCount = 0;

      const pos = pose.position;
      const q = pose.orientation;
      const rosYaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      const v = twist.linear;
      const gps = cartesianToLatLon(pos.x, pos.y, MAP_ORIGIN.lat, MAP_ORIGIN.lng);

      const buf = poseBufferRef.current;
      const arrival = performance.now();
      const st = msg.msg.header?.stamp;
      const stampMs = st ? (st.sec ?? st.secs ?? 0) * 1000 + (st.nanosec ?? st.nsecs ?? 0) / 1e6 : 0;
      netLog.push(LOC_TOPIC, channel, stampMs, arrival, bytes); // ?netlog=1 时记录（去重前，两个通道都记）
      // 独立通道与业务通道都可提供定位作为容灾；按 ROS stamp 只接受最先到达的那份。
      if (stampMs > 0 && lastPoseStampMs > 0 && stampMs <= lastPoseStampMs) {
        // ROS 时钟重置时允许大幅回跳，普通的重复/乱序帧直接丢弃。
        if (lastPoseStampMs - stampMs < 10000) return;
      }
      if (stampMs > 0) lastPoseStampMs = stampMs;
      // 系统状态页的定位频率：只统计去重后真正采用的帧（两个通道收到的同一帧只算一次）
      const stat = topicStatsRef.current[LOC_TOPIC] || (topicStatsRef.current[LOC_TOPIC] = { lastAt: 0, stamps: [] });
      stat.lastAt = now;
      stat.stamps.push(now);
      if (stat.stamps.length > 200) stat.stamps.shift();
      const prevSample = buf[buf.length - 1];
      // ── 用 ROS 时间戳排时间轴，而不是用到达时刻 ──
      // 网络抖动时帧会“几帧挤在一起到、然后空一段”，按到达时刻插值，车会忽快忽慢、一顿一顿。
      // 按 stamp 排列则样本间隔恒定，同时统计到达抖动，自适应回看时长（见 poseTimelinePush）。
      let sampleT = poseTimelinePush(poseTimelineRef.current, stampMs, arrival);
      if (prevSample && sampleT <= prevSample.t) sampleT = prevSample.t + 1;
      // 车体速度（base_link 下 x 前 y 左）转到 map，再转 Three 坐标（x=ROS.y, z=ROS.x），供曲线插值用切线
      const vxb = Number(v.x) || 0;
      const vyb = Number(v.y) || 0;
      const cy = Math.cos(rosYaw);
      const sy = Math.sin(rosYaw);
      const sample = {
        t: sampleT, arrival, stamp: stampMs,
        x: pos.y, y: Number.isFinite(pos.z) ? pos.z : 0, z: pos.x, yaw: rosYaw,
        vx: sy * vxb + cy * vyb, // Three x 方向速度（m/s）
        vz: cy * vxb - sy * vyb, // Three z 方向速度（m/s）
      };
      if (teleportNextRef.current || (prevSample && Math.hypot(sample.x - prevSample.x, sample.z - prevSample.z) > POSE_JUMP_M)) {
        buf.length = 0;
        teleportNextRef.current = false;
      }
      buf.push(sample);
      if (buf.length > 30) buf.shift();

      ego = {
        ...ego,
        // ROS (x 前, y 左, z 上) -> Three.js (x, y, z) = (ROS.y, ROS.z, ROS.x)
        x: pos.y,
        y: Number.isFinite(pos.z) ? pos.z : 0,
        z: pos.x,
        yaw: rosYaw,
        // 速度只用一个来源：底盘反馈新鲜时以底盘为准，否则用定位速度（两者交替写会让仪表盘来回跳）
        speed: Date.now() - chassisFreshAt < 1000 ? ego.speed : Math.sqrt(v.x * v.x + v.y * v.y),
        lat: gps.lat,
        lng: gps.lng,
      };
      scheduleEgoFlush();

      // 实车定位初始化完成后，取独立定位通道上的最新位置作为起点。
      const chk = gnssCheckRef.current;
      if (chk && chk.checkAfter && arrival >= chk.checkAfter) {
        gnssCheckRef.current = null;
        setState((s) => ({
          ...s,
          startPoint: { lat: gps.lat, lng: gps.lng },
          startPose: { x: pos.y, z: pos.x, yaw: rosYaw },
        }));
        console.info(`📍 工控机定位初始化完成，小车位置 map (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, z=${Number(pos.z).toFixed(2)})，朝向 ${((rosYaw * 180) / Math.PI).toFixed(1)}°`);
        setStartStatus("ok", `小车位置 (${pos.x.toFixed(1)}, ${pos.y.toFixed(1)})，朝向 ${((rosYaw * 180) / Math.PI).toFixed(0)}°（来自工控机）`, "gnss");
      }
    };

    const stopPoseSocket = () => {
      clearTimeout(poseReconnectTimer);
      poseReconnectTimer = null;
      const pws = poseWs;
      poseWs = null;
      if (pws && (pws.readyState === WebSocket.OPEN || pws.readyState === WebSocket.CONNECTING)) pws.close();
    };

    const startPoseSocket = (url, ownerWs) => {
      stopPoseSocket();
      if (disposed || !url) return;
      const pws = new WebSocket(url);
      poseWs = pws;
      let opened = false;
      const timeout = setTimeout(() => {
        if (!opened && pws.readyState === WebSocket.CONNECTING) pws.close();
      }, WS_CONNECT_TIMEOUT_MS);
      pws.onopen = () => {
        opened = true;
        clearTimeout(timeout);
        pws.send(JSON.stringify({ op: "subscribe", id: "realtime-pose", topic: LOC_TOPIC, type: "nav_msgs/msg/Odometry", throttle_rate: SUB_POSE_MS, queue_length: 1 }));
        console.info("⚡ 实时定位独立通道已连接");
      };
      pws.onmessage = (e) => {
        trafficRef.current.rxBytes += typeof e.data === "string" ? e.data.length : (e.data?.byteLength || e.data?.size || 0);
        if (typeof e.data !== "string") return;
        try {
          const msg = JSON.parse(e.data);
          // 每一帧定位都进缓冲区（解析 1 KB JSON 的开销可以忽略）。
          // 以前按渲染帧合并、只留最新一帧：网络把几帧挤在一起送到时会丢样本，时间轴出现空洞，插值就会顿一下。
          // 现在样本按 ROS 时间戳排队，挤在一起到的帧也能均匀播放；界面数字仍由 scheduleEgoFlush 限频。
          if (msg.topic === LOC_TOPIC) handleLocalizationMessage(msg, "pose", e.data.length);
        } catch (err) {
          console.warn("⚠️ 实时定位消息解析失败:", err);
        }
      };
      pws.onerror = () => {};
      pws.onclose = () => {
        clearTimeout(timeout);
        const wasCurrent = poseWs === pws;
        if (wasCurrent) poseWs = null;
        if (!wasCurrent) return;
        if (!disposed && wsRef.current === ownerWs && ownerWs.readyState === WebSocket.OPEN) {
          poseReconnectTimer = setTimeout(() => startPoseSocket(url, ownerWs), 500);
        }
      };
    };

    const handleUdpHeavyChunk = (parsed, packet) => {
      const type = parsed?.type;
      const enabled = type === "objects" ? ROS_UDP_TELEMETRY.objectsEnabled
        : type === "trajectory" ? ROS_UDP_TELEMETRY.trajectoryEnabled
          : type === "path" ? ROS_UDP_TELEMETRY.pathEnabled : false;
      if (!enabled || parsed?.v !== 1) return false;
      const part = Number(parsed.part);
      const parts = Number(parsed.parts);
      const sec = Number(parsed.stamp?.[0]);
      const nanosec = Number(parsed.stamp?.[1]);
      if (!Number.isInteger(part) || !Number.isInteger(parts) || parts < 1 || parts > 128 || part < 0 || part >= parts) return false;
      const sourceItems = type === "objects" ? parsed.o : parsed.p;
      if (!Number.isFinite(sec) || !Number.isFinite(nanosec) || !Array.isArray(sourceItems)) return false;

      let items;
      if (type === "objects") {
        items = sourceItems.map((o, index) => {
          if (!Array.isArray(o) || o.length < 8) return null;
          const values = o.slice(1, 8).map(Number);
          if (!values.every(Number.isFinite)) return null;
          const [x, y, z, yaw, length, width, height] = values;
          if (length <= 0 || width <= 0 || height <= 0) return null;
          return { i: String(o[0] ?? `obj_${index}`), x, y, z, yaw, l: length, w: width, h: height };
        }).filter(Boolean);
      } else {
        const expected = type === "trajectory" ? 2 : 3;
        items = sourceItems.map((point) => {
          if (!Array.isArray(point) || point.length < expected) return null;
          const values = point.slice(0, expected).map(Number);
          return values.every(Number.isFinite) ? values : null;
        }).filter(Boolean);
      }

      const now = Date.now();
      const packetBytes = Number(packet.bytes) || packet.text.length;
      for (const [key, frame] of udpHeavyFrames) {
        if (now - frame.createdAt > 800) {
          udpDiagIncompleteFrame(udpDiagnosticsRef.current, frame.type);
          udpHeavyFrames.delete(key);
        }
      }
      const frameKey = `${type}:${packet.address || ""}:${String(parsed.seq)}:${sec}:${nanosec}`;
      let frame = udpHeavyFrames.get(frameKey);
      if (!frame) {
        udpDiagStartFrame(udpDiagnosticsRef.current, type, parsed.seq, now);
        frame = { type, createdAt: now, parts, chunks: Array.from({ length: parts }), got: 0, bytes: 0, stampMs: sec * 1000 + nanosec / 1e6 };
        udpHeavyFrames.set(frameKey, frame);
        if (udpHeavyFrames.size > 16) {
          const oldestKey = udpHeavyFrames.keys().next().value;
          const oldestFrame = udpHeavyFrames.get(oldestKey);
          if (oldestFrame) udpDiagIncompleteFrame(udpDiagnosticsRef.current, oldestFrame.type);
          udpHeavyFrames.delete(oldestKey);
        }
      }
      udpDiagPacket(udpDiagnosticsRef.current, type, packetBytes, packet.address, now);
      if (frame.parts !== parts) {
        udpDiagIncompleteFrame(udpDiagnosticsRef.current, type);
        udpHeavyFrames.delete(frameKey);
        return false;
      }
      trafficRef.current.rxBytes += packetBytes;
      if (frame.chunks[part] === undefined) {
        frame.chunks[part] = items;
        frame.bytes += packetBytes;
        frame.got += 1;
      }
      if (frame.got !== frame.parts) return true;
      udpHeavyFrames.delete(frameKey);
      udpDiagCompleteFrame(udpDiagnosticsRef.current, type, now);

      // 旧帧或乱序帧不覆盖新帧；ROS 时钟重置造成的大幅回跳仍允许通过。
      const previousStamp = udpHeavyStampMs[type];
      if (frame.stampMs > 0 && previousStamp > 0 && frame.stampMs <= previousStamp
          && previousStamp - frame.stampMs < 10000) return true;
      if (frame.stampMs > 0) udpHeavyStampMs[type] = frame.stampMs;
      udpHeavyAt[type] = now;

      const merged = frame.chunks.flat();
      let statTopic;
      if (type === "objects") {
        const objects = objectsToScene(objectsFromWeb({ o: merged }));
        setState((s) => ({ ...s, objects, objectsAt: now }));
        statTopic = OBJECTS_TOPIC;
      } else if (type === "trajectory") {
        const newPath = trajectoryFromWeb({ p: merged });
        if (newPath.length >= 2) setState((s) => ({ ...s, path: newPath, pathAt: now }));
        statTopic = TRAJ_TOPIC;
      } else {
        const rosPts = routeFromWeb({ p: merged });
        const globalPath = rosPts.map((point) => cartesianToLatLon(point.x, point.y, MAP_ORIGIN.lat, MAP_ORIGIN.lng));
        const globalPath3d = rosPts.map((point) => ({ x: point.y, y: point.z, z: point.x }));
        if (globalPath.length >= 2) setState((s) => ({ ...s, globalPath, globalPath3d }));
        statTopic = BEHAVIOR_PATH_TOPIC;
      }

      const stat = topicStatsRef.current[statTopic] || (topicStatsRef.current[statTopic] = { lastAt: 0, stamps: [] });
      stat.lastAt = now;
      stat.stamps.push(now);
      if (stat.stamps.length > 200) stat.stamps.shift();
      if (LIVE_TOPICS.includes(statTopic)) {
        gotAnyData = true;
        lastDataAt = now;
        lastDataAtRef.current = now;
        idleCloseCount = 0;
      }
      netLog.push(statTopic, "udp", frame.stampMs, performance.now(), frame.bytes);
      if (!udpHeavyEverReceived[type]) {
        udpHeavyEverReceived[type] = true;
        const label = { objects: "bounding-box 障碍物", trajectory: "控制轨迹", path: "行为路径" }[type];
        console.info(`📦 UDP ${label}已接通：${packet.address}:${packet.port}`);
      }
      return true;
    };

    const startUdpTelemetryReceiver = () => {
      if (!ROS_UDP_TELEMETRY.enabled) return false;
      if (!window.rosUdp?.start || !window.rosUdp?.onMessage) {
        console.info("ℹ️ 当前不是 Electron 环境，UDP 遥测不可用，将使用 WebSocket");
        return false;
      }

      udpMessageUnsubscribe = window.rosUdp.onMessage((packet) => {
        if (disposed || typeof packet?.text !== "string") return;
        try {
          const parsed = JSON.parse(packet.text);
          const msg = udpPosePacketToRosMessage(parsed)
            || (parsed?.topic === LOC_TOPIC && parsed?.msg ? parsed : null);
          if (msg) {
            const now = Date.now();
            const packetBytes = Number(packet.bytes) || packet.text.length;
            udpDiagStartFrame(udpDiagnosticsRef.current, "pose", parsed.seq, now);
            udpDiagPacket(udpDiagnosticsRef.current, "pose", packetBytes, packet.address, now);
            udpDiagCompleteFrame(udpDiagnosticsRef.current, "pose", now);
            lastUdpPoseAt = now;
            trafficRef.current.rxBytes += packetBytes;
            handleLocalizationMessage(msg, "udp", packetBytes);

            // UDP 恢复后停止额外的高频 WebSocket 定位，业务 WebSocket 与 5 Hz 容灾订阅继续保留。
            if (poseWs) stopPoseSocket();
            if (!udpEverReceived) {
              udpEverReceived = true;
              console.info(`⚡ UDP 实时定位已接通：${packet.address}:${packet.port}`);
            }
            return;
          }
          handleUdpHeavyChunk(parsed, packet);
        } catch (err) {
          console.warn("⚠️ UDP 遥测消息解析失败:", err);
        }
      });
      udpErrorUnsubscribe = window.rosUdp.onError?.((message) => {
        console.warn(`⚠️ UDP 遥测接收错误：${message}`);
      });

      window.rosUdp.start({
        address: ROS_UDP_TELEMETRY.listenAddress,
        port: ROS_UDP_TELEMETRY.listenPort,
        allowedSource: ROS_UDP_TELEMETRY.allowedSource,
      }).then(({ address, port }) => {
        if (!disposed) console.info(`📡 UDP 遥测监听 ${address}:${port}`);
      }).catch((err) => {
        console.warn(`⚠️ UDP 遥测监听启动失败：${err?.message || err}；将使用 WebSocket`);
      });
      return true;
    };

    const schedulePoseTransport = (url, ownerWs) => {
      clearTimeout(udpFallbackTimer);
      if (!ROS_UDP_TELEMETRY.enabled || !window.rosUdp) {
        startPoseSocket(url, ownerWs);
        return;
      }

      const check = () => {
        if (disposed || wsRef.current !== ownerWs || ownerWs.readyState !== WebSocket.OPEN) return;
        const udpFresh = lastUdpPoseAt > 0 && Date.now() - lastUdpPoseAt <= ROS_UDP_TELEMETRY.fallbackMs;
        if (udpFresh) {
          if (poseWs) stopPoseSocket();
        } else if (!poseWs) {
          console.warn("⚠️ UDP 定位未到达或已超时，启用 WebSocket 实时定位回退");
          startPoseSocket(url, ownerWs);
        }
        udpFallbackTimer = setTimeout(check, 1000);
      };

      // 启动时先给工控机 UDP 节点一个完整的等待窗口，避免同时接收两份 20 Hz 定位。
      udpFallbackTimer = setTimeout(check, ROS_UDP_TELEMETRY.fallbackMs);
    };

    const startWatchdog = () => {
      clearInterval(watchdogTimer);
      watchdogTimer = setInterval(() => {
        if (disposed) return;
        if (document.hidden) return;
        // 曾收到过数据、但超过 10 秒没有新数据时，视为链路假死，主动断开触发重连。
        if (gotAnyData && lastDataAt > 0 && Date.now() - lastDataAt > 10000 && idleCloseCount < 3) {
          const w = wsRef.current;
          if (w && (w.readyState === WebSocket.OPEN || w.readyState === WebSocket.CONNECTING)) {
            console.warn("⚠️ 超过 10 秒未收到数据，主动断开重连...");
            idleCloseCount += 1;
            w.close();
          }
        }
      }, 5000);
    };

    const connect = () => {
    if (!attempts.length) {
      setConnectionError("rosbridge 地址无效，请检查车辆通道配置");
      return;
    }
    const attempt = attempts[attemptIdx];
    wsUrl = attempt.url;
    const connectStartedAt = performance.now();
    console.log(`🔌 正在连接车辆：${wsUrl}（候选 ${attemptIdx + 1}/${attempts.length}）`);
    setActiveUrl(wsUrl);
    if (attempt.blockedByMixedContent) {
      const reason = "当前网页使用 HTTPS，浏览器禁止连接局域网 ws://。请用 HTTP 打开本网页，或给 rosbridge 配置可信的 WSS 反向代理";
      console.error(`❌ ${reason}`);
      setConnected(false);
      setConnectionMs(null);
      setConnectionError(reason);
      return;
    }
    let opened = false;
    let ws;
    try {
      ws = new WebSocket(wsUrl);
    } catch (err) {
      const reason = err?.message || "无法创建 WebSocket 连接";
      console.error(`❌ WebSocket 地址 ${wsUrl} 无效:`, err);
      setConnectionError(reason);
      return;
    }
    wsRef.current = ws;

    // ── UDP 紧凑遥测 / 原始 Autoware 话题自动回退 ──
    // 正常不订阅任何 /web/*；UDP 某类数据超时后，只回退该类原始话题。
    const RAW_HEAVY_SUBS = [
      { op: "subscribe", id: "raw-objects", topic: OBJECTS_TOPIC, type: "autoware_perception_msgs/msg/PredictedObjects", throttle_rate: SUB_OBJ_MS, queue_length: 1 },
      { op: "subscribe", id: "raw-trajectory", topic: TRAJ_TOPIC, type: "autoware_planning_msgs/msg/Trajectory", throttle_rate: SUB_PLAN_MS, queue_length: 1 },
      { op: "subscribe", id: "raw-path", topic: BEHAVIOR_PATH_TOPIC, type: "autoware_planning_msgs/msg/Path", throttle_rate: SUB_SLOW_MS, queue_length: 1 },
    ];
    const rawHeavySubscribed = new Set();
    let heavyFallbackTimer = null;
    const unsubscribeRawTopic = (rawTopic) => {
      const sub = RAW_HEAVY_SUBS.find((s) => s.topic === rawTopic);
      if (!sub || !rawHeavySubscribed.has(sub.id) || ws.readyState !== WebSocket.OPEN) return;
      ws.send(JSON.stringify({ op: "unsubscribe", id: sub.id, topic: sub.topic }));
      rawHeavySubscribed.delete(sub.id);
    };
    const subscribeRawTopic = (sub, reason) => {
      if (rawHeavySubscribed.has(sub.id) || ws.readyState !== WebSocket.OPEN) return;
      ws.send(JSON.stringify(sub));
      rawHeavySubscribed.add(sub.id);
      console.warn(`⚠️ ${reason}：回退订阅原始话题 ${sub.topic}（流量较大）`);
    };
    const scheduleHeavyTransport = () => {
      clearTimeout(heavyFallbackTimer);
      const startedAt = Date.now();
      const specForTopic = (topic) => {
        if (topic === OBJECTS_TOPIC) return { type: "objects", enabled: ROS_UDP_TELEMETRY.objectsEnabled, timeout: ROS_UDP_TELEMETRY.objectsFallbackMs };
        if (topic === TRAJ_TOPIC) return { type: "trajectory", enabled: ROS_UDP_TELEMETRY.trajectoryEnabled, timeout: ROS_UDP_TELEMETRY.trajectoryFallbackMs };
        return { type: "path", enabled: ROS_UDP_TELEMETRY.pathEnabled, timeout: ROS_UDP_TELEMETRY.pathFallbackMs };
      };
      const check = () => {
        if (disposed || wsRef.current !== ws || ws.readyState !== WebSocket.OPEN) return;
        const now = Date.now();
        RAW_HEAVY_SUBS.forEach((sub) => {
          const spec = specForTopic(sub.topic);
          const udpUsable = ROS_UDP_TELEMETRY.enabled && spec.enabled && Boolean(window.rosUdp);
          const at = udpHeavyAt[spec.type];
          const fresh = udpUsable && at > 0 && now - at <= spec.timeout;
          const waitingFirstFrame = udpUsable && at <= 0 && now - startedAt < spec.timeout;
          if (fresh || waitingFirstFrame) {
            if (fresh) unsubscribeRawTopic(sub.topic);
          } else {
            subscribeRawTopic(sub, `UDP ${spec.type} 未到达或已超时`);
          }
        });
        heavyFallbackTimer = setTimeout(check, 1000);
      };
      check();
    };
    // 握手超时：地址不通时浏览器可能很久才报错，主动放弃，尽快换下一个地址
    clearTimeout(openTimer);
    openTimer = setTimeout(() => {
      if (!opened && ws.readyState === WebSocket.CONNECTING) {
        console.warn(`⚠️ ${wsUrl} ${WS_CONNECT_TIMEOUT_MS / 1000} 秒内未连上，换下一个地址`);
        ws.close();
      }
    }, WS_CONNECT_TIMEOUT_MS);

    ws.onopen = () => {
      opened = true;
      clearTimeout(openTimer);
      if (attempt.addr !== serverIp.trim() || attempt.sc !== attempt.scheme) {
        console.warn(`⚠️ 填写的地址是 ${serverIp}，实际通过 ${wsUrl} 连上`);
      }
      console.log(`✅ WebSocket 已连接 ROS 2 上位机: ${wsUrl}`);
      setConnectionError("");
      startWatchdog();
      schedulePoseTransport(wsUrl, ws);
      setConnected(true);
      setConnectionMs(performance.now() - connectStartedAt);
      const send = (payload) => {
        const json = JSON.stringify(payload);
        ws.send(json);
      };
      // 显式声明发布者，兼容要求 advertise 后才能 publish 的 rosbridge 配置。
      send({ op: "advertise", topic: "/initialpose", type: "geometry_msgs/msg/PoseWithCovarianceStamped" });
      send({ op: "advertise", topic: "/planning/mission_planning/goal", type: "geometry_msgs/msg/PoseStamped" });
      // 未启用 CAN 直连时才保留旧的 ROS 底盘控制发布者，避免两个控制源同时下发。
      if (!CHASSIS_GATEWAY.enabled) {
        send({
          op: "advertise",
          topic: CONTROL_TOPIC,
          type: "autoware_control_msgs/msg/Control",
          qos: { history: "keep_last", depth: 1, reliability: "reliable", durability: "volatile" },
        });
        send({
          op: "advertise",
          topic: GEAR_TOPIC,
          type: "autoware_vehicle_msgs/msg/GearCommand",
          qos: { history: "keep_last", depth: 1, reliability: "reliable", durability: "volatile" },
        });
      }
      send({ op: "advertise", topic: OBSTACLE_CMD_TOPIC, type: "std_msgs/msg/String" });
      send({ op: "subscribe", topic: OBSTACLE_LIST_TOPIC, type: "std_msgs/msg/String" });
      // 请求节点立即回传一次障碍物列表（刷新网页后恢复显示）
      setTimeout(() => {
        if (ws.readyState === WebSocket.OPEN) send({ op: "publish", topic: OBSTACLE_CMD_TOPIC, type: "std_msgs/msg/String", msg: { data: JSON.stringify({ op: "list" }) } });
      }, 300);
      // 高频/大话题留在业务通道；定位主要由独立低延迟 WebSocket 接收。
      // 业务通道仍保留一份小消息容灾订阅，stamp 去重后不会重复驱动车辆。
      ws.send(JSON.stringify({ op: "subscribe", id: "fallback-pose", topic: LOC_TOPIC, type: "nav_msgs/msg/Odometry", throttle_rate: 200, queue_length: 1 })); // 容灾份 5 Hz 即可，原来 20 Hz 等于把定位流量收了两遍
      ws.send(JSON.stringify({ op: "subscribe", topic: "/initialpose", type: "geometry_msgs/msg/PoseWithCovarianceStamped" }));
      ws.send(JSON.stringify({ op: "subscribe", topic: "/goal_pose", type: "geometry_msgs/msg/PoseStamped" }));
      ws.send(JSON.stringify({ op: "subscribe", topic: "/planning/mission_planning/goal", type: "geometry_msgs/msg/PoseStamped" }));
      // 路线状态与路线内容（ADAPI，transient_local：rosbridge 会按发布端的 QoS 订阅，连上即可拿到当前路线）
      ws.send(JSON.stringify({ op: "subscribe", topic: ROUTE_STATE_TOPIC, type: "autoware_adapi_v1_msgs/msg/RouteState" }));
      ws.send(JSON.stringify({ op: "subscribe", topic: ROUTE_TOPIC, type: "autoware_adapi_v1_msgs/msg/Route" }));
      ws.send(JSON.stringify({ op: "subscribe", topic: CONTROL_TOPIC, type: "autoware_control_msgs/msg/Control", throttle_rate: SUB_CONTROL_MS, queue_length: 1 }));
      scheduleHeavyTransport();
      ws.send(JSON.stringify({ op: "subscribe", topic: GNSS_TOPIC, ...(GNSS_MSG_TYPE ? { type: GNSS_MSG_TYPE } : {}), throttle_rate: SUB_GNSS_MS, queue_length: 1 }));
      ws.send(JSON.stringify({ op: "subscribe", topic: "/api/operation_mode/state", type: "autoware_adapi_v1_msgs/msg/OperationModeState" }));
      if (!CHASSIS_GATEWAY.enabled) {
        ws.send(JSON.stringify({ op: "subscribe", topic: YUNLE_FEEDBACK_TOPIC, type: "chassis_interfaces/msg/CcuStatus", throttle_rate: 50, queue_length: 1 }));
        // 云乐底盘 0x100 BMS 反馈：SOC 百分比、电池电压和电流。
        ws.send(JSON.stringify({ op: "subscribe", topic: YUNLE_BMS_TOPIC, type: "chassis_interfaces/msg/BmsStatus", throttle_rate: 500, queue_length: 1 }));
      }
      // 定位初始化状态（1 未初始化 / 2 初始化中 / 3 已初始化）：用来显示进度，并在服务回复丢失时兜底判断成功
      ws.send(JSON.stringify({ op: "subscribe", topic: "/api/localization/initialization_state", type: "autoware_adapi_v1_msgs/msg/LocalizationInitializationState" }));
    };

    // rosbridge 会把超过分片阈值的大消息拆成多个 op:"fragment" 帧，
    // 客户端按 id + 序号拼回完整 JSON，收齐后只解析一次；单帧小消息不受影响。
    const fragmentParts = new Map();

    ws.onmessage = (e) => {
      trafficRef.current.rxBytes += typeof e.data === "string" ? e.data.length : (e.data?.byteLength || e.data?.size || 0);
      let msg;
      try {
        if (typeof e.data === "string") {
          const parsed = JSON.parse(e.data);
          if (parsed.op === "fragment") {
            const total = Number(parsed.total) || 0;
            let entry = fragmentParts.get(parsed.id);
            if (!entry || entry.total !== total) {
              entry = { total, parts: new Array(total), got: 0 };
              fragmentParts.set(parsed.id, entry);
              // 只保留最近几组分片，网络丢帧时不让半截数据越积越多
              if (fragmentParts.size > 4) fragmentParts.delete(fragmentParts.keys().next().value);
            }
            const idx = Number(parsed.num);
            if (entry.parts[idx] === undefined) { entry.parts[idx] = parsed.data; entry.got += 1; }
            if (entry.got === entry.total) {
              fragmentParts.delete(parsed.id);
              msg = JSON.parse(entry.parts.join(""));
            }
          } else {
            msg = parsed;
          }
        }
      } catch (err) {
        console.warn("⚠️ 消息解析失败:", err);
      }
      if (!msg) return;

      // 系统状态页统计各话题的到达频率（瘦身话题按对应的原始话题统计）
      const statTopic = WEB_TOPIC_ALIAS[msg.topic] || msg.topic;
      if (statTopic !== LOC_TOPIC && HEALTH_TOPICS.includes(statTopic)) {
        const st = topicStatsRef.current[statTopic] || (topicStatsRef.current[statTopic] = { lastAt: 0, stamps: [] });
        st.lastAt = Date.now();
        st.stamps.push(st.lastAt);
        if (st.stamps.length > 200) st.stamps.shift();
      }
      // 只有驱动小车画面位置/轨迹的话题才算“活数据”。
      if (statTopic !== LOC_TOPIC && LIVE_TOPICS.includes(statTopic)) {
        gotAnyData = true;
        lastDataAt = Date.now();
        lastDataAtRef.current = Date.now();
        idleCloseCount = 0;
      }

      // 实车定位初始化结果：工控机（Autoware）自行完成初始化，成功后读取工控机发布的小车位置
      if (msg.op === "service_response" && msg.id === "init_gnss") {
        const raw = msg.values;
        const ok = msg.result !== false && raw?.status?.success !== false;
        if (ok) {
          // 已经通过初始化状态话题判定成功（服务回复晚到），不再重复处理
          if (startStatusRef.current.kind === "gnss" && (startStatusRef.current.state === "verifying" || startStatusRef.current.state === "ok")) return;
          teleportNextRef.current = true;
          // 稍等片刻，让工控机发布初始化后的第一批定位，再读取
          gnssCheckRef.current = { checkAfter: performance.now() + GNSS_VERIFY_DELAY_MS };
          setStartStatus("verifying", "工控机已完成初始化，正在读取小车位置…", "gnss");
          return;
        }
        const detail = typeof raw === "string" ? raw : raw?.status?.message || "未知原因";
        gnssCheckRef.current = null;
        console.error("❌ 定位初始化失败:", detail);
        let reason = detail;
        if (/does not exist/i.test(detail)) reason = "Autoware 未启动";
        else if (/not supported/i.test(detail)) reason = "未启用卫星初始化"; // pose_initializer 的 gnss_enabled 未开启
        setStartStatus("failed", reason, "gnss");
        return;
      }

      if (msg.op === "service_response" && msg.id === "init_start") {
        const raw = msg.values;
        const ok = msg.result !== false && raw?.status?.success !== false;
        if (ok) {
          pendingInitRef.current = null;
          teleportNextRef.current = true;
          setStartStatus("ok");
          return;
        }
        const detail = typeof raw === "string" ? raw : raw?.status?.message || "未知原因";
        if (/does not exist/i.test(detail) && pendingInitRef.current) {
          // 没有定位初始化服务：退回发布 /initialpose（与 RViz 的 2D Pose Estimate 相同入口）
          wsRef.current?.send(JSON.stringify(pendingInitRef.current));
          pendingInitRef.current = null;
          teleportNextRef.current = true;
          setStartStatus("sent");
          return;
        }
        console.error("❌ 设起点失败:", detail);
        setStartStatus("failed", detail);
        return;
      }

      if (msg.op === "service_response" && msg.id === "change_to_stop") {
        const success = msg.result !== false && msg.values?.status?.success !== false;
        const raw = msg.values;
        const detail = typeof raw === "string" ? raw : raw?.status?.message || "";
        const reason = /does not exist/i.test(detail) ? "Autoware 未启动" : detail || "未知原因";
        if (!success) console.error("❌ 停车指令失败:", detail || msg.values || msg);
        setState((s) => ({ ...s, stopStatus: { state: success ? "ok" : "failed", msg: success ? "" : reason, at: Date.now() } }));
        return;
      }

      if (msg.op === "service_response" && msg.id === "clear_route") {
        const success = msg.result !== false && msg.values?.status?.success !== false;
        const raw = msg.values;
        const detail = typeof raw === "string" ? raw : raw?.status?.message || "";
        const reason = /does not exist/i.test(detail) ? "Autoware 未启动" : detail || "未知原因";
        if (!success) console.error("❌ 取消路线失败:", detail || msg.values || msg);
        setState((s) => (success
          ? { ...s, goalPoint: null, goalPose: null, globalPath: [], globalPath3d: [], path: [], routeClear: { state: "ok", msg: "", at: Date.now() } }
          : { ...s, routeClear: { state: "failed", msg: reason, at: Date.now() } }));
        return;
      }

      if (msg.op === "service_response" && msg.id === "change_to_autonomous") {
        const success = msg.result !== false && msg.values?.status?.success !== false;
        const raw = msg.values;
        const detail = typeof raw === "string" ? raw : raw?.status?.message || "";
        let reason = detail || "未知原因";
        if (/does not exist/i.test(detail)) reason = "Autoware 未启动";
        else if (/not available/i.test(detail)) reason = "不满足启动条件"; // 定位不可信 / 车辆不在轨迹上或未停稳 / 底盘未允许线控 / 存在紧急停车
        setState((s) => ({ ...s, autoRequestStatus: success ? "accepted" : "failed", autoFail: success ? null : { msg: reason, raw: detail, at: Date.now() } }));
        if (!success) console.error("❌ Autoware AUTO 切换失败:", detail || msg.values || msg);
        return;
      }

      if (msg.topic === "/api/localization/initialization_state" && msg.msg) {
        const st = Number(msg.msg.state);
        const prev = locInitStateRef.current;
        locInitStateRef.current = st;
        if (st !== prev) setLocInitState(st);
        const cur = startStatusRef.current;
        if (cur.kind === "gnss" && cur.state === "pending") {
          if (st === 2 && prev !== 2) {
            // 刷新提示并重新计时：Autoware 确实在工作，不算无响应
            setStartStatus("pending", "工控机正在匹配点云地图…", "gnss");
          } else if (st === 3 && prev === 2) {
            // 初始化中 → 已初始化：即使服务回复丢了，也按成功处理，随后读取工控机的小车位置
            teleportNextRef.current = true;
            gnssCheckRef.current = { checkAfter: performance.now() + GNSS_VERIFY_DELAY_MS };
            setStartStatus("verifying", "工控机已完成初始化，正在读取小车位置…", "gnss");
          }
        }
      }

      if (msg.topic === "/api/operation_mode/state") {
        setState((s) => ({ ...s, operationMode: msg.msg }));
      }

      // 云乐底盘 0x51(CCU) 反馈：手动控制的在线判定和安全联锁只依赖该消息，
      // 因此即使未启动 Autoware 定位，网页也能安全进行低速手动控制。
      if (msg.topic === YUNLE_FEEDBACK_TOPIC && msg.msg) {
        const at = Date.now();
        const status = msg.msg;
        chassisStatusRef.current = { value: status, at };
        const speedKmh = Number(status.ccu_vehicle_speed) || 0;
        const steeringDeg = Number(status.ccu_steering_wheel_angle) || 0;
        // 20 Hz 反馈只写入闭包里的 ego 和 ref（安全联锁读 ref，不受影响）；
        // 界面经 scheduleEgoFlush 统一限频到 5 Hz，底盘状态每 500 ms 才 setState 一次。
        chassisFreshAt = at;
        ego = { ...ego, speed: speedKmh / 3.6, steering: (steeringDeg * Math.PI) / 180 };
        scheduleEgoFlush();
        if (at - lastChassisUiAt >= 500) {
          lastChassisUiAt = at;
          setState((s) => ({ ...s, chassisStatus: status, chassisFeedbackAt: at }));
        }
      }

      if (msg.topic === YUNLE_BMS_TOPIC && msg.msg) {
        const at = Date.now();
        setState((s) => ({
          ...s,
          bmsStatus: msg.msg,
          bmsFeedbackAt: at,
        }));
      }

      // 仿真障碍物列表（节点每秒回传一次，内容变化时才更新界面）
      if (msg.topic === OBSTACLE_LIST_TOPIC && msg.msg) {
        obstacleListAtRef.current = Date.now();
        try {
          const list = (JSON.parse(msg.msg.data || "{}").obstacles || []).filter(
            (o) => Number.isFinite(Number(o.x)) && Number.isFinite(Number(o.y)) && Array.isArray(o.size)
          );
          const key = JSON.stringify(list);
          if (key !== obstacleListKeyRef.current) {
            obstacleListKeyRef.current = key;
            setState((s) => ({ ...s, placedObstacles: list }));
          }
        } catch (err) {
          console.warn("⚠️ 障碍物列表解析失败:", err);
        }
      }

      // 兼容某些 rosbridge 会在业务通道上仍转发已缓存定位帧的情况。
      if (msg.topic === LOC_TOPIC) {
        handleLocalizationMessage(msg, "main", typeof e.data === "string" ? e.data.length : 0);
        return;
      }

      // rosbridge 的订阅/调用错误（例如话题类型不匹配）会以 op:"status" 返回，打印出来便于排查
      if (msg.op === "status" && (msg.level === "error" || msg.level === "warning")) {
        console.warn(`⚠️ rosbridge ${msg.level}:`, msg.msg);
      }

      if (msg.topic === GNSS_TOPIC && msg.msg) {
        // 自动识别格式：GINS2000 有 gnss_fix_status 字段；标准 NavSatFix 用 status.status
        const m = msg.msg;
        const isGins = m.gnss_fix_status !== undefined || m.ins_status !== undefined;
        const status = isGins ? Number(m.gnss_fix_status ?? 0) : Number(m.status?.status ?? 0);
        const fix = {
          lat: Number(m.latitude),
          lon: Number(m.longitude),
          alt: Number(m.altitude) || 0,
          status,
          format: isGins ? "gins" : "navsat",
          insStatus: isGins ? Number(m.ins_status ?? 0) : null,
          at: Date.now(),
        };
        if (Number.isFinite(fix.lat) && Number.isFinite(fix.lon) && (fix.lat !== 0 || fix.lon !== 0)) {
          gnssRef.current = fix;
          setGnss(fix);
        }
      }


      // ── 处理起点（网页或 RViz 设置的起点都会显示标牌；车辆位置仍以定位结果为准） ──
      if (msg.topic === "/initialpose") {
        const pos = msg.msg.pose.pose.position;
        const q = msg.msg.pose.pose.orientation;
        const rosYaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const gps = cartesianToLatLon(pos.x, pos.y, MAP_ORIGIN.lat, MAP_ORIGIN.lng);
        setState((s) => ({
          ...s,
          startPoint: { lat: gps.lat, lng: gps.lng },
          startPose: { x: pos.y, z: pos.x, yaw: rosYaw },
        }));
      }

      // ── 处理终点 ──
      if (msg.topic === "/goal_pose" || msg.topic === "/planning/mission_planning/goal") {
        const pose = msg.msg.pose || {};
        const pos = pose.position || { x: 0, y: 0 };
        const q = pose.orientation || { x: 0, y: 0, z: 0, w: 1 };
        const rosYaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const gps = cartesianToLatLon(pos.x, pos.y, MAP_ORIGIN.lat, MAP_ORIGIN.lng);
        setState((s) => ({
          ...s,
          goalPoint: { lat: gps.lat, lng: gps.lng },
          goalPose: { x: pos.y, z: pos.x, yaw: rosYaw },
        }));
      }

      // ── 路线（Autoware 标准接口）：无论终点是网页、RViz、命令行还是路线服务设置的，都以这里为准 ──
      // 工控机用路线服务（如 /api/routing/set_route_points）设终点时不会发布 goal 话题，以前网页就一直显示“待命中”。
      // 两个来源共用同一套处理：ADAPI 原始接口（变化时才发），以及 web_relay 每秒重发的 /web/route（兜底）
      if (msg.topic === ROUTE_TOPIC && msg.msg) {
        const goal = msg.msg.data?.[0]?.goal;
        if (goal?.position) applyRouteGoal(goal.position.x, goal.position.y, quatYaw(goal.orientation || { x: 0, y: 0, z: 0, w: 1 }));
        else if (Array.isArray(msg.msg.data) && msg.msg.data.length === 0) applyRouteCleared();
      }
      // 路线状态：1 未设置 / 2 已设置 / 3 已到达 / 4 切换中
      if (msg.topic === ROUTE_STATE_TOPIC && msg.msg) applyRouteState(Number(msg.msg.state));
      if (msg.topic === WEB_ROUTE_TOPIC && msg.msg) {
        let r = null;
        try { r = JSON.parse(msg.msg.data || "{}"); } catch { r = null; }
        if (r) {
          // 每秒重发一次，内容没变就不触发界面更新
          const key = JSON.stringify(r);
          const inGrace = Date.now() - localGoalAtRef.current < LOCAL_GOAL_GRACE_MS; // 忽略期内不记“已处理”，忽略期过后再用
          if (key !== lastWebRouteKey && !inGrace) {
            lastWebRouteKey = key;
            if (r.state != null) applyRouteState(Number(r.state));
            if (Array.isArray(r.goal) && r.goal.length >= 2) applyRouteGoal(Number(r.goal[0]), Number(r.goal[1]), Number(r.goal[2]) || 0);
          }
        }
      }

      if (msg.topic === CONTROL_TOPIC) {
        const steer = Number(msg.msg.lateral?.steering_tire_angle) || 0;
        const nowMs = Date.now();
        if (nowMs - lastControlUiAt >= 500) { // 系统状态页 2 Hz 刷新足够
          lastControlUiAt = nowMs;
          setControlCmd({ steer, vel: Number(msg.msg.longitudinal?.velocity) || 0 });
        }
        // 转向只用一个来源：有底盘反馈时用实际转角，没有时才用控制指令
        if (nowMs - chassisFreshAt >= 1000) {
          ego = { ...ego, steering: steer };
          scheduleEgoFlush();
        }
      }

      // ── 障碍物（欧式聚类，全部为 UNKNOWN） ──
      // footprint 直接换算成“世界朝向下相对中心的偏移”，渲染时不再旋转：
      // UNKNOWN 目标的朝向不可靠，跳变时画面也不会抖。
      // 瘦身话题的内容是 std_msgs/String 里的 JSON
      let webData = null;
      if (WEB_TOPIC_ALIAS[msg.topic] && msg.msg) {
        try { webData = JSON.parse(msg.msg.data || "{}"); } catch { webData = null; }
        if (!webData) return;
      }
      if (NETLOG_ENABLED && (webData || msg.topic === OBJECTS_TOPIC || msg.topic === TRAJ_TOPIC || msg.topic === BEHAVIOR_PATH_TOPIC)) {
        const hs = msg.msg?.header?.stamp;
        const stampMs = webData ? Number(webData.t) || 0 : hs ? (hs.sec ?? 0) * 1000 + (hs.nanosec ?? 0) / 1e6 : 0;
        netLog.push(msg.topic, "main", stampMs, performance.now(), typeof e.data === "string" ? e.data.length : 0);
      }

      // ── 障碍物（欧式聚类，全部为 UNKNOWN） ──
      if (msg.topic === WEB_OBJECTS_TOPIC || msg.topic === OBJECTS_TOPIC) {
        const objects = objectsToScene(webData ? objectsFromWeb(webData) : objectsFromRaw(msg.msg.objects));
        setState((s) => ({ ...s, objects, objectsAt: Date.now() }));
      }

      // ── 局部规划路径 ──
      // 只使用控制器实际跟踪的 Trajectory。不要把 Behavior Path 和 Trajectory
      // 写进同一个 path：两者发布频率、采样点数和前视范围不同，交替到达时
      // 会造成绿色引导带长度突变。
      if (msg.topic === WEB_TRAJ_TOPIC || msg.topic === TRAJ_TOPIC) {
        const newPath = webData ? trajectoryFromWeb(webData) : trajectoryFromRaw(msg.msg.points);
        // 规划器切换状态时可能短暂发布空/单点帧，保留上一条有效轨迹
        if (newPath.length >= 2) {
          setState((s) => ({ ...s, path: newPath, pathAt: Date.now() }));
        }
      }

      // ── 卫星地图全局路径 ──
      // 与 3D 绿色局部 Trajectory 分开保存，避免两种不同长度的路径互相覆盖。
      if (msg.topic === WEB_PATH_TOPIC || msg.topic === BEHAVIOR_PATH_TOPIC) {
        const rosPts = webData ? routeFromWeb(webData) : routeFromRaw(msg.msg.points);
        // 卫星图用经纬度；3D 路线条带用同一条路径（ROS → Three: x=ROS.y, y=高程, z=ROS.x）
        const globalPath = rosPts.map((pt) => cartesianToLatLon(pt.x, pt.y, MAP_ORIGIN.lat, MAP_ORIGIN.lng));
        const globalPath3d = rosPts.map((pt) => ({ x: pt.y, y: pt.z, z: pt.x }));

        // 保留上一条有效全局路径，防止规划模块切换时的空帧导致地图折线闪烁。
        if (globalPath.length >= 2) {
          setState((s) => ({ ...s, globalPath, globalPath3d }));
        }
      }
    };

    ws.onerror = (err) => {
      if (!disposed) {
        console.error(`❌ WebSocket 连接到 ${wsUrl} 失败:`, err);
        setConnectionError(`无法连接 ${wsUrl}，请检查 rosbridge、端口和网络`);
      }
    };
    ws.onclose = () => {
      clearTimeout(openTimer);
      clearTimeout(heavyFallbackTimer);
      clearTimeout(udpFallbackTimer);
      stopPoseSocket();
      if (wsRef.current === ws) wsRef.current = null;
      setConnected(false);
      setConnectionMs(null);
      if (disposed) return;
      if (reconnectTimer) return;
      // 从没连上过 → 换下一个候选；连上过又断了 → 先原地重连，再失败就会换
      if (!opened) attemptIdx = (attemptIdx + 1) % attempts.length;
      const delay = opened ? 1000 : attemptIdx === 0 ? 3000 : 500; // 一轮全试完再歇 3 秒
      console.log(`WebSocket 已关闭，${delay / 1000} 秒后连接 ${attempts[attemptIdx].url}...`);
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connect();
      }, delay);
    };
    };

    startUdpTelemetryReceiver();
    connect();

    return () => {
      disposed = true;
      clearTimeout(reconnectTimer);
      clearTimeout(poseReconnectTimer);
      clearTimeout(udpFallbackTimer);
      clearTimeout(openTimer);
      clearTimeout(startStatusTimerRef.current);
      clearTimeout(egoFlushTimer);
      clearInterval(watchdogTimer);
      const w = wsRef.current;
      if (w && w.readyState === WebSocket.OPEN) {
        w.send(JSON.stringify({ op: "unadvertise", topic: "/initialpose" }));
        w.send(JSON.stringify({ op: "unadvertise", topic: "/planning/mission_planning/goal" }));
        if (!CHASSIS_GATEWAY.enabled) {
          w.send(JSON.stringify({ op: "unadvertise", topic: CONTROL_TOPIC }));
          w.send(JSON.stringify({ op: "unadvertise", topic: GEAR_TOPIC }));
        }
        w.send(JSON.stringify({ op: "unadvertise", topic: OBSTACLE_CMD_TOPIC }));
      }
      if (wsRef.current === w) wsRef.current = null;
      stopPoseSocket();
      if (w) w.close();
      udpMessageUnsubscribe?.();
      udpErrorUnsubscribe?.();
      if (window.rosUdp?.stop) window.rosUdp.stop().catch(() => {});
    };
  }, [serverIp]);

  // 设起点：调用 Autoware 定位初始化服务并带上位姿（仿真中车辆会被直接放到起点）；
  // 设终点：发布到 /planning/mission_planning/goal，mission planner 随后发布全局路线。
  const publishPose = useCallback((kind, mapX, mapY, yaw = 0, mapZ = 0) => {
    const gps = cartesianToLatLon(mapX, mapY, MAP_ORIGIN.lat, MAP_ORIGIN.lng);
    const pose3d = { x: mapY, z: mapX, yaw };
    setState((s) => kind === "start"
      ? { ...s, startPoint: gps, startPose: pose3d }
      : { ...s, goalPoint: gps, goalPose: pose3d, globalPath: [], globalPath3d: [] });
    if (kind !== "start") localGoalAtRef.current = Date.now();

    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      console.warn("ROS bridge is offline; pose is shown locally but was not published.");
      if (kind === "start") setStartStatus("failed", "未连接车辆");
      return true;
    }

    const orientation = { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
    const now = Date.now();
    const stamp = { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1000000 };
    const position = { x: mapX, y: mapY, z: Number.isFinite(mapZ) ? mapZ : 0 };

    if (kind === "start") {
      const covariance = Array(36).fill(0);
      covariance[0] = 0.25; covariance[7] = 0.25; covariance[35] = 0.0685;
      const poseMsg = { header: { stamp, frame_id: "map" }, pose: { pose: { position, orientation }, covariance } };
      pendingInitRef.current = { op: "publish", topic: "/initialpose", type: "geometry_msgs/msg/PoseWithCovarianceStamped", msg: poseMsg };
      setStartStatus("pending");
      ws.send(JSON.stringify({
        op: "call_service",
        id: "init_start",
        service: "/api/localization/initialize",
        type: "autoware_adapi_v1_msgs/srv/InitializeLocalization",
        args: { pose: [poseMsg] },
      }));
    } else {
      // Autoware Mission Planner 的目标入口。/goal_pose 常用于界面中转，
      // 但并非所有部署都会把它转发给任务规划器。
      ws.send(JSON.stringify({ op: "publish", topic: "/planning/mission_planning/goal", type: "geometry_msgs/msg/PoseStamped", msg: {
        header: { stamp, frame_id: "map" },
        pose: { position, orientation },
      }}));
    }
    return true;
  }, [setStartStatus]);

  // 实车模式：上位机只发送“定位初始化”请求，由工控机（Autoware）自己读取 GNSS 并匹配点云完成初始化；
  // 完成后读取工控机发布的小车位置（/localization/kinematic_state）作为起点并显示。
  // 等同于在工控机上执行：ros2 service call /api/localization/initialize autoware_adapi_v1_msgs/srv/InitializeLocalization "{}"
  const initFromGnss = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      setStartStatus("failed", "未连接车辆", "gnss");
      return false;
    }
    gnssCheckRef.current = null; // 等工控机回复成功后再开始读取位置
    setStartStatus("pending", "已发送定位初始化请求，等待工控机完成…", "gnss");
    console.info("📡 已向工控机发送定位初始化请求（/api/localization/initialize，参数 {}）");
    ws.send(JSON.stringify({
      op: "call_service",
      id: "init_gnss",
      service: "/api/localization/initialize",
      type: "autoware_adapi_v1_msgs/srv/InitializeLocalization",
      args: {},
    }));
    return true;
  }, [setStartStatus]);

  const requestAutonomousMode = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return false;
    setState((s) => ({ ...s, autoRequestStatus: "pending", autoFail: null }));
    // 服务长时间不回包时复位，避免按钮一直处于“切换中”
    setTimeout(() => setState((s) => (s.autoRequestStatus === "pending"
      ? { ...s, autoRequestStatus: "failed", autoFail: { msg: "车辆无响应", raw: "", at: Date.now() } }
      : s)), 8000);
    ws.send(JSON.stringify({
      op: "call_service",
      id: "change_to_autonomous",
      service: "/api/operation_mode/change_to_autonomous",
      type: "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
      args: {},
    }));
    return true;
  }, []);

  // 手动控制只检查底盘反馈和物理自动驾驶按钮。
  // 不能在首帧 0x121 发送前强制要求 ccu_drive_mode==1：底盘正是收到持续 0x121 后才会从遥控模式转入自动模式。
  const requestRemoteMode = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return false;
    const feedback = chassisStatusRef.current;
    if (!feedback.value || Date.now() - feedback.at > TELEOP.feedbackTimeoutMs) {
      setState((s) => ({ ...s, manualError: "没有收到云乐底盘反馈，请检查 chassis_driver、can0 和 rosbridge" }));
      return false;
    }
    if (!feedback.value.ccu_drive_mode_shift_button) {
      setState((s) => ({ ...s, manualError: "请先闭合底盘自动驾驶切换按钮" }));
      return false;
    }
    remoteGearRef.current = 1;
    setState((s) => ({ ...s, manualError: "" }));
    return true;
  }, []);

  // 停车：切换到 STOP 模式，Autoware 平稳减速停车并保持
  // 等同于：ros2 service call /api/operation_mode/change_to_stop autoware_adapi_v1_msgs/srv/ChangeOperationMode
  const requestStop = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      setState((s) => ({ ...s, stopStatus: { state: "failed", msg: "未连接车辆", at: Date.now() } }));
      return false;
    }
    setState((s) => ({ ...s, stopStatus: { state: "pending", msg: "", at: Date.now() } }));
    setTimeout(() => setState((s) => (s.stopStatus.state === "pending"
      ? { ...s, stopStatus: { state: "failed", msg: "车辆无响应", at: Date.now() } }
      : s)), 8000);
    ws.send(JSON.stringify({
      op: "call_service",
      id: "change_to_stop",
      service: "/api/operation_mode/change_to_stop",
      type: "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
      args: {},
    }));
    return true;
  }, []);

  // 取消路线：清除 Autoware 当前路线（自动驾驶中不允许，需先停车）
  // 等同于：ros2 service call /api/routing/clear_route autoware_adapi_v1_msgs/srv/ClearRoute
  const clearRoute = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      setState((s) => ({ ...s, routeClear: { state: "failed", msg: "未连接车辆", at: Date.now() } }));
      return false;
    }
    setState((s) => ({ ...s, routeClear: { state: "pending", msg: "", at: Date.now() } }));
    setTimeout(() => setState((s) => (s.routeClear.state === "pending"
      ? { ...s, routeClear: { state: "failed", msg: "车辆无响应", at: Date.now() } }
      : s)), 8000);
    ws.send(JSON.stringify({
      op: "call_service",
      id: "clear_route",
      service: "/api/routing/clear_route",
      type: "autoware_adapi_v1_msgs/srv/ClearRoute",
      args: {},
    }));
    return true;
  }, []);

  // 保留函数名以减少界面改动。linearX 是带方向的目标速度(m/s)，
  // angularZ 是 Autoware 轮胎转角(rad，左正右负)；forceNeutral 用于空格停车和退出遥控。
  const publishCmdVel = useCallback((linearX, angularZ, forceNeutral = false) => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return false;
    const now = Date.now();
    const stamp = { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1000000 };
    const requestedSpeed = Number(linearX) || 0;
    const drive = requestedSpeed > 1e-4;
    const reverse = requestedSpeed < -1e-4;
    const moving = drive || reverse;
    const feedback = chassisStatusRef.current;
    const feedbackFresh = Boolean(feedback.value) && now - feedback.at <= TELEOP.feedbackTimeoutMs;
    const actualSpeedKmh = Math.abs(Number(feedback.value?.ccu_vehicle_speed) || 0);
    const status = feedback.value || {};
    const chassisReady = feedbackFresh
      && Boolean(status.ccu_drive_mode_shift_button)
      && Number(status.ccu_drive_mode) === 1;
    const safetyBlocked = !chassisReady
      || Boolean(status.ccu_parking_status)
      || Boolean(status.emergency_brake_request_status)
      || Boolean(status.remote_brake_request_status)
      || Boolean(status.touch_brake_request_status)
      || Boolean(status.handle_brake_request_status);

    // 未停稳时禁止 D/R 直接切换。松开运动键时保持当前档并主动制动；
    // 空格或退出遥控仅在基本停稳后切到 N。
    let gear = remoteGearRef.current;
    const desiredGear = drive ? 2 : reverse ? 20 : gear;
    const changingDirection = moving && gear !== 1 && gear !== desiredGear;
    const directionChangeBlocked = changingDirection && actualSpeedKmh >= 0.2;
    if (moving && !directionChangeBlocked && actualSpeedKmh < 0.2) gear = desiredGear;
    if (forceNeutral && actualSpeedKmh < 0.2) gear = 1;
    remoteGearRef.current = gear;

    const steeringRad = Math.min(TELEOP.steering, Math.max(-TELEOP.steering, Number(angularZ) || 0));
    const allowMotion = moving && !safetyBlocked && !directionChangeBlocked;
    // 档位与控制同周期发布，相当于同时运行 ros2 topic pub -r 20。
    ws.send(JSON.stringify({
      op: "publish",
      topic: GEAR_TOPIC,
      type: "autoware_vehicle_msgs/msg/GearCommand",
      msg: { stamp, command: gear },
    }));
    ws.send(JSON.stringify({
      op: "publish",
      topic: CONTROL_TOPIC,
      type: "autoware_control_msgs/msg/Control",
      msg: {
        stamp,
        control_time: { sec: 0, nanosec: 0 },
        lateral: {
          stamp,
          control_time: { sec: 0, nanosec: 0 },
          steering_tire_angle: steeringRad,
          steering_tire_rotation_rate: 0,
          is_defined_steering_tire_rotation_rate: false,
        },
        longitudinal: {
          stamp,
          control_time: { sec: 0, nanosec: 0 },
          velocity: allowMotion ? requestedSpeed : 0,
          acceleration: 0,
          jerk: 0,
          is_defined_acceleration: false,
          is_defined_jerk: false,
        },
      },
    }));
    return true;
  }, []);

  // 仿真障碍物指令：add / remove / clear（由工控机 click_to_obstacle 节点执行）
  const sendObstacleCmd = useCallback((cmd) => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return false;
    ws.send(JSON.stringify({ op: "publish", topic: OBSTACLE_CMD_TOPIC, type: "std_msgs/msg/String", msg: { data: JSON.stringify(cmd) } }));
    return true;
  }, []);
  const addObstacle = useCallback((x, y, yaw, preset) => sendObstacleCmd({
    op: "add", x, y, z: 0, yaw, size: preset.size, label: OBSTACLE_LABEL,
  }), [sendObstacleCmd]);
  const removeObstacle = useCallback((id) => sendObstacleCmd({ op: "remove", id }), [sendObstacleCmd]);
  const clearObstacles = useCallback(() => sendObstacleCmd({ op: "clear" }), [sendObstacleCmd]);

  return { ...state, activeUrl, connectionError, poseBufferRef, poseTimelineRef, publishPose, addObstacle, removeObstacle, clearObstacles, obstacleNodeOnline, initFromGnss, gnss, requestAutonomousMode, requestRemoteMode, requestStop, clearRoute, publishCmdVel, connected, connectionMs, rxKbps, dataAgeSec, topicHealth, udpDiagnostics, resetUdpDiagnostics, locInitState, controlCmd };
}

// ── 相机画面：MediaMTX WebRTC（WHEP）播放器 ──
// 向 http://<IP>:8889/cam/whep 发起 WebRTC 拉流，画面直接进 <video>；
// 断流、卡死、推流端重启后都会自动重连；重连期间保留最后一帧，不黑屏。

// 相机地址规范化：ws→http、wss→https，没写路径时补上默认流名 /cam/
function normalizeCameraUrl(raw) {
  let u = String(raw || "").trim();
  if (!u) return u;
  u = u.replace(/^ws:\/\//i, "http://").replace(/^wss:\/\//i, "https://");
  if (!/^https?:\/\//i.test(u)) u = `http://${u}`;
  try {
    const url = new URL(u);
    if (url.pathname === "/" || url.pathname === "") url.pathname = "/cam/";
    if (!url.pathname.endsWith("/")) url.pathname += "/";
    return url.toString();
  } catch {
    return u;
  }
}

// 只保留连接地址为 host 的 ICE 候选，让手动选择的 ZeroTier / FRP 路线真正隔离。
// MediaMTX 的 webrtcAdditionalHosts 同时发布 IP 和域名，因此两类地址都可匹配。
function pinCandidates(sdp, host) {
  const target = String(host || "").trim().toLowerCase();
  if (!target) return sdp;
  const lines = sdp.split(/\r?\n/);
  const isCand = (l) => l.startsWith("a=candidate:");
  const kept = lines.filter((l) => !isCand(l) || String(l.split(" ")[4] || "").toLowerCase() === target);
  return kept.some(isCand) ? kept.join("\r\n") : sdp;
}

// ── 相机通道测速：对一个通道建一条临时 WebRTC 会话（不显示画面），统计一小段时间的传输质量 ──
// 返回 { ok, score（越小越好）, rtt, loss, kbps, firstMs, reason }
async function probeCameraChannel(channel) {
  const base = normalizeCameraUrl(channel.url);
  const whepUrl = new URL("whep", base.endsWith("/") ? base : `${base}/`).toString();
  const host = new URL(whepUrl).hostname;
  const startedAt = performance.now();
  const deadline = startedAt + CAMERA_PROBE_TIMEOUT_MS;
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const pc = new RTCPeerConnection();
  let sessionUrl = null;
  const fail = (reason) => ({ ...channel, url: base, ok: false, score: Infinity, reason });
  try {
    pc.addTransceiver("video", { direction: "recvonly" });
    await pc.setLocalDescription(await pc.createOffer());
    await new Promise((resolve) => {
      if (pc.iceGatheringState === "complete") return resolve();
      const t = setTimeout(resolve, CAMERA_ICE_GATHER_TIMEOUT_MS);
      pc.addEventListener("icegatheringstatechange", () => {
        if (pc.iceGatheringState === "complete") { clearTimeout(t); resolve(); }
      });
    });
    const ctrl = new AbortController();
    const fetchTimer = setTimeout(() => ctrl.abort(), Math.max(1000, deadline - performance.now()));
    let res;
    try {
      res = await fetch(whepUrl, { method: "POST", headers: { "Content-Type": "application/sdp" }, body: pc.localDescription.sdp, signal: ctrl.signal });
    } finally {
      clearTimeout(fetchTimer);
    }
    if (res.status !== 201) return fail(`HTTP ${res.status}`);
    const loc = res.headers.get("Location");
    if (loc) sessionUrl = new URL(loc, whepUrl).toString();
    let answer = await res.text();
    if (CAMERA_PIN_HOST) answer = pinCandidates(answer, host);
    await pc.setRemoteDescription({ type: "answer", sdp: answer });

    const read = async () => {
      const report = await pc.getStats();
      let inbound = null;
      let pair = null;
      report.forEach((r) => {
        if (r.type === "inbound-rtp" && (r.kind === "video" || r.mediaType === "video")) inbound = r;
        if (r.type === "candidate-pair" && (r.nominated || r.selected) && r.state === "succeeded") pair = r;
      });
      return {
        t: performance.now(),
        bytes: inbound?.bytesReceived || 0,
        recv: inbound?.packetsReceived || 0,
        lost: Math.max(0, inbound?.packetsLost || 0),
        rtt: pair?.currentRoundTripTime != null ? pair.currentRoundTripTime * 1000 : null,
      };
    };

    // 等第一批数据
    let a = await read();
    while (a.bytes === 0) {
      if (performance.now() > deadline) return fail("没有收到数据");
      await sleep(200);
      a = await read();
    }
    const firstMs = a.t - startedAt;
    // 统计一段时间
    await sleep(Math.min(CAMERA_PROBE_MS, Math.max(500, deadline - performance.now())));
    const b = await read();
    const dt = (b.t - a.t) / 1000;
    const kbps = dt > 0 ? ((b.bytes - a.bytes) * 8) / dt / 1000 : 0;
    const dRecv = b.recv - a.recv;
    const dLost = b.lost - a.lost;
    const loss = dRecv + dLost > 0 ? (dLost / (dRecv + dLost)) * 100 : 0;
    const rtt = b.rtt ?? a.rtt;
    // 评分：延迟为主，丢包重罚（1% 丢包约等于 40 ms 延迟），码率过低（几乎没画面）额外罚分，出图慢略罚
    const score = (rtt ?? 200) + loss * 40 + (kbps < 100 ? 500 : 0) + firstMs * 0.05;
    return { ...channel, url: base, ok: true, score, rtt, loss, kbps, firstMs };
  } catch (err) {
    return fail(err?.name === "AbortError" ? "连接超时" : err?.message === "Failed to fetch" ? "无法访问" : err?.message || "连接失败");
  } finally {
    if (sessionUrl) fetch(sessionUrl, { method: "DELETE" }).catch(() => {});
    pc.close();
  }
}

// ── 相机通道：手动选择（不自动切换），并定期给“没在用的通道”测一次连通性和带宽 ──
// 当前通道的状态由播放器实时统计提供，不再额外测速，避免重复占带宽。
function useCameraChannel(channels, enabled) {
  const [selectedName, setSelectedName] = useState(() => {
    try {
      const saved = localStorage.getItem("dt_cam_channel");
      if (saved && channels.some((c) => c.name === saved)) return saved;
    } catch { /* 忽略 */ }
    return channels[0].name;
  });
  const current = channels.find((c) => c.name === selectedName) || channels[0];
  const [results, setResults] = useState({}); // name → 测速结果
  const [probing, setProbing] = useState({}); // name → 是否正在测
  const probingRef = useRef(new Set());

  const select = useCallback((name) => {
    setSelectedName(name);
    try { localStorage.setItem("dt_cam_channel", name); } catch { /* 忽略 */ }
  }, []);

  const probeOne = useCallback(async (channel) => {
    if (probingRef.current.has(channel.name)) return;
    probingRef.current.add(channel.name);
    setProbing((p) => ({ ...p, [channel.name]: true }));
    const r = await probeCameraChannel(channel);
    console.info(r.ok
      ? `📷 相机通道「${r.name}」连通：${(r.kbps / 1000).toFixed(1)} Mbps / 延迟 ${r.rtt != null ? r.rtt.toFixed(0) : "--"} ms / 丢包 ${r.loss.toFixed(1)}%`
      : `📷 相机通道「${r.name}」不通：${r.reason}`);
    probingRef.current.delete(channel.name);
    setProbing((p) => ({ ...p, [channel.name]: false }));
    setResults((m) => ({ ...m, [channel.name]: { ...r, at: Date.now() } }));
  }, []);

  // 只测没在用的通道：打开页面、切换通道后立即测一次，之后定期复测
  useEffect(() => {
    if (!enabled || channels.length < 2) return;
    const run = () => channels.filter((c) => c.name !== current.name).forEach((c) => probeOne(c));
    run();
    const id = setInterval(run, CAMERA_REPROBE_MS);
    return () => clearInterval(id);
  }, [enabled, channels, current.name, probeOne]);

  return { current, select, results, probing, probeOne };
}

// ── 相机通道下拉：悬浮在视频右上角，显示当前通道与带宽；展开后列出所有通道，点击手动切换 ──
function CameraChannelMenu({ channels, current, results, probing, liveStatus, liveStats, onSelect, onProbe }) {
  const others = channels.filter((c) => c.name !== current.name);
  const rows = channels.map((c) => {
    if (c.name === current.name) {
      if (liveStatus === "playing") {
        return { key: c.name, name: c.name, url: c.url, current: true, tone: "ok", text: channelMetric(liveStats?.kbps, liveStats?.rtt),
          sub: `延迟 ${liveStats?.rtt != null ? liveStats.rtt.toFixed(0) : "--"} ms · 丢包 ${liveStats?.loss != null ? liveStats.loss.toFixed(1) : "--"}%`, tip: "正在使用（实时数据）" };
      }
      if (liveStatus === "error") return { key: c.name, name: c.name, url: c.url, current: true, tone: "bad", text: "不通", sub: "自动重连中", tip: "连接失败，自动重连中" };
      return { key: c.name, name: c.name, url: c.url, current: true, tone: "wait", text: "连接中", sub: "", tip: "正在连接" };
    }
    const r = results[c.name];
    const busy = probing[c.name];
    if (!r) return { key: c.name, name: c.name, url: c.url, tone: "wait", text: busy ? "测试中" : "--", sub: busy ? "" : "尚未测试", tip: "点击切换到这个通道" };
    if (r.ok) return { key: c.name, name: c.name, url: c.url, tone: "ok", text: channelMetric(r.kbps, r.rtt),
      sub: `延迟 ${r.rtt != null ? r.rtt.toFixed(0) : "--"} ms · 丢包 ${r.loss.toFixed(1)}% · ${timeOf(r.at)}${busy ? " · 重测中" : ""}`, tip: "点击切换到这个通道" };
    return { key: c.name, name: c.name, url: c.url, tone: "bad", text: "不通", sub: `${r.reason} · ${timeOf(r.at)}`, tip: `不通：${r.reason}` };
  });
  const curRow = rows.find((r) => r.current);
  const short = curRow.tone === "ok" ? channelMetric(liveStats?.kbps, liveStats?.rtt) : curRow.text;
  return (
    <ChannelDropdown
      className="dt-dd-camera"
      align="right"
      triggerTitle="切换相机通道"
      trigger={<span className="dt-dd-pill"><i className={`dt-cam-dot ${curRow.tone}`} />{current.name}<b className="dt-num">{short}</b></span>}
      title="相机通道 · 手动切换"
      rows={rows}
      onSelect={onSelect}
      refreshing={others.some((c) => probing[c.name])}
      onRefresh={() => others.forEach(onProbe)}
    />
  );
}

function WebRtcCamera({ baseUrl, active, onStatus, onStats, fit = "cover", focusY = CAMERA_FOCUS_Y }) {
  const videoRef = useRef(null);
  const [status, setStatus] = useState("idle"); // idle | connecting | playing | error
  const [errMsg, setErrMsg] = useState("");
  const [frozenSec, setFrozenSec] = useState(0);
  const [hasFrame, setHasFrame] = useState(false); // 出过画面后，重连期间保留最后一帧
  const [stats, setStats] = useState(null);
  const onStatusRef = useRef(onStatus);
  onStatusRef.current = onStatus;
  const onStatsRef = useRef(onStats);
  onStatsRef.current = onStats;

  useEffect(() => {
    onStatusRef.current?.(status);
  }, [status]);

  // 实时传输数据回报给通道开关（显示当前通道带宽）
  useEffect(() => {
    onStatsRef.current?.(stats);
  }, [stats]);

  // 换通道时清掉上一个通道的画面和统计
  useEffect(() => {
    setHasFrame(false);
    setStats(null);
  }, [baseUrl]);

  useEffect(() => {
    if (!active || !baseUrl) {
      setStatus("idle");
      return;
    }
    const whepUrl = new URL("whep", baseUrl.endsWith("/") ? baseUrl : `${baseUrl}/`).toString();
    const host = new URL(whepUrl).hostname;
    let disposed = false;
    let pc = null;
    let sessionUrl = null;
    let retryTimer = null;
    let watchTimer = null;
    let statsTimer = null;
    let failCount = 0;

    const teardown = () => {
      clearInterval(watchTimer);
      clearInterval(statsTimer);
      watchTimer = null;
      statsTimer = null;
      if (sessionUrl) {
        fetch(sessionUrl, { method: "DELETE" }).catch(() => {}); // 通知 MediaMTX 释放会话
        sessionUrl = null;
      }
      if (pc) {
        pc.ontrack = null;
        pc.onconnectionstatechange = null;
        pc.close();
        pc = null;
      }
    };

    const retry = (msg) => {
      if (disposed || retryTimer) return;
      teardown();
      setStatus("error");
      setErrMsg(msg || "连接失败");
      setStats(null);
      failCount += 1;
      const delay = Math.min(CAMERA_RETRY_MS * 2 ** (failCount - 1), CAMERA_RETRY_MAX_MS);
      console.warn(`⚠️ 相机：${msg || "连接失败"}，${(delay / 1000).toFixed(0)} 秒后重连（第 ${failCount} 次）`);
      retryTimer = setTimeout(() => {
        retryTimer = null;
        start();
      }, delay);
    };

    const waitIce = (p) =>
      new Promise((resolve) => {
        if (p.iceGatheringState === "complete") return resolve();
        const t = setTimeout(resolve, CAMERA_ICE_GATHER_TIMEOUT_MS);
        p.addEventListener("icegatheringstatechange", () => {
          if (p.iceGatheringState === "complete") {
            clearTimeout(t);
            resolve();
          }
        });
      });

    // 每 2 秒读一次 WebRTC 统计：码率、帧率、丢包率、往返延迟、实际走的协议
    const startStats = (p) => {
      if (!CAMERA_STATS) return;
      let prev = null;
      statsTimer = setInterval(async () => {
        if (p !== pc) return;
        try {
          const report = await p.getStats();
          let inbound = null;
          let pair = null;
          const byId = new Map();
          report.forEach((r) => {
            byId.set(r.id, r);
            if (r.type === "inbound-rtp" && (r.kind === "video" || r.mediaType === "video")) inbound = r;
            if (r.type === "candidate-pair" && (r.nominated || r.selected) && r.state === "succeeded") pair = r;
          });
          if (!inbound) return;
          const now = inbound.timestamp;
          const bytes = inbound.bytesReceived || 0;
          const recv = inbound.packetsReceived || 0;
          const lost = Math.max(0, inbound.packetsLost || 0);
          let kbps = null;
          let loss = null;
          if (prev) {
            const dt = (now - prev.t) / 1000;
            if (dt > 0) kbps = ((bytes - prev.bytes) * 8) / dt / 1000;
            const dRecv = recv - prev.recv;
            const dLost = lost - prev.lost;
            if (dRecv + dLost > 0) loss = (dLost / (dRecv + dLost)) * 100;
          }
          prev = { t: now, bytes, recv, lost };
          const remote = pair ? byId.get(pair.remoteCandidateId) : null;
          setStats({
            proto: remote?.protocol ? remote.protocol.toUpperCase() : null,
            kbps,
            fps: inbound.framesPerSecond ?? null,
            loss,
            rtt: pair?.currentRoundTripTime != null ? pair.currentRoundTripTime * 1000 : null,
            res: inbound.frameWidth ? `${inbound.frameWidth}×${inbound.frameHeight}` : null,
          });
        } catch { /* 统计失败不影响播放 */ }
      }, 2000);
    };

    const start = async () => {
      if (disposed) return;
      setStatus((st) => (st === "playing" ? st : "connecting"));
      try {
        const p = new RTCPeerConnection();
        pc = p;
        p.addTransceiver("video", { direction: "recvonly" });
        p.addTransceiver("audio", { direction: "recvonly" });
        p.ontrack = (e) => {
          // 接收缓冲：留一点余量吸收网络抖动（0 为最低延迟）
          try {
            if ("jitterBufferTarget" in e.receiver) e.receiver.jitterBufferTarget = CAMERA_JITTER_BUFFER_MS;
            if ("playoutDelayHint" in e.receiver) e.receiver.playoutDelayHint = CAMERA_JITTER_BUFFER_MS / 1000;
          } catch { /* 旧浏览器不支持，忽略 */ }
          const v = videoRef.current;
          if (!v || e.track.kind !== "video") return;
          v.srcObject = e.streams[0] || new MediaStream([e.track]);
          v.play().catch(() => {});
        };
        // disconnected 多为短暂丢包，浏览器会自己恢复，不立即重连；failed 才重连
        p.onconnectionstatechange = () => {
          if (p !== pc) return;
          if (p.connectionState === "failed" || p.connectionState === "closed") retry("连接断开，正在重连");
        };

        await p.setLocalDescription(await p.createOffer());
        await waitIce(p); // 一次性把 ICE 候选写进 offer，省掉 WHEP 的 PATCH 流程
        if (disposed || p !== pc) return;

        const res = await fetch(whepUrl, {
          method: "POST",
          headers: { "Content-Type": "application/sdp" },
          body: p.localDescription.sdp,
        });
        if (res.status !== 201) {
          const notMtx = !/mediamtx/i.test(res.headers.get("Server") || "") && res.status !== 404;
          throw new Error(
            res.status === 404 ? "该地址上没有找到视频流（推流端未启动，或这个端口不是 MediaMTX）"
              : notMtx ? `HTTP ${res.status}：这个地址/端口很可能不是 MediaMTX 视频服务（例如转发的是 rosbridge）`
              : `MediaMTX 返回 HTTP ${res.status}`
          );
        }
        const loc = res.headers.get("Location");
        if (loc) sessionUrl = new URL(loc, whepUrl).toString();
        let answer = await res.text();
        if (disposed || p !== pc) return;
        if (CAMERA_PIN_HOST) answer = pinCandidates(answer, host);
        await p.setRemoteDescription({ type: "answer", sdp: answer });
        startStats(p);

        // 看门狗：用 video.currentTime 是否前进判断画面是否真的在动
        const startedAt = performance.now();
        let lastTime = -1;
        let lastMoveAt = 0;
        watchTimer = setInterval(() => {
          const v = videoRef.current;
          if (!v) return;
          const now = performance.now();
          if (v.currentTime > 0 && v.currentTime !== lastTime) {
            lastTime = v.currentTime;
            lastMoveAt = now;
            failCount = 0;
            setStatus("playing");
            setHasFrame(true);
            setErrMsg("");
            setFrozenSec(0);
          } else if (lastMoveAt === 0) {
            if (now - startedAt > CAMERA_CONNECT_TIMEOUT_MS) retry("长时间未收到画面，正在重连");
          } else {
            const frozen = now - lastMoveAt;
            setFrozenSec(Math.floor(frozen / 1000));
            if (frozen > CAMERA_FREEZE_MS) retry("画面卡住，正在重连");
          }
        }, 500);
      } catch (err) {
        console.warn("⚠️ WebRTC 相机连接失败:", err);
        const blocked = window.location.protocol === "https:" && whepUrl.startsWith("http:");
        retry(blocked ? "网页是 https 打开的，浏览器拦截了 http 视频地址" : err?.message === "Failed to fetch" ? `无法访问 ${whepUrl}` : err?.message);
      }
    };

    start();
    return () => {
      disposed = true;
      clearTimeout(retryTimer);
      teardown();
      const v = videoRef.current;
      if (v) v.srcObject = null;
    };
  }, [active, baseUrl]);

  const playing = status === "playing";
  const showVideo = playing || hasFrame;
  return (
    <div style={{ position: "absolute", inset: 0, background: "#000" }}>
      {/* 用透明度隐藏而不是 display:none：部分浏览器会暂停不可见视频的解码，导致看门狗误判卡死 */}
      <video
        ref={videoRef}
        muted
        autoPlay
        playsInline
        disablePictureInPicture
        onPlaying={() => { setStatus("playing"); setHasFrame(true); setErrMsg(""); }} // 首帧一解出来就显示，不等看门狗的 500 ms 轮询
        style={{ width: "100%", height: "100%", objectFit: fit, objectPosition: fit === "cover" ? `50% ${focusY}%` : "50% 50%", background: "#000", opacity: showVideo ? 1 : 0, filter: playing ? "none" : "grayscale(.6) brightness(.7)", transition: "filter .3s" }}
      />
      {!showVideo && (
        <div className="dt-cam-empty">
          <svg viewBox="0 0 24 24" width="34" height="34" fill="none" stroke="#34506e" strokeWidth="1.5">
            <rect x="3" y="6" width="13" height="12" rx="2" />
            <path d="M16 10l5-3v10l-5-3" />
          </svg>
          <span title={errMsg || undefined}>{status === "error" ? "相机未连接，正在重试" : "正在连接相机画面…"}</span>
          <small>{baseUrl}</small>
        </div>
      )}
      {showVideo && !playing && <div className="dt-cam-stale">信号波动，正在重新连接…</div>}
      {playing && frozenSec >= 2 && <div className="dt-cam-stale">画面卡顿 {frozenSec} 秒</div>}
    </div>
  );
}

// ── 天地图 Leaflet 组件（起点/终点/当前位置/规划路线/已行驶轨迹） ──
function TiandituMap({ ego, startPoint, goalPoint, globalPath, trail, onLoaded }) {
  const onLoadedRef = useRef(onLoaded);
  onLoadedRef.current = onLoaded;
  const mapContainerRef = useRef(null);
  const mapInstanceRef = useRef(null);
  const egoMarkerRef = useRef(null);
  const startMarkerRef = useRef(null);
  const goalMarkerRef = useRef(null);
  const globalPathGlowRef = useRef(null);
  const globalPathLineRef = useRef(null);
  const trailLineRef = useRef(null);
  const [mapReady, setMapReady] = useState(false);

  useEffect(() => {
    const activeKey =
      typeof TIANDITU_TK !== "undefined" && TIANDITU_TK !== "YOUR_TIANDITU_KEY_HERE"
        ? TIANDITU_TK
        : "6cab7569cbef175735a7864e8478beaa";

    if (!document.getElementById("leaflet-css")) {
      const link = document.createElement("link");
      link.id = "leaflet-css";
      link.rel = "stylesheet";
      link.href = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css";
      document.head.appendChild(link);
    }

    const initMap = () => {
      try {
        if (!window.L || !mapContainerRef.current) return;

        if (mapContainerRef.current._leaflet_id) {
          mapContainerRef.current._leaflet_id = null;
        }

        if (mapInstanceRef.current) return;

        const L = window.L;
        const lat = ego?.lat && !isNaN(ego.lat) ? ego.lat : MAP_ORIGIN.lat;
        const lng = ego?.lng && !isNaN(ego.lng) ? ego.lng : MAP_ORIGIN.lng;

        const map = L.map(mapContainerRef.current, {
          center: [lat, lng],
          zoom: 17,
          zoomControl: false,
          attributionControl: false,
          renderer: L.canvas(),
        });

        const imgUrl = `https://t{s}.tianditu.gov.cn/DataServer?T=img_w&x={x}&y={y}&l={z}&tk=${activeKey}`;
        const ciaUrl = `https://t{s}.tianditu.gov.cn/DataServer?T=cia_w&x={x}&y={y}&l={z}&tk=${activeKey}`;

        const imgLayer = L.tileLayer(imgUrl, { subdomains: ["0", "1", "2", "3", "4", "5", "6", "7"], maxZoom: 18 }).addTo(map);
        // 可视范围内的卫星瓦片全部加载完成（开场画面据此判断“卫星地图就绪”）
        imgLayer.once("load", () => onLoadedRef.current?.());
        L.tileLayer(ciaUrl, { subdomains: ["0", "1", "2", "3", "4", "5", "6", "7"], maxZoom: 18 }).addTo(map);

        // 带朝向箭头与呼吸光圈的自车标记
        const carIcon = L.divIcon({
          className: "dt-ego-marker",
          html: `<div class="dt-ego"><span class="dt-ego-pulse"></span><span class="dt-ego-core"></span><svg class="dt-ego-arrow" viewBox="0 0 24 24" width="26" height="26"><path d="M12 2 L20 21 L12 16.5 L4 21 Z" fill="#34f5a4" stroke="#ffffff" stroke-width="1.6" stroke-linejoin="round"/></svg></div>`,
          iconSize: [40, 40],
          iconAnchor: [20, 20],
        });
        egoMarkerRef.current = L.marker([lat, lng], { icon: carIcon, zIndexOffset: 1000 }).addTo(map);

        mapInstanceRef.current = map;
        setMapReady(true);
        setTimeout(() => map.invalidateSize(), 300);
      } catch (err) {
        console.error("Map initialization error:", err);
      }
    };

    if (window.L) {
      initMap();
    } else {
      let script = document.getElementById("leaflet-js");
      if (!script) {
        script = document.createElement("script");
        script.id = "leaflet-js";
        script.src = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js";
        document.body.appendChild(script);
      }

      const checkAndInit = () => {
        if (window.L) initMap();
        else setTimeout(checkAndInit, 50);
      };
      script.onload = () => checkAndInit();
    }

    return () => {
      if (mapInstanceRef.current) {
        mapInstanceRef.current.remove();
        mapInstanceRef.current = null;
      }
    };
  }, []);

  useEffect(() => {
    const map = mapInstanceRef.current;
    if (map && egoMarkerRef.current && ego?.lat && ego?.lng) {
      const newLatLng = [ego.lat, ego.lng];
      egoMarkerRef.current.setLatLng(newLatLng);
      // 只有小车跑出视野中心较远（>60 m）才平移底图，避免高频更新让卫星图底图不停跳动
      const center = map.getCenter();
      if (map.distance(center, newLatLng) > 60) {
        map.panTo(newLatLng, { animate: false });
      }
    }
  }, [ego?.lat, ego?.lng, mapReady]);

  // 自车箭头朝向：ROS ENU 航向（0=正东，逆时针）→ 屏幕旋转（0=正北，顺时针）
  useEffect(() => {
    const el = egoMarkerRef.current?.getElement?.()?.querySelector(".dt-ego-arrow");
    if (el && Number.isFinite(ego?.yaw)) {
      el.style.transform = `rotate(${90 - (ego.yaw * 180) / Math.PI}deg)`;
    }
  }, [ego?.yaw, ego?.lat, ego?.lng, mapReady]);

  useEffect(() => {
    if (!mapInstanceRef.current || !window.L || !startPoint?.lat) return;
    const L = window.L;

    const startIcon = L.divIcon({
      className: "dt-pin-marker",
      html: `<div class="dt-pin dt-pin-start"><span>起</span></div>`,
      iconSize: [28, 28],
      iconAnchor: [14, 34],
    });

    if (startMarkerRef.current) {
      startMarkerRef.current.setLatLng([startPoint.lat, startPoint.lng]);
    } else {
      startMarkerRef.current = L.marker([startPoint.lat, startPoint.lng], { icon: startIcon }).addTo(mapInstanceRef.current);
    }
  }, [startPoint, mapReady]);

  useEffect(() => {
    if (!mapInstanceRef.current || !window.L || !goalPoint?.lat) return;
    const L = window.L;

    const goalIcon = L.divIcon({
      className: "dt-pin-marker",
      html: `<div class="dt-pin dt-pin-goal"><span>终</span></div>`,
      iconSize: [28, 28],
      iconAnchor: [14, 34],
    });

    if (goalMarkerRef.current) {
      goalMarkerRef.current.setLatLng([goalPoint.lat, goalPoint.lng]);
    } else {
      goalMarkerRef.current = L.marker([goalPoint.lat, goalPoint.lng], { icon: goalIcon }).addTo(mapInstanceRef.current);
    }
  }, [goalPoint, mapReady]);

  // 规划路线：外层半透明光晕 + 内层实线
  useEffect(() => {
    if (!mapInstanceRef.current || !window.L) return;
    const latLngs = (globalPath || [])
      .filter((point) => Number.isFinite(point.lat) && Number.isFinite(point.lng))
      .map((point) => [point.lat, point.lng]);
    if (latLngs.length < 2) {
      globalPathGlowRef.current?.setLatLngs([]);
      globalPathLineRef.current?.setLatLngs([]);
      return;
    }
    const L = window.L;

    if (globalPathLineRef.current) {
      globalPathGlowRef.current.setLatLngs(latLngs);
      globalPathLineRef.current.setLatLngs(latLngs);
    } else {
      globalPathGlowRef.current = L.polyline(latLngs, {
        color: "#3fd4ff", weight: 14, opacity: 0.22, lineCap: "round", lineJoin: "round", interactive: false,
      }).addTo(mapInstanceRef.current);
      globalPathLineRef.current = L.polyline(latLngs, {
        color: "#3fd4ff", weight: 4, opacity: 0.95, lineCap: "round", lineJoin: "round", interactive: false,
      }).addTo(mapInstanceRef.current);
    }
    globalPathGlowRef.current.bringToFront();
    globalPathLineRef.current.bringToFront();
    trailLineRef.current?.bringToFront();
  }, [globalPath, mapReady]);

  // 已行驶轨迹（绿色）
  useEffect(() => {
    if (!mapInstanceRef.current || !window.L) return;
    const latLngs = (trail || []).map((p) => [p.lat, p.lng]);
    if (trailLineRef.current) {
      trailLineRef.current.setLatLngs(latLngs);
    } else if (latLngs.length >= 2) {
      trailLineRef.current = window.L.polyline(latLngs, {
        color: "#34f5a4", weight: 4, opacity: 0.9, lineCap: "round", lineJoin: "round", interactive: false,
      }).addTo(mapInstanceRef.current);
    }
    trailLineRef.current?.bringToFront();
  }, [trail, mapReady]);

  return (
    <div className="dt-map" style={{ width: "100%", height: "100%", position: "relative", isolation: "isolate", zIndex: 0 }}>
      <div ref={mapContainerRef} style={{ width: "100%", height: "100%", background: "#0b1322" }} />
      <div className="dt-map-vignette" />
    </div>
  );
}

function MapLegend() {
  return (
    <div className="dt-map-legend">
      <span><i style={{ background: "#3fd4ff" }} />规划路线</span>
      <span><i style={{ background: "#34f5a4" }} />已行驶</span>
      <span><i className="dot" style={{ background: "#1f9bff" }} />起点</span>
      <span><i className="dot" style={{ background: "#ff4d5e" }} />终点</span>
    </div>
  );
}

function Scene({ viewMode, pointSize, pcdConfig, carOffset, currentCar, sim, poseMode, setPoseMode, publishPose, teleopActive, onAssetStatus, intro, routePoints, onObstacleAlert, onPlaceObstacle, onRemoveObstacle }) {
  const containerRef = useRef(null);
  // 点云地面高度查询函数（ROS 坐标 x, y → 地面 z），点云加载完成后才有
  const groundZAtRef = useRef(null);
  // 仿真障碍物：点击放置 / 删除（由 App 转发给工控机节点）
  const onPlaceObstacleRef = useRef(onPlaceObstacle);
  onPlaceObstacleRef.current = onPlaceObstacle;
  const onRemoveObstacleRef = useRef(onRemoveObstacle);
  onRemoveObstacleRef.current = onRemoveObstacle;
  const onObstacleAlertRef = useRef(onObstacleAlert);
  onObstacleAlertRef.current = onObstacleAlert;
  const teleopRef = useRef(teleopActive);
  teleopRef.current = teleopActive;
  const onAssetStatusRef = useRef(onAssetStatus);
  onAssetStatusRef.current = onAssetStatus;
  const internals = useRef(null);
  const simRef = useRef(null);
  simRef.current = sim;
  const poseModeRef = useRef(poseMode);
  poseModeRef.current = poseMode;
  const publishPoseRef = useRef(publishPose);
  publishPoseRef.current = publishPose;
  // 车道中心线（ROS 坐标），用于把网页画的起/终点吸附到车道中心线并取正确朝向。
  const laneCenterlinesRef = useRef([]);

  const propsRef = useRef({ viewMode, pointSize, pcdConfig, carOffset, currentCar, intro, routePoints });
  propsRef.current = { viewMode, pointSize, pcdConfig, carOffset, currentCar, intro, routePoints };


  const freeCamera = useRef({ theta: 0.8, phi: 0.6, dist: 40, target: new THREE.Vector3(0, 0, 0) });
  const mouse = useRef({ down: false, button: 0, lx: 0, ly: 0 });
  const keysPressed = useRef({});

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const W = container.clientWidth || 800;
    const H = container.clientHeight || 500;

    // 关闭 MSAA：点云是自发光散点，抗锯齿收益很小，而 4x MSAA 会让显卡带宽翻数倍
    const renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: "high-performance" });
    renderer.setSize(W, H);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, MAX_PIXEL_RATIO));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;
    renderer.shadowMap.enabled = SHADOWS;
    container.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(SCENE_BG);
    scene.fog = new THREE.FogExp2(SCENE_BG, 0.012);

    const camera = new THREE.PerspectiveCamera(55, W / H, 0.1, 500);
    camera.position.set(0, 80, 60); // 从高空开场，入场时俯冲到车辆

    // 点云扫描波的共享 uniform（animate 中逐帧更新）
    const scanUniforms = {
      uScanCenter: { value: new THREE.Vector2() },
      uScanRadius: { value: 0 },
      uScanAlpha: { value: 0 },
    };

    scene.add(new THREE.AmbientLight(0x5577aa, 0.9));
    scene.add(new THREE.HemisphereLight(0x88bbff, 0x0a0f1a, 0.6));
    const dir = new THREE.DirectionalLight(0xffeedd, 1.3);
    dir.position.set(30, 50, 20);
    dir.castShadow = SHADOWS;
    scene.add(dir);

    // ── 后处理：多重采样渲染目标 + 泛光 ──
    const pr = renderer.getPixelRatio();
    let composer = null;
    if (BLOOM.enabled) {
      composer = new EffectComposer(
        renderer,
        new THREE.WebGLRenderTarget(W * pr, H * pr, { type: THREE.HalfFloatType, samples: 4 })
      );
      composer.setSize(W, H);
      composer.addPass(new RenderPass(scene, camera));
      composer.addPass(new UnrealBloomPass(new THREE.Vector2(W, H), BLOOM.strength, BLOOM.radius, BLOOM.threshold));
      composer.addPass(new OutputPass());
    }

    const grid = new THREE.GridHelper(240, 120, 0x1d3d5c, 0x10233a);
    grid.material.transparent = true;
    grid.material.opacity = 0.55;
    grid.material.depthWrite = false;
    grid.position.y = 0.01;
    scene.add(grid);

    const pointCloudGroup = new THREE.Group();
    scene.add(pointCloudGroup);

    // OSM 车道与 PCD 放在同一个容器中，共享地图平移/旋转校准。
    const laneMapGroup = new THREE.Group();
    pointCloudGroup.add(laneMapGroup);
    const laneMaterial = new THREE.LineBasicMaterial({
      color: LANE_EDGE_COLOR,
      transparent: true,
      opacity: 0.75,
      depthWrite: false,
    });
    const roadMaterial = new THREE.MeshBasicMaterial({
      color: ROAD_COLOR,
      transparent: true,
      opacity: ROAD_OPACITY,
      depthWrite: false,
      side: THREE.DoubleSide,
      polygonOffset: true,
      polygonOffsetFactor: -1,
      polygonOffsetUnits: -1,
    });
    const osmAbortController = new AbortController();

    // ── 车道高度空间索引：路径只读取车道高度，绝不改动水平坐标 ──
    // centerline 使用 ROS 坐标 (x 前, y 左, z 高)，路径使用 Three 坐标 (x=ROS.y, z=ROS.x)。
    let mapHasEle = false;
    let laneHeightCells = new Map();
    const rebuildLaneHeightIndex = () => {
      const cells = new Map();
      const add = (key, segment) => {
        let list = cells.get(key);
        if (!list) cells.set(key, (list = []));
        list.push(segment);
      };
      for (const line of laneCenterlinesRef.current) {
        // 无 OSM 高程时，同一个 lanelet 使用点云地面的中位高度；不会再因逐点
        // 命中路沿、墙面或离群点而产生截图中的上下锯齿。
        const sampled = mapHasEle ? [] : line
          .map((point) => groundAt(point.x, point.y))
          .filter((height) => height != null && Number.isFinite(height))
          .sort((a, b) => a - b);
        const laneGround = sampled.length ? sampled[Math.floor(sampled.length / 2)] : 0;
        for (let i = 0; i < line.length - 1; i++) {
          const a = line[i];
          const b = line[i + 1];
          const length = Math.hypot(b.x - a.x, b.y - a.y);
          const steps = Math.max(1, Math.ceil(length / (PATH_LANE_HEIGHT_CELL * 0.5)));
          const segment = {
            a,
            b,
            heightA: mapHasEle ? a.z : laneGround,
            heightB: mapHasEle ? b.z : laneGround,
          };
          const occupied = new Set();
          for (let k = 0; k <= steps; k++) {
            const t = k / steps;
            const x = a.x + (b.x - a.x) * t;
            const y = a.y + (b.y - a.y) * t;
            occupied.add(`${Math.floor(x / PATH_LANE_HEIGHT_CELL)},${Math.floor(y / PATH_LANE_HEIGHT_CELL)}`);
          }
          occupied.forEach((key) => add(key, segment));
        }
      }
      laneHeightCells = cells;
      laneHeightCache.clear();
    };

    // 引导带每帧要查约 50 个点的车道高度，每次都要遍历邻近网格、新建 Set；
    // 按 0.25 m 位置 + 22.5° 方向量化后缓存，车道高度本身是静态的，命中率很高。
    const laneHeightCache = new Map();
    const laneHeightForPoint = (point, previous = point, next = point) => {
      if (!point || !laneHeightCells.size) return null;
      const headingBin = Math.round(Math.atan2(next.x - previous.x, next.z - previous.z) / (Math.PI / 8));
      const cacheKey = `${Math.round(point.x * 4)},${Math.round(point.z * 4)},${headingBin}`;
      if (laneHeightCache.has(cacheKey)) return laneHeightCache.get(cacheKey);
      if (laneHeightCache.size > 50000) laneHeightCache.clear();
      const result = laneHeightForPointRaw(point, previous, next);
      laneHeightCache.set(cacheKey, result);
      return result;
    };
    const laneHeightForPointRaw = (point, previous, next) => {
      const rosX = point.z;
      const rosY = point.x;
      const cx = Math.floor(rosX / PATH_LANE_HEIGHT_CELL);
      const cy = Math.floor(rosY / PATH_LANE_HEIGHT_CELL);
      const radius = Math.ceil(PATH_LANE_HEIGHT_DISTANCE / PATH_LANE_HEIGHT_CELL) + 1;
      const candidates = new Set();
      for (let dx = -radius; dx <= radius; dx++) {
        for (let dy = -radius; dy <= radius; dy++) {
          for (const segment of laneHeightCells.get(`${cx + dx},${cy + dy}`) || []) candidates.add(segment);
        }
      }

      // 路径切线换回 ROS 坐标，用方向惩罚避免在路口读取横向/对向车道的高度。
      const pathDx = next.z - previous.z;
      const pathDy = next.x - previous.x;
      const pathLength = Math.hypot(pathDx, pathDy);
      let best = null;
      for (const segment of candidates) {
        const { a, b } = segment;
        const abx = b.x - a.x;
        const aby = b.y - a.y;
        const len2 = abx * abx + aby * aby;
        if (len2 < 1e-8) continue;
        const t = Math.max(0, Math.min(1, ((rosX - a.x) * abx + (rosY - a.y) * aby) / len2));
        const x = a.x + abx * t;
        const y = a.y + aby * t;
        const distance = Math.hypot(rosX - x, rosY - y);
        if (distance > PATH_LANE_HEIGHT_DISTANCE) continue;
        const alignment = pathLength > 1e-4
          ? Math.max(-1, Math.min(1, (pathDx * abx + pathDy * aby) / (pathLength * Math.sqrt(len2))))
          : 1;
        const score = distance + (1 - alignment) * 1.5;
        if (!best || score < best.score) {
          best = { score, height: segment.heightA + (segment.heightB - segment.heightA) * t };
        }
      }
      return best ? best.height : null;
    };

    // ── 全局路线：起点到终点整条路线的发光条带 ──
    const routeMaterial = new THREE.MeshBasicMaterial({
      color: ROUTE_COLOR, transparent: true, opacity: 0.45, depthWrite: false, side: THREE.DoubleSide,
      polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
    });
    const routeEdgeMaterial = new THREE.LineBasicMaterial({ color: ROUTE_COLOR, transparent: true, opacity: 0.95, depthWrite: false });
    const routeMesh = new THREE.Mesh(new THREE.BufferGeometry(), routeMaterial);
    const routeEdges = new THREE.LineSegments(new THREE.BufferGeometry(), routeEdgeMaterial);
    routeMesh.renderOrder = 12;
    routeEdges.renderOrder = 13;
    routeMesh.frustumCulled = routeEdges.frustumCulled = false;
    routeMesh.visible = routeEdges.visible = false;
    laneMapGroup.add(routeMesh, routeEdges);
    let lastRouteProp = null;   // 上一次收到的路径（引用）
    let builtRoutePts = null;   // 当前条带所用的路径
    let routeStartSeg = 0;      // 条带当前从第几段开始显示（跟随车辆）
    const routeLocal = new THREE.Vector3();
    // 新路径与已绘制路线是否“同一条”：终点相同，且新路径起点仍落在旧路线上。
    // Autoware 每次重新发布路径时点位都会略有变化，若每次都重建条带，起点会一跳一跳。
    const sameRoute = (oldPts, newPts) => {
      if (!oldPts || !newPts || oldPts.length < 2 || newPts.length < 2) return false;
      const oe = oldPts[oldPts.length - 1];
      const ne = newPts[newPts.length - 1];
      if (Math.hypot(oe.x - ne.x, oe.z - ne.z) > 1) return false;
      const f = newPts[0];
      let md = Infinity;
      for (const pt of oldPts) md = Math.min(md, Math.hypot(pt.x - f.x, pt.z - f.z));
      return md < 1.5;
    };
    const buildRoute = (pts) => {
      routeMesh.geometry.dispose();
      routeEdges.geometry.dispose();
      routeMesh.geometry = new THREE.BufferGeometry();
      routeEdges.geometry = new THREE.BufferGeometry();
      if (!pts || pts.length < 2) {
        routeMesh.visible = routeEdges.visible = false;
        return;
      }
      const verts = [];
      const idx = [];
      const edges = [];
      pts.forEach((pt, i) => {
        const a = pts[Math.max(0, i - 1)];
        const b = pts[Math.min(pts.length - 1, i + 1)];
        const dx = b.x - a.x;
        const dz = b.z - a.z;
        const len = Math.hypot(dx, dz) || 1;
        const nx = (dz / len) * ROUTE_HALF_WIDTH;
        const nz = (-dx / len) * ROUTE_HALF_WIDTH;
        // 水平坐标完全沿用 Autoware 路径；高度直接取附近车道中心线。
        const laneY = laneHeightForPoint(pt, a, b);
        const baseY = laneY != null ? laneY : (Number.isFinite(pt.y) ? pt.y : 0);
        const y = baseY + ROUTE_Y;
        verts.push(pt.x + nx, y, pt.z + nz, pt.x - nx, y, pt.z - nz);
        if (i > 0) {
          const k = i * 2;
          idx.push(k - 2, k - 1, k, k - 1, k + 1, k);
          edges.push(...verts.slice((k - 2) * 3, (k - 1) * 3), ...verts.slice(k * 3, (k + 1) * 3));
          edges.push(...verts.slice((k - 1) * 3, k * 3), ...verts.slice((k + 1) * 3, (k + 2) * 3));
        }
      });
      routeMesh.geometry.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));
      routeMesh.geometry.setIndex(idx);
      routeEdges.geometry.setAttribute("position", new THREE.Float32BufferAttribute(edges, 3));
      routeMesh.visible = routeEdges.visible = true;
    };

    // ── 车道/路面贴地；路径高度则从稳定的车道中心高度索引读取 ──
    let roadMesh = null;     // 路面
    let laneEdgeLines = null; // 车道边线
    const groundCache = new Map();
    // 查询点云地面高度（ROS 坐标），按 0.5 m 网格缓存；点云未加载时返回 null
    const groundAt = (rosX, rosY) => {
      const fn = groundZAtRef.current;
      if (!fn) return null;
      const key = `${Math.round(rosX * 2)},${Math.round(rosY * 2)}`;
      if (groundCache.has(key)) return groundCache.get(key);
      const v = fn(rosX, rosY);
      const g = v != null && Number.isFinite(v) ? v : null;
      groundCache.set(key, g);
      return g;
    };
    // 把一个几何体的每个顶点高度改为“地面高度 + 抬升量”（Three: x=ROS.y, z=ROS.x）
    // 该处没有点云时沿用相邻顶点的地面高度，避免出现掉到 0 的尖刺
    const drapeGeometry = (geometry, lift) => {
      const pos = geometry?.getAttribute("position");
      if (!pos) return;
      let last = null;
      for (let i = 0; i < pos.count; i++) {
        const g = groundAt(pos.getZ(i), pos.getX(i));
        if (g != null) last = g;
        if (last != null) pos.setY(i, last + lift);
      }
      pos.needsUpdate = true;
      geometry.computeBoundingSphere();
    };
    // OSM 与点云异步加载，谁后到都调用一次：两者都就绪时完成贴地
    const drapeLaneMap = () => {
      if (!groundZAtRef.current) return;
      if (!mapHasEle) {
        if (roadMesh) drapeGeometry(roadMesh.geometry, ROAD_SURFACE_Y);
        if (laneEdgeLines) drapeGeometry(laneEdgeLines.geometry, LANE_EDGE_Y);
      }
      rebuildLaneHeightIndex();
      if (builtRoutePts) buildRoute(builtRoutePts);
      if (roadMesh || laneEdgeLines) console.info(mapHasEle ? "🟫 路径已对齐 OSM 车道高程" : "🟫 路径已对齐车道的稳定地面高度");
    };

    fetch("/map.osm", { signal: osmAbortController.signal })
      .then((response) => {
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return response.text();
      })
      .then((xmlText) => {
        const xml = new DOMParser().parseFromString(xmlText, "application/xml");
        if (xml.querySelector("parsererror")) throw new Error("OSM XML 格式错误");

        const nodes = new Map();
        const rosNodes = new Map();
        xml.querySelectorAll("node").forEach((node) => {
          const tags = Object.fromEntries(
            Array.from(node.querySelectorAll(":scope > tag")).map((tag) => [tag.getAttribute("k"), tag.getAttribute("v")])
          );
          let rosX = Number(tags.local_x);
          let rosY = Number(tags.local_y);

          // 某些 Lanelet2 地图只有经纬度，使用当前地图原点转换为局部 ENU 米制坐标。
          if (!Number.isFinite(rosX) || !Number.isFinite(rosY)) {
            const lat = Number(node.getAttribute("lat"));
            const lon = Number(node.getAttribute("lon"));
            if (Number.isFinite(lat) && Number.isFinite(lon)) {
              const m = projectToMap(lat, lon, 0, MAP_PROJECTION);
              rosX = m.x;
              rosY = m.y;
            }
          }

          if (Number.isFinite(rosX) && Number.isFinite(rosY)) {
            const elevation = Number(tags.ele);
            if (tags.ele !== undefined && Number.isFinite(elevation)) mapHasEle = true;
            nodes.set(node.getAttribute("id"), {
              // 与 PCD 使用同一转换：(Three.x, Three.z) = (ROS.y, ROS.x)；y 为高程，绘制时再加抬升量
              x: rosY,
              y: Number.isFinite(elevation) ? elevation : 0,
              z: rosX,
            });
            rosNodes.set(node.getAttribute("id"), { x: rosX, y: rosY, z: Number.isFinite(elevation) ? elevation : 0 });
          }
        });

        const ways = new Map();
        xml.querySelectorAll("way").forEach((way) => {
          ways.set(way.getAttribute("id"), Array.from(way.querySelectorAll(":scope > nd")).map((nd) => nd.getAttribute("ref")));
        });

        // 只把 Lanelet 左右边界画成边线（中心线不画，画面更干净）。
        const edgeWayIds = new Set();
        let hasLanelets = false;
        // 所有车道路面合并进一个几何体，减少绘制调用
        const roadVertices = [];
        const roadIndices = [];

        // 构建车道中心线（ROS 坐标），供起/终点吸附到车道中心线并取正确朝向。
        // 与 Autoware mission planner 的 get_closest_lanelet 角度平局规则一致：
        // 终点压在重叠车道上时，规划器会按发布朝向选择角度最接近的车道。
        laneCenterlinesRef.current = [];
        xml.querySelectorAll("relation").forEach((relation) => {
          const tags = Object.fromEntries(
            Array.from(relation.querySelectorAll(":scope > tag")).map((tag) => [tag.getAttribute("k"), tag.getAttribute("v")])
          );
          if (tags.type !== "lanelet") return;
          hasLanelets = true;
          const members = {};
          relation.querySelectorAll(":scope > member[type='way']").forEach((member) => {
            members[member.getAttribute("role")] = member.getAttribute("ref");
          });
          if (!members.left || !members.right) return;
          edgeWayIds.add(members.left);
          edgeWayIds.add(members.right);

          const toPts = (wayId) => (ways.get(wayId) || []).map((nodeId) => rosNodes.get(nodeId)).filter(Boolean);
          let left = toPts(members.left);
          let right = toPts(members.right);
          if (left.length < 2 || right.length < 2) return;
          // 左右边界顺序一致时逐点取中点；若方向相反则反转右边界。
          const dlx = left[left.length - 1].x - left[0].x;
          const dly = left[left.length - 1].y - left[0].y;
          const drx = right[right.length - 1].x - right[0].x;
          const dry = right[right.length - 1].y - right[0].y;
          const reversed = dlx * drx + dly * dry < 0;
          if (reversed) right = right.slice().reverse();
          const n = Math.min(left.length, right.length);
          const center = [];
          for (let i = 0; i < n; i++) {
            center.push({ x: (left[i].x + right[i].x) / 2, y: (left[i].y + right[i].y) / 2, z: (left[i].z + right[i].z) / 2 });
          }
          if (center.length >= 2) {
            laneCenterlinesRef.current.push(center);
          }

          // ── 路面：左右边界按弧长重采样到相同点数，拼成三角带 ──
          const toThree = (wayId) => (ways.get(wayId) || []).map((nodeId) => nodes.get(nodeId)).filter(Boolean);
          const leftT = toThree(members.left);
          let rightT = toThree(members.right);
          if (leftT.length < 2 || rightT.length < 2) return;
          if (reversed) rightT = rightT.slice().reverse();
          const count = Math.min(400, Math.max(leftT.length, rightT.length, 2));
          const l = resamplePolyline(leftT, count);
          const r = resamplePolyline(rightT, count);
          const base = roadVertices.length / 3;
          for (let i = 0; i < count; i++) {
            roadVertices.push(l[i].x, l[i].y + ROAD_SURFACE_Y, l[i].z);
            roadVertices.push(r[i].x, r[i].y + ROAD_SURFACE_Y, r[i].z);
          }
          for (let i = 0; i < count - 1; i++) {
            const a = base + i * 2;
            roadIndices.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
          }
        });

        if (roadVertices.length) {
          const roadGeometry = new THREE.BufferGeometry();
          roadGeometry.setAttribute("position", new THREE.Float32BufferAttribute(roadVertices, 3));
          roadGeometry.setIndex(roadIndices);
          const road = new THREE.Mesh(roadGeometry, roadMaterial);
          road.renderOrder = 5;
          laneMapGroup.add(road);
          roadMesh = road;
        }

        // 兼容非 Lanelet2 的普通 OSM 道路文件：所有 way 都画成线。
        if (!hasLanelets) ways.forEach((_, id) => edgeWayIds.add(id));

        const edgeVertices = [];
        edgeWayIds.forEach((wayId) => {
          const vertices = (ways.get(wayId) || []).map((nodeId) => nodes.get(nodeId)).filter(Boolean);
          for (let i = 1; i < vertices.length; i++) {
            const a = vertices[i - 1];
            const b = vertices[i];
            edgeVertices.push(a.x, a.y + LANE_EDGE_Y, a.z, b.x, b.y + LANE_EDGE_Y, b.z);
          }
        });
        if (edgeVertices.length) {
          const edgeGeometry = new THREE.BufferGeometry();
          edgeGeometry.setAttribute("position", new THREE.Float32BufferAttribute(edgeVertices, 3));
          const edges = new THREE.LineSegments(edgeGeometry, laneMaterial);
          edges.renderOrder = 15;
          laneMapGroup.add(edges);
          laneEdgeLines = edges;
        }
        rebuildLaneHeightIndex();
        // 路径可能比 OSM 先到；车道高度索引就绪后强制重建一次。
        if (builtRoutePts) buildRoute(builtRoutePts);
        drapeLaneMap(); // 点云若已先加载完，立即贴地
        onAssetStatusRef.current?.("map", { lanes: laneCenterlinesRef.current.length });
        console.log(`✅ OSM 车道加载完成：${laneCenterlinesRef.current.length} 条车道，${edgeWayIds.size} 条边线`);
      })
      .catch((err) => {
        if (err.name !== "AbortError") console.error("❌ OSM 车道加载失败，请检查 public/map.osm:", err);
      });

    const pcdLoader = new PCDLoader();
    let pcdPointsNode = null;

    pcdLoader.load(
      "/goal_map.pcd",
      (points) => {
        let positions = points.geometry.attributes.position;
        const rawPointCount = positions ? positions.count : 0;
        // 点数过多时均匀抽稀（点云是每帧开销最大的部分）
        if (positions && positions.count > PCD_MAX_POINTS) {
          const total = positions.count;
          const step = total / PCD_MAX_POINTS;
          const kept = new Float32Array(PCD_MAX_POINTS * 3);
          for (let k = 0; k < PCD_MAX_POINTS; k++) {
            const i = Math.floor(k * step);
            kept[k * 3] = positions.getX(i);
            kept[k * 3 + 1] = positions.getY(i);
            kept[k * 3 + 2] = positions.getZ(i);
          }
          const thinned = new THREE.BufferGeometry();
          thinned.setAttribute("position", new THREE.Float32BufferAttribute(kept, 3));
          points.geometry.dispose();
          points.geometry = thinned;
          positions = thinned.attributes.position;
          console.info(`☁️ 点云 ${total} 点，已抽稀为 ${PCD_MAX_POINTS} 点显示`);
        }
        if (positions) {
          for (let i = 0; i < positions.count; i++) {
            const x = positions.getX(i);
            const y = positions.getY(i);
            const z = positions.getZ(i);
            // ROS PCD (x 前, y 左, z 上) -> Three.js (ROS.y, ROS.z, ROS.x)。
            // 循环换轴的行列式为 +1，不包含镜像反射。
            positions.setXYZ(i, y, z, x);
          }
          positions.needsUpdate = true;
          points.geometry.computeBoundingBox();
          // PCDLoader 按 ROS 坐标预先算好了包围球，换轴后必须重算，
          // 否则 three.js 按旧位置判断“点云不在视野内”，近距离视角下整片点云被裁掉。
          points.geometry.computeBoundingSphere();

          // ── 地面高度索引：按 1 m 网格统计每格的低位高度（ROS 坐标，不含界面上的地图校准偏移） ──
          {
            const cellZs = new Map();
            for (let i = 0; i < positions.count; i++) {
              const rx = positions.getZ(i); // Three.z = ROS.x
              const ry = positions.getX(i); // Three.x = ROS.y
              const rz = positions.getY(i); // Three.y = ROS.z
              const key = `${Math.floor(rx / GROUND_CELL_M)},${Math.floor(ry / GROUND_CELL_M)}`;
              let arr = cellZs.get(key);
              if (!arr) cellZs.set(key, (arr = []));
              arr.push(rz);
            }
            const groundCells = new Map();
            cellZs.forEach((arr, key) => {
              arr.sort((a, b) => a - b);
              groundCells.set(key, arr[Math.floor((arr.length - 1) * GROUND_PERCENTILE)]);
            });
            const span = Math.ceil(GROUND_SEARCH_M / GROUND_CELL_M);
            const groundZAt = (x, y) => {
              const cx = Math.floor(x / GROUND_CELL_M);
              const cy = Math.floor(y / GROUND_CELL_M);
              const found = [];
              for (let dx = -span; dx <= span; dx++) {
                for (let dy = -span; dy <= span; dy++) {
                  const v = groundCells.get(`${cx + dx},${cy + dy}`);
                  if (v !== undefined) found.push(v);
                }
              }
              if (!found.length) return null;
              found.sort((a, b) => a - b);
              return found[Math.floor(found.length / 2)];
            };
            groundZAtRef.current = groundZAt;
            groundCache.clear();
            drapeLaneMap(); // 车道若已先加载完，现在贴地
            console.info(`🟫 地面高度索引完成：${groundCells.size} 个网格`);
          }

          // ── 按高度渐变着色（取 2%~98% 分位，排除离群点） ──
          const count = positions.count;
          if (count > 0) {
            const heights = new Float32Array(count);
            for (let i = 0; i < count; i++) heights[i] = positions.getY(i);
            const sorted = heights.slice().sort();
            const lo = sorted[Math.floor(count * 0.02)];
            const hi = sorted[Math.min(count - 1, Math.floor(count * 0.98))];
            const span = Math.max(hi - lo, 0.001);
            const cLow = new THREE.Color(PCD_LOW_COLOR);
            const cMid = new THREE.Color(PCD_MID_COLOR);
            const cHigh = new THREE.Color(PCD_HIGH_COLOR);
            const tmp = new THREE.Color();
            const colors = new Float32Array(count * 3);
            for (let i = 0; i < count; i++) {
              const t = Math.min(1, Math.max(0, (heights[i] - lo) / span));
              if (t < 0.5) tmp.lerpColors(cLow, cMid, t * 2);
              else tmp.lerpColors(cMid, cHigh, (t - 0.5) * 2);
              colors[i * 3] = tmp.r;
              colors[i * 3 + 1] = tmp.g;
              colors[i * 3 + 2] = tmp.b;
            }
            points.geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
          }
        }

        points.material?.dispose?.();
        const pcdMaterial = new THREE.PointsMaterial({
          size: pointSize,
          vertexColors: true,
          sizeAttenuation: true,
          // 点云不透明：省掉逐像素混合与逐帧排序开销（40 万点半透明是主要瓶颈之一）
          // depthWrite 仍为 false：避免点与点互相遮挡出“黑边”，视觉上更接近原始点云
          transparent: false,
          depthWrite: false,
        });
        // 雷达扫描波：点到车辆的水平距离接近当前波半径时提亮，波后留一段渐隐余晖
        const [sr, sg, sb] = SCAN_WAVE.color.map((c) => c.toFixed(3));
        pcdMaterial.onBeforeCompile = (shader) => {
          shader.uniforms.uScanCenter = scanUniforms.uScanCenter;
          shader.uniforms.uScanRadius = scanUniforms.uScanRadius;
          shader.uniforms.uScanAlpha = scanUniforms.uScanAlpha;
          shader.vertexShader = "uniform vec2 uScanCenter;\nvarying float vScanDist;\n" + shader.vertexShader.replace(
            "#include <begin_vertex>",
            "#include <begin_vertex>\n  vScanDist = distance((modelMatrix * vec4(transformed, 1.0)).xz, uScanCenter);"
          );
          shader.fragmentShader = "uniform float uScanRadius;\nuniform float uScanAlpha;\nvarying float vScanDist;\n" + shader.fragmentShader.replace(
            "#include <color_fragment>",
            `#include <color_fragment>
  float scanEdge = 1.0 - smoothstep(0.0, ${SCAN_WAVE.width.toFixed(2)}, abs(vScanDist - uScanRadius));
  float scanTrail = step(vScanDist, uScanRadius) * (1.0 - smoothstep(0.0, 8.0, uScanRadius - vScanDist));
  diffuseColor.rgb += vec3(${sr}, ${sg}, ${sb}) * (scanEdge * ${SCAN_WAVE.intensity.toFixed(2)} + scanTrail * 0.25) * uScanAlpha;`
          );
        };
        points.material = pcdMaterial;

        // 点云铺满整个场景，逐帧视锥裁剪没有意义，关闭以免被误裁
        points.frustumCulled = false;

        pcdPointsNode = points;
        onAssetStatusRef.current?.("pcd", { points: rawPointCount });
        pointCloudGroup.add(points);

        if (internals.current) {
          internals.current.pointCloudGroup = pointCloudGroup;
          internals.current.pcdPointsNode = pcdPointsNode;
        }
      },
      undefined,
      (err) => console.error("❌ PCD 点云加载失败:", err)
    );

    const vehicle = new THREE.Group();
    const modelInnerGroup = new THREE.Group();
    vehicle.add(modelInnerGroup);
    // 车辆、轨迹、标牌和障碍物都使用 ROS map 坐标，因此必须与点云/车道
    // 共享同一个地图根节点；这样 pcdConfig 的平移、旋转和高度校准只应用一次。
    pointCloudGroup.add(vehicle);

    // ── 车底特效：柔光底盘 + 两圈向外扩散的扫描光环 ──
    const egoFx = new THREE.Group();
    pointCloudGroup.add(egoFx);
    const glowCanvas = document.createElement("canvas");
    glowCanvas.width = glowCanvas.height = 128;
    const glowCtx = glowCanvas.getContext("2d");
    const glowGrad = glowCtx.createRadialGradient(64, 64, 0, 64, 64, 64);
    glowGrad.addColorStop(0, "rgba(63,212,255,0.55)");
    glowGrad.addColorStop(0.45, "rgba(63,212,255,0.18)");
    glowGrad.addColorStop(1, "rgba(63,212,255,0)");
    glowCtx.fillStyle = glowGrad;
    glowCtx.fillRect(0, 0, 128, 128);
    const glowTexture = new THREE.CanvasTexture(glowCanvas);
    glowTexture.colorSpace = THREE.SRGBColorSpace;
    const glowMaterial = new THREE.MeshBasicMaterial({ map: glowTexture, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending });
    const glowDisc = new THREE.Mesh(new THREE.PlaneGeometry(2.8, 2.8), glowMaterial);
    glowDisc.rotation.x = -Math.PI / 2;
    glowDisc.renderOrder = 17;
    egoFx.add(glowDisc);
    const scanRingGeometry = new THREE.RingGeometry(0.97, 1, 96);
    const scanRings = [0, 1].map(() => {
      const ring = new THREE.Mesh(
        scanRingGeometry,
        new THREE.MeshBasicMaterial({ color: SCAN_COLOR, transparent: true, opacity: 0, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide })
      );
      ring.rotation.x = -Math.PI / 2;
      ring.renderOrder = 18;
      egoFx.add(ring);
      return ring;
    });

    // Persistent 3D signposts for the poses selected on the point-cloud map.
    const createSignTexture = (label, color) => {
      const canvas = document.createElement("canvas");
      canvas.width = 512;
      canvas.height = 192;
      const context = canvas.getContext("2d");
      context.clearRect(0, 0, canvas.width, canvas.height);
      context.fillStyle = "rgba(8, 14, 24, 0.52)";
      context.strokeStyle = color;
      context.lineWidth = 14;
      const left = 12;
      const top = 12;
      const right = 500;
      const bottom = 180;
      const radius = 30;
      context.beginPath();
      context.moveTo(left + radius, top);
      context.lineTo(right - radius, top);
      context.quadraticCurveTo(right, top, right, top + radius);
      context.lineTo(right, bottom - radius);
      context.quadraticCurveTo(right, bottom, right - radius, bottom);
      context.lineTo(left + radius, bottom);
      context.quadraticCurveTo(left, bottom, left, bottom - radius);
      context.lineTo(left, top + radius);
      context.quadraticCurveTo(left, top, left + radius, top);
      context.closePath();
      context.fill();
      context.stroke();
      context.fillStyle = "#ffffff";
      context.font = "700 92px system-ui, sans-serif";
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.fillText(label, 256, 100);

      const texture = new THREE.CanvasTexture(canvas);
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
      return texture;
    };

    const createRoundedBoardGeometry = () => {
      const width = 1.68;
      const height = 0.64;
      const radius = 0.1;
      const left = -width / 2;
      const right = width / 2;
      const top = height / 2;
      const bottom = -height / 2;
      const shape = new THREE.Shape();

      shape.moveTo(left + radius, bottom);
      shape.lineTo(right - radius, bottom);
      shape.quadraticCurveTo(right, bottom, right, bottom + radius);
      shape.lineTo(right, top - radius);
      shape.quadraticCurveTo(right, top, right - radius, top);
      shape.lineTo(left + radius, top);
      shape.quadraticCurveTo(left, top, left, top - radius);
      shape.lineTo(left, bottom + radius);
      shape.quadraticCurveTo(left, bottom, left + radius, bottom);

      const geometry = new THREE.ExtrudeGeometry(shape, {
        depth: 0.16,
        bevelEnabled: true,
        bevelThickness: 0.035,
        bevelSize: 0.04,
        bevelSegments: 3,
      });
      geometry.center();
      return geometry;
    };

    const createPoseSign = (label, color, colorCss) => {
      const sign = new THREE.Group();
      sign.visible = false;

      const standardMaterial = new THREE.MeshStandardMaterial({ color, metalness: 0.28, roughness: 0.42 });
      const boardMaterial = new THREE.MeshStandardMaterial({
        color,
        metalness: 0.28,
        roughness: 0.42,
        transparent: true,
        opacity: 0.62,
        depthWrite: false,
      });
      const darkMaterial = new THREE.MeshStandardMaterial({ color: 0x182231, metalness: 0.55, roughness: 0.35 });
      const glowMaterial = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.82, depthWrite: false });

      const base = new THREE.Mesh(new THREE.CylinderGeometry(0.31, 0.4, 0.18, 24), darkMaterial);
      base.position.y = 0.09;
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.095, 1.25, 18), standardMaterial);
      pole.position.y = 0.78;
      const board = new THREE.Mesh(createRoundedBoardGeometry(), boardMaterial);
      board.position.y = 1.48;
      base.castShadow = pole.castShadow = board.castShadow = true;

      const labelTexture = createSignTexture(label, colorCss);
      const labelMaterial = new THREE.MeshBasicMaterial({
        map: labelTexture,
        transparent: true,
        opacity: 0.88,
        depthWrite: false,
      });
      const labelFace = new THREE.Mesh(new THREE.PlaneGeometry(1.5, 0.5), labelMaterial);
      labelFace.position.set(0, 1.48, 0.086);
      labelFace.renderOrder = 31;
      const labelBack = new THREE.Mesh(new THREE.PlaneGeometry(1.5, 0.5), labelMaterial);
      labelBack.position.set(0, 1.48, -0.086);
      labelBack.rotation.y = Math.PI;
      labelBack.renderOrder = 31;

      const ring = new THREE.Mesh(new THREE.TorusGeometry(0.64, 0.055, 12, 48), glowMaterial);
      ring.rotation.x = Math.PI / 2;
      ring.position.y = 0.08;
      ring.renderOrder = 29;

      const arrowShape = new THREE.Shape();
      arrowShape.moveTo(0, 0.92);
      arrowShape.lineTo(0.38, 0.34);
      arrowShape.lineTo(0.15, 0.34);
      arrowShape.lineTo(0.15, -0.48);
      arrowShape.lineTo(-0.15, -0.48);
      arrowShape.lineTo(-0.15, 0.34);
      arrowShape.lineTo(-0.38, 0.34);
      arrowShape.closePath();
      const directionArrow = new THREE.Mesh(
        new THREE.ExtrudeGeometry(arrowShape, { depth: 0.07, bevelEnabled: false }),
        glowMaterial.clone()
      );
      directionArrow.geometry.center();
      directionArrow.rotation.x = Math.PI / 2;
      directionArrow.position.set(0, 0.1, 1.05);
      directionArrow.renderOrder = 30;

      sign.add(base, pole, board, labelFace, labelBack, ring, directionArrow);
      sign.userData.ring = ring;
      sign.userData.materials = [standardMaterial, boardMaterial, darkMaterial, glowMaterial, labelMaterial, directionArrow.material];
      sign.userData.texture = labelTexture;
      pointCloudGroup.add(sign);
      return sign;
    };

    const startSign = createPoseSign("起点", 0x00aaff, "#00aaff");
    const goalSign = createPoseSign("终点", 0xff4d3d, "#ff4d3d");
    const showMarkerDemo = new URLSearchParams(window.location.search).get("markerDemo") === "1";
    const demoStartPose = { x: -1.8, z: 1.5, yaw: Math.PI * 0.2 };
    const demoGoalPose = { x: 2.2, z: 5.5, yaw: -Math.PI * 0.25 };

    // 标牌与车辆/路径共用车道高度；组原点位于圆柱底座最低点，因此再加
    // ROAD_SURFACE_Y 后，底端会准确落在道路表面而不是点云估算平面上。
    const updatePoseSign = (sign, pose, time, fallbackY = 0) => {
      const valid = pose && Number.isFinite(pose.x) && Number.isFinite(pose.z);
      sign.visible = Boolean(valid);
      if (!valid) return;
      const yaw = Number.isFinite(pose.yaw) ? pose.yaw : 0;
      const forwardX = Math.sin(yaw);
      const forwardZ = Math.cos(yaw);
      const laneY = laneHeightForPoint(
        pose,
        { x: pose.x - forwardX, z: pose.z - forwardZ },
        { x: pose.x + forwardX, z: pose.z + forwardZ }
      );
      const pointCloudY = groundZAtRef.current?.(pose.z, pose.x);
      const baseGround = laneY != null
        ? laneY
        : (Number.isFinite(pointCloudY) ? pointCloudY : fallbackY);
      const y = baseGround + ROAD_SURFACE_Y;
      sign.position.set(pose.x, y, pose.z);
      sign.rotation.y = yaw;
      const pulse = 1 + Math.sin(time * 0.003) * 0.08;
      sign.userData.ring.scale.setScalar(pulse);
    };

    let frontLeftWheel = null;
    let frontRightWheel = null;

    const dracoLoader = new DRACOLoader();
    dracoLoader.setDecoderPath("https://www.gstatic.com/draco/versioned/decoders/1.5.6/");

    const gltfLoader = new GLTFLoader();
    gltfLoader.setDRACOLoader(dracoLoader);

    // 断线时车辆模型半透明（表示显示的是断线前的旧位置）
    let lastStale = null;
    const applyVehicleFade = (faded) => {
      modelInnerGroup.traverse((o) => {
        if (!o.isMesh || !o.material) return;
        (Array.isArray(o.material) ? o.material : [o.material]).forEach((m) => {
          if (m.userData.baseOpacity === undefined) {
            m.userData.baseOpacity = m.opacity;
            m.userData.baseTransparent = m.transparent;
          }
          m.transparent = faded ? true : m.userData.baseTransparent;
          m.opacity = faded ? 0.3 : m.userData.baseOpacity;
          m.needsUpdate = true;
        });
      });
    };

    const loadVehicleModel = (carConfig) => {
      while (modelInnerGroup.children.length > 0) {
        const child = modelInnerGroup.children[0];
        if (child.geometry) child.geometry.dispose();
        modelInnerGroup.remove(child);
      }
      frontLeftWheel = null;
      frontRightWheel = null;

      gltfLoader.load(
        carConfig.path,
        (gltf) => {
          const carModel = gltf.scene;

          carModel.traverse((child) => {
            if (child.isMesh) {
              child.castShadow = true;
              child.receiveShadow = true;
              if (child.material) child.material.side = THREE.DoubleSide;
            }

            const name = child.name.toLowerCase();
            if (name.includes("wheel_fl") || name.includes("front_left") || name.includes("wheel_l")) frontLeftWheel = child;
            if (name.includes("wheel_fr") || name.includes("front_right") || name.includes("wheel_r")) frontRightWheel = child;
          });

          carModel.rotation.y = carConfig.yawOffset ?? 0;
          const box = new THREE.Box3().setFromObject(carModel);
          const rawSize = box.getSize(new THREE.Vector3());
          const modelRawLength = Math.max(rawSize.x, rawSize.z);

          if (modelRawLength > 0) {
            const scaleFactor = REAL_VEHICLE_PARAM.length / modelRawLength;
            carModel.scale.set(scaleFactor, scaleFactor, scaleFactor);
          }

          const scaledBox = new THREE.Box3().setFromObject(carModel);
          const scaledCenter = scaledBox.getCenter(new THREE.Vector3());

          carModel.position.x = -scaledCenter.x;
          carModel.position.y = -scaledBox.min.y;
          carModel.position.z = -(scaledBox.min.z + REAL_VEHICLE_PARAM.rearOverhang);

          modelInnerGroup.add(carModel);
          lastStale = null; // 新模型加载后重新应用断线置灰
          onAssetStatusRef.current?.("model");
        },
        undefined,
        (err) => console.warn(`⚠️ 模型 ${carConfig.path} 加载失败:`, err)
      );
    };

    loadVehicleModel(propsRef.current.currentCar);

    // ── 高频局部引导线 (Guide Ribbon) ──
    const guideGroup = new THREE.Group();
    const guideFrameMaterial = new THREE.LineBasicMaterial({ color: GUIDE_GREEN, transparent: true, opacity: 0.95, depthTest: true, depthWrite: false });
    const guideFillMaterial = new THREE.MeshBasicMaterial({ color: GUIDE_GREEN, transparent: true, opacity: 0.25, depthTest: true, depthWrite: false, side: THREE.DoubleSide });
    const guideArrowMaterial = new THREE.MeshBasicMaterial({ color: GUIDE_GREEN, transparent: true, opacity: 0.9, depthTest: true, depthWrite: false });

    const guideArrows = [];
    const guideRibbon = new THREE.Mesh(new THREE.BufferGeometry(), guideFillMaterial);
    const guideOutline = new THREE.LineSegments(new THREE.BufferGeometry(), guideFrameMaterial);

    guideRibbon.renderOrder = 20;
    guideOutline.renderOrder = 21;
    // 几何每帧原地更新，包围球会过期；引导带始终在车前，直接关闭视锥裁剪
    guideRibbon.frustumCulled = false;
    guideOutline.frustumCulled = false;
    guideGroup.add(guideRibbon, guideOutline);

    const arrowShape = new THREE.Shape();
    arrowShape.moveTo(-0.5, 0.7);
    arrowShape.lineTo(-0.25, 0.7);
    arrowShape.lineTo(0.18, 0);
    arrowShape.lineTo(-0.25, -0.7);
    arrowShape.lineTo(-0.5, -0.7);
    arrowShape.lineTo(-0.07, 0);
    arrowShape.closePath();
    const arrowGeometry = new THREE.ExtrudeGeometry(arrowShape, { depth: 0.1, bevelEnabled: true, bevelSize: 0.04, bevelThickness: 0.04, bevelSegments: 3 });
    arrowGeometry.rotateX(-Math.PI / 2);

    for (let i = 0; i < 5; i++) {
      const arrow = new THREE.Mesh(arrowGeometry, guideArrowMaterial.clone());
      arrow.scale.setScalar(0.45);
      arrow.renderOrder = 22;
      arrow.visible = false;
      guideArrows.push(arrow);
      guideGroup.add(arrow);
    }
    guideGroup.renderOrder = 20;
    pointCloudGroup.add(guideGroup);

    // 把任意长度、任意点密度的规划线转换成固定长度、固定点数的引导线。
    // 当上游轨迹不足 limit 时，沿末段切线补齐，避免可见长度随消息跳变。
    const makeFixedLengthGuide = (points, limit, sampleCount) => {
      const clean = [];
      points.forEach((point) => {
        const previous = clean[clean.length - 1];
        if (!previous || Math.hypot(point.x - previous.x, point.z - previous.z) > 0.01) {
          clean.push({ x: point.x, z: point.z });
        }
      });
      if (clean.length < 2) return [];

      let total = 0;
      for (let i = 1; i < clean.length; i++) {
        total += Math.hypot(clean[i].x - clean[i - 1].x, clean[i].z - clean[i - 1].z);
      }

      if (total < limit) {
        const end = clean[clean.length - 1];
        const previous = clean[clean.length - 2];
        const segmentLength = Math.max(0.001, Math.hypot(end.x - previous.x, end.z - previous.z));
        const missing = limit - total;
        clean.push({
          x: end.x + ((end.x - previous.x) / segmentLength) * missing,
          z: end.z + ((end.z - previous.z) / segmentLength) * missing,
        });
      }

      const fixed = [];
      for (let i = 0; i < sampleCount; i++) {
        const point = pointOnGuide(clean, (limit * i) / (sampleCount - 1));
        if (point) fixed.push({ x: point.x, z: point.z });
      }
      return fixed;
    };

    const pointOnGuide = (points, wantedDistance) => {
      let distance = 0;
      for (let i = 1; i < points.length; i++) {
        const a = points[i - 1];
        const b = points[i];
        const length = Math.hypot(b.x - a.x, b.z - a.z);
        if (distance + length >= wantedDistance) {
          const t = (wantedDistance - distance) / Math.max(length, 0.001);
          return { x: THREE.MathUtils.lerp(a.x, b.x, t), z: THREE.MathUtils.lerp(a.z, b.z, t), yaw: Math.atan2(b.x - a.x, b.z - a.z) };
        }
        distance += length;
      }
      return null;
    };

    const roundGuideCorners = (points, radius) => {
      if (points.length < 3) return points;
      const rounded = [{ x: points[0].x, z: points[0].z }];

      for (let i = 1; i < points.length - 1; i++) {
        const previous = points[i - 1];
        const corner = points[i];
        const next = points[i + 1];
        const incoming = new THREE.Vector2(previous.x - corner.x, previous.z - corner.z);
        const outgoing = new THREE.Vector2(next.x - corner.x, next.z - corner.z);
        const incomingLength = incoming.length();
        const outgoingLength = outgoing.length();

        if (incomingLength < 0.001 || outgoingLength < 0.001) continue;
        incoming.divideScalar(incomingLength);
        outgoing.divideScalar(outgoingLength);
        const angle = Math.acos(THREE.MathUtils.clamp(incoming.dot(outgoing), -1, 1));

        if (angle < 0.05 || Math.abs(Math.PI - angle) < 0.05) {
          rounded.push({ x: corner.x, z: corner.z });
          continue;
        }

        const tangentDistance = Math.min(radius / Math.tan(angle / 2), incomingLength * 0.45, outgoingLength * 0.45);
        const start = new THREE.Vector2(corner.x, corner.z).addScaledVector(incoming, tangentDistance);
        const end = new THREE.Vector2(corner.x, corner.z).addScaledVector(outgoing, tangentDistance);
        const bisector = incoming.clone().add(outgoing);
        if (bisector.lengthSq() < 0.000001) {
          rounded.push({ x: corner.x, z: corner.z });
          continue;
        }

        const effectiveRadius = tangentDistance * Math.tan(angle / 2);
        const centerDistance = effectiveRadius / Math.sin(angle / 2);
        const center = new THREE.Vector2(corner.x, corner.z).addScaledVector(bisector.normalize(), centerDistance);
        const startAngle = Math.atan2(start.y - center.y, start.x - center.x);
        const endAngle = Math.atan2(end.y - center.y, end.x - center.x);
        let deltaAngle = endAngle - startAngle;
        if (deltaAngle > Math.PI) deltaAngle -= Math.PI * 2;
        if (deltaAngle < -Math.PI) deltaAngle += Math.PI * 2;

        rounded.push({ x: start.x, z: start.y });
        const segments = Math.max(2, Math.ceil(Math.abs(deltaAngle) * effectiveRadius / 0.15));
        for (let segment = 1; segment < segments; segment++) {
          const arcAngle = startAngle + (deltaAngle * segment) / segments;
          rounded.push({ x: center.x + Math.cos(arcAngle) * effectiveRadius, z: center.y + Math.sin(arcAngle) * effectiveRadius });
        }
        rounded.push({ x: end.x, z: end.y });
      }

      rounded.push({ x: points[points.length - 1].x, z: points[points.length - 1].z });
      return rounded;
    };

    // 逐帧复用的临时缓冲：引导带每帧重建，若每次都新建数组与 Vector3（约 200 个），
    // 60FPS 下就是上万次/秒的分配，GC 停顿会表现为间歇性掉帧。
    const ribbonBuffers = { vertices: [], indices: [], outline: [], left: [], right: [], leftBase: [], rightBase: [] };

    const updateRibbon = (points) => {
      if (points.length < 2) {
        guideRibbon.visible = guideOutline.visible = false;
        return;
      }
      const { vertices, indices, outline: outlineVertices, left, right, leftBase, rightBase } = ribbonBuffers;
      vertices.length = 0;
      indices.length = 0;
      outlineVertices.length = 0;
      // left/right 等 Vector3 按索引长期复用，不要清空；否则会在 60 FPS 下持续制造短命对象和 GC 停顿。
      points.forEach((point, i) => {
        const previous = points[Math.max(0, i - 1)];
        const next = points[Math.min(points.length - 1, i + 1)];
        const dx = next.x - previous.x;
        const dz = next.z - previous.z;
        const scale = GUIDE_HALF_WIDTH / Math.max(Math.hypot(dx, dz), 0.001);
        const nx = dz * scale;
        const nz = -dx * scale;
        const l = left[i] || (left[i] = new THREE.Vector3());
        const r = right[i] || (right[i] = new THREE.Vector3());
        const lb = leftBase[i] || (leftBase[i] = new THREE.Vector3());
        const rb = rightBase[i] || (rightBase[i] = new THREE.Vector3());
        const laneY = laneHeightForPoint(point, previous, next);
        const guideGround = laneY != null ? laneY : (displayGroundZ ?? 0);
        const topY = guideGround + GUIDE_TOP_Y;
        const baseY = guideGround + GUIDE_BASE_Y;
        l.set(point.x + nx, topY, point.z + nz);
        r.set(point.x - nx, topY, point.z - nz);
        lb.set(l.x, baseY, l.z);
        rb.set(r.x, baseY, r.z);
        vertices.push(l.x, topY, l.z, r.x, topY, r.z, l.x, baseY, l.z, r.x, baseY, r.z);
      });
      for (let i = 0; i < points.length - 1; i++) {
        const a = i * 4;
        const b = (i + 1) * 4;
        indices.push(a, b, a + 1, a + 1, b, b + 1, a, a + 2, b, a + 2, b + 2, b, a + 1, b + 1, a + 3, a + 3, b + 1, b + 3);
      }
      const ribbonGeo = guideRibbon.geometry;
      const ribbonPos = ribbonGeo.getAttribute("position");
      if (ribbonPos && ribbonPos.array.length === vertices.length) {
        ribbonPos.array.set(vertices);
        ribbonPos.needsUpdate = true;
      } else {
        ribbonGeo.dispose(); // 点数变化时才重建，并释放旧的显卡缓冲区
        ribbonGeo.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
        ribbonGeo.setIndex(indices);
      }

      const addLine = (a, b) => outlineVertices.push(a.x, a.y, a.z, b.x, b.y, b.z);
      for (let i = 1; i < points.length; i++) {
        addLine(left[i - 1], left[i]);
        addLine(right[i - 1], right[i]);
        addLine(leftBase[i - 1], leftBase[i]);
        addLine(rightBase[i - 1], rightBase[i]);
      }
      [0, points.length - 1].forEach((i) => {
        addLine(left[i], right[i]);
        addLine(leftBase[i], rightBase[i]);
        addLine(left[i], leftBase[i]);
        addLine(right[i], rightBase[i]);
      });
      const outlineGeo = guideOutline.geometry;
      const outlinePos = outlineGeo.getAttribute("position");
      if (outlinePos && outlinePos.array.length === outlineVertices.length) {
        outlinePos.array.set(outlineVertices);
        outlinePos.needsUpdate = true;
      } else {
        outlineGeo.dispose();
        outlineGeo.setAttribute("position", new THREE.Float32BufferAttribute(outlineVertices, 3));
      }
      guideRibbon.visible = guideOutline.visible = true;
    };

    // ── 静态障碍物渲染（欧式聚类 UNKNOWN 目标） ──
    // 颜色按风险：普通=灰蓝，5 m 内=琥珀，压在本车规划轨迹上=红色闪烁
    const objEntries = [];
    const objLayer = new THREE.Group();
    pointCloudGroup.add(objLayer);
    let objMsgAt = 0;
    let lastAlertKey = "";
    let lastAlertAt = 0;

    const LABEL_BG = { danger: "rgba(150,20,35,.88)", near: "rgba(70,45,0,.85)" };
    const drawObjLabel = (lb, text, style) => {
      const key = text + style;
      if (lb.key === key) return; // 文字不变就不重绘纹理
      lb.key = key;
      const c = lb.canvas.getContext("2d");
      c.clearRect(0, 0, 256, 64);
      c.fillStyle = LABEL_BG[style] || LABEL_BG.near;
      c.beginPath();
      if (c.roundRect) c.roundRect(8, 6, 240, 52, 14);
      else c.rect(8, 6, 240, 52);
      c.fill();
      c.fillStyle = "#fff";
      c.font = "700 30px system-ui, sans-serif";
      c.textAlign = "center";
      c.textBaseline = "middle";
      c.fillText(text, 128, 33);
      lb.tex.needsUpdate = true;
    };

    // ── 已放置的仿真障碍物：琥珀色虚线框 + 编号，感知结果出来之前也能看到放在哪 ──
    const placedGroup = new THREE.Group();
    pointCloudGroup.add(placedGroup);
    const placedEdgeMat = new THREE.LineDashedMaterial({ color: OBSTACLE_COLOR, dashSize: 0.22, gapSize: 0.14, transparent: true, opacity: 0.95, depthTest: false });
    const placedFillMat = new THREE.MeshBasicMaterial({ color: OBSTACLE_COLOR, transparent: true, opacity: 0.14, depthWrite: false });
    let placedBuiltFor = null;
    let placedItems = [];
    const disposePlaced = () => {
      placedItems.forEach(({ g, tex, spriteMat }) => {
        g.traverse((c) => c.geometry?.dispose());
        tex.dispose();
        spriteMat.dispose();
        placedGroup.remove(g);
      });
      placedItems = [];
    };
    const rebuildPlaced = (list) => {
      disposePlaced();
      (list || []).forEach((o) => {
        const [sx, sy, sz] = o.size.map((v) => Math.max(0.05, Number(v) || 0.5));
        const g = new THREE.Group();
        // 局部 +z 为障碍物前方（ROS x），+x 为左侧（ROS y）
        const box = new THREE.BoxGeometry(sy, sz, sx);
        const fill = new THREE.Mesh(box, placedFillMat);
        fill.renderOrder = 23;
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(box), placedEdgeMat);
        edges.computeLineDistances();
        edges.renderOrder = 24;
        const cv = document.createElement("canvas");
        cv.width = 256;
        cv.height = 64;
        const cx = cv.getContext("2d");
        cx.fillStyle = "rgba(70,45,0,.88)";
        if (cx.roundRect) { cx.beginPath(); cx.roundRect(8, 6, 240, 52, 14); cx.fill(); } else cx.fillRect(8, 6, 240, 52);
        cx.fillStyle = "#fff";
        cx.font = "700 28px system-ui, sans-serif";
        cx.textAlign = "center";
        cx.textBaseline = "middle";
        cx.fillText(`#${o.id} ${o.size.map((v) => Number(v).toFixed(1)).join("×")}`, 128, 33);
        const tex = new THREE.CanvasTexture(cv);
        tex.colorSpace = THREE.SRGBColorSpace;
        const spriteMat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
        const label = new THREE.Sprite(spriteMat);
        label.scale.set(1.4, 0.35, 1);
        label.position.y = sz / 2 + 0.45;
        label.renderOrder = 50;
        g.add(fill, edges, label);
        g.rotation.y = Number(o.yaw) || 0;
        placedGroup.add(g);
        placedItems.push({ g, o, h: sz, tex, spriteMat });
      });
    };

    const createObjEntry = () => {
      const fillMat = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide });
      const edgeMat = new THREE.LineBasicMaterial({ transparent: true, opacity: 0, depthWrite: false });
      const body = new THREE.Mesh(new THREE.BufferGeometry(), fillMat);
      const edges = new THREE.LineSegments(new THREE.BufferGeometry(), edgeMat);
      body.renderOrder = 24;
      edges.renderOrder = 25;
      const canvas = document.createElement("canvas");
      canvas.width = 256;
      canvas.height = 64;
      const tex = new THREE.CanvasTexture(canvas);
      tex.colorSpace = THREE.SRGBColorSpace;
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
      sprite.scale.set(1.4, 0.35, 1);
      sprite.renderOrder = 50;
      sprite.visible = false;
      const g = new THREE.Group();
      g.visible = false;
      g.add(body, edges, sprite);
      objLayer.add(g);
      return {
        g, body, edges, fillMat, edgeMat,
        label: { canvas, tex, sprite, key: "", at: 0 },
        id: null, cur: null, tgt: null, fp: null, H: 0, shapeH: 0, bbox: null, shapeAt: 0, hits: 0, lastSeen: 0, alpha: 0,
      };
    };

    const rebuildObjShape = (e, fp, H) => {
      e.body.geometry.dispose();
      e.edges.geometry.dispose();
      // Shape 平面取 (x, -z)，挤出后绕 X 轴转 -90°，得到 Three 局部 (x, 高度, z)
      const sh = new THREE.Shape(fp.map((v) => new THREE.Vector2(v.x, -v.z)));
      const geo = new THREE.ExtrudeGeometry(sh, { depth: H, bevelEnabled: false });
      geo.rotateX(-Math.PI / 2);
      geo.translate(0, -H / 2, 0);
      e.body.geometry = geo;
      e.edges.geometry = new THREE.EdgesGeometry(geo, 20);
      e.shapeH = H;
    };

    const fpBox = (fp, H) => {
      let a = Infinity, b = -Infinity, c = Infinity, d = -Infinity;
      for (const v of fp) {
        a = Math.min(a, v.x); b = Math.max(b, v.x);
        c = Math.min(c, v.z); d = Math.max(d, v.z);
      }
      return [a, b, c, d, H];
    };

    const disposeObjEntry = (e) => {
      objLayer.remove(e.g);
      e.body.geometry.dispose();
      e.edges.geometry.dispose();
      [e.fillMat, e.edgeMat, e.label.sprite.material].forEach((m) => m.dispose());
      e.label.tex.dispose();
    };

    // 点 (px,pz) 到障碍物轮廓的距离（在轮廓内部返回 0）
    const distToFootprint = (px, pz, cx, cz, fp) => {
      let inside = false;
      let best = Infinity;
      for (let i = 0, j = fp.length - 1; i < fp.length; j = i++) {
        const ax = cx + fp[j].x, az = cz + fp[j].z;
        const bx = cx + fp[i].x, bz = cz + fp[i].z;
        if ((bz > pz) !== (az > pz) && px < ((ax - bx) * (pz - bz)) / (az - bz) + bx) inside = !inside;
        const abx = bx - ax, abz = bz - az;
        const len2 = abx * abx + abz * abz;
        const t = len2 > 0 ? Math.max(0, Math.min(1, ((px - ax) * abx + (pz - az) * abz) / len2)) : 0;
        best = Math.min(best, Math.hypot(px - (ax + t * abx), pz - (az + t * abz)));
      }
      return inside ? 0 : best;
    };

    internals.current = {
      renderer, scene, camera, vehicle, modelInnerGroup, pointCloudGroup, guideGroup,
      loadVehicleModel, loadedCarId: propsRef.current.currentCar.id
    };

    const el = renderer.domElement;
    const onContextMenu = (e) => e.preventDefault();
    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();
    const mapPickPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const mapPickWorld = new THREE.Vector3();
    const pickedPoint = new THREE.Vector3();
    const poseDrag = { active: false, mode: null, start: new THREE.Vector3(), end: new THREE.Vector3(), startPx: { x: 0, y: 0 } };
    const poseArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 1, 0x00aaff, 0.45, 0.24);
    poseArrow.visible = false;
    poseArrow.line.material.depthTest = false;
    poseArrow.cone.material.depthTest = false;
    poseArrow.line.renderOrder = 40;
    poseArrow.cone.renderOrder = 40;
    pointCloudGroup.add(poseArrow);

    const pointOnMapPlane = (e, target) => {
      const rect = el.getBoundingClientRect();
      pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      // 地图根节点只绕 Y 轴旋转，因此其 local y=0 平面在世界坐标中仍为水平面。
      // 命中点必须转回地图局部坐标，发布给 Autoware 的才仍是原始 ROS map 坐标。
      mapPickPlane.constant = -pointCloudGroup.position.y;
      const hit = raycaster.ray.intersectPlane(mapPickPlane, mapPickWorld);
      if (!hit) return null;
      target.copy(mapPickWorld);
      pointCloudGroup.worldToLocal(target);
      return target;
    };

    const onDown = (e) => {
      if (e.button === 0 && poseModeRef.current) {
        if (pointOnMapPlane(e, pickedPoint)) {
          poseDrag.active = true;
          poseDrag.mode = poseModeRef.current;
          poseDrag.start.copy(pickedPoint);
          poseDrag.end.copy(pickedPoint);
          poseDrag.startPx = { x: e.clientX, y: e.clientY };
          poseArrow.position.copy(pickedPoint).add(new THREE.Vector3(0, 0.12, 0));
          poseArrow.setColor(new THREE.Color(poseDrag.mode === "start" ? 0x00aaff : poseDrag.mode === "obstacle" ? OBSTACLE_COLOR : 0xff4444));
          poseArrow.setLength(0.01, 0.01, 0.01);
          poseArrow.visible = true;
        }
        return;
      }
      mouse.current = { down: true, button: e.button, lx: e.clientX, ly: e.clientY };
    };
    const onUp = (e) => {
      mouse.current.down = false;
      if (!poseDrag.active) return;
      pointOnMapPlane(e, poseDrag.end);
      const dx = poseDrag.end.x - poseDrag.start.x;
      const dz = poseDrag.end.z - poseDrag.start.z;
      const dragPx = Math.hypot(e.clientX - poseDrag.startPx.x, e.clientY - poseDrag.startPx.y);
      poseDrag.active = false;
      poseArrow.visible = false;

      // Three Δ(x,z) = (ROS Δy, ROS Δx)，所以 ROS yaw = atan2(Δy, Δx)。
      let yaw = Math.atan2(dx, dz);
      let rosX = poseDrag.start.z;
      let rosY = poseDrag.start.x;
      let rosZ = 0;

      // ── 仿真障碍物：点击空地放置，点击已放置的障碍物删除；保持放置模式，可连续操作 ──
      if (poseDrag.mode === "obstacle") {
        if (dragPx < 10) {
          const hit = (simRef.current?.placedObstacles || []).find((o) => {
            const ddx = rosX - o.x;
            const ddy = rosY - o.y;
            const c = Math.cos(o.yaw || 0);
            const sn = Math.sin(o.yaw || 0);
            const lx = ddx * c + ddy * sn;
            const ly = -ddx * sn + ddy * c;
            return Math.abs(lx) <= o.size[0] / 2 + 0.3 && Math.abs(ly) <= o.size[1] / 2 + 0.3;
          });
          if (hit) {
            onRemoveObstacleRef.current?.(hit.id);
            return;
          }
          // 单击放置：朝向取附近车道方向（6 m 内），车辆等长条障碍物顺着道路摆放
          let best = null;
          for (const line of laneCenterlinesRef.current) {
            for (let i = 0; i < line.length - 1; i++) {
              const a = line[i];
              const b = line[i + 1];
              const abx = b.x - a.x;
              const aby = b.y - a.y;
              const len2 = abx * abx + aby * aby;
              const t = len2 > 0 ? Math.max(0, Math.min(1, ((rosX - a.x) * abx + (rosY - a.y) * aby) / len2)) : 0;
              const d = Math.hypot(rosX - (a.x + t * abx), rosY - (a.y + t * aby));
              if (d <= 6 && (!best || d < best.d)) best = { d, yaw: Math.atan2(aby, abx) };
            }
          }
          yaw = best ? best.yaw : 0;
        }
        onPlaceObstacleRef.current?.(rosX, rosY, yaw);
        return;
      }

      // 屏幕拖拽 < 10px 视为“点击”，没有明确方向意图：
      // 直接吸附最近车道中心线，朝向取车道方向。
      // 拖拽 >= 10px 时以拖拽方向为准，优先吸附“方向与拖拽一致”的车道，
      // 避免在双向车道上吸到对向车道，导致朝向反转 180°。
      // 吸附后朝向为车道精确方向，也顺带修掉了鼠标拖拽带来的角度误差。
      const hasIntentDirection = dragPx >= 10;
      const intendedYaw = yaw;
      const SNAP_DIST = 6;
      const SNAP_ALIGNED_DIST = 8; // 朝向一致的车道搜索半径略放宽，避免宽路/对侧车道够不到
      const ANGLE_TOL = Math.PI / 4; // 45°，与规划器 goal_angle_threshold_deg 一致
      let snapped = null;        // 最近车道（任意朝向）
      let snappedAligned = null; // 最近且朝向与拖拽意图一致的车道
      for (const line of laneCenterlinesRef.current) {
        for (let i = 0; i < line.length - 1; i++) {
          const a = line[i];
          const b = line[i + 1];
          const abx = b.x - a.x;
          const aby = b.y - a.y;
          const len2 = abx * abx + aby * aby;
          let t = len2 > 0 ? ((rosX - a.x) * abx + (rosY - a.y) * aby) / len2 : 0;
          t = Math.max(0, Math.min(1, t));
          const px = a.x + t * abx;
          const py = a.y + t * aby;
          const d = Math.hypot(rosX - px, rosY - py);
          const pz = (a.z || 0) + t * ((b.z || 0) - (a.z || 0));
          const laneYaw = Math.atan2(aby, abx);
          if (d <= SNAP_DIST && (!snapped || d < snapped.d)) snapped = { d, x: px, y: py, z: pz, yaw: laneYaw };
          if (hasIntentDirection && d <= SNAP_ALIGNED_DIST && Math.abs(normalizeAngle(laneYaw - intendedYaw)) <= ANGLE_TOL && (!snappedAligned || d < snappedAligned.d)) {
            snappedAligned = { d, x: px, y: py, z: pz, yaw: laneYaw };
          }
        }
      }

      // 优先采用朝向一致的车道；找不到时退回最近车道（保持车道方向，规划器才接受）。
      if (snappedAligned) {
        rosX = snappedAligned.x;
        rosY = snappedAligned.y;
        rosZ = snappedAligned.z;
        yaw = snappedAligned.yaw;
      } else if (snapped) {
        rosX = snapped.x;
        rosY = snapped.y;
        rosZ = snapped.z;
        yaw = snapped.yaw;
        if (hasIntentDirection && Math.abs(normalizeAngle(yaw - intendedYaw)) > ANGLE_TOL) {
          console.warn("⚠️ 拖拽方向与附近车道不一致（可能拖反了或点到了对向车道），已按最近车道方向放置");
        }
      } else {
        if (hasIntentDirection) console.warn("⚠️ 6m 内没有可吸附的车道，按原始点击位置/拖拽方向发布");
        else yaw = 0;
      }

      rosZ = 0; // 地图没有高程，发给 Autoware 的高度固定为 0

      if (publishPoseRef.current?.(poseDrag.mode, rosX, rosY, yaw, rosZ)) setPoseMode(null);
    };
    const onMove = (e) => {
      if (poseDrag.active) {
        if (!pointOnMapPlane(e, poseDrag.end)) return;
        const direction = poseDrag.end.clone().sub(poseDrag.start);
        direction.y = 0;
        const length = direction.length();
        if (length > 0.001) {
          poseArrow.setDirection(direction.normalize());
          poseArrow.setLength(length, Math.min(0.55, length * 0.3), Math.min(0.3, length * 0.18));
        }
        return;
      }
      if (!mouse.current.down || propsRef.current.viewMode !== "free") return;
      const dx = e.clientX - mouse.current.lx;
      const dy = e.clientY - mouse.current.ly;
      const fc = freeCamera.current;

      if (mouse.current.button === 0) {
        fc.theta -= dx * 0.005;
        fc.phi = Math.max(0.05, Math.min(Math.PI / 2 - 0.01, fc.phi - dy * 0.005));
      } else if (mouse.current.button === 2) {
        const panSpeed = fc.dist * 0.0015;
        const sinT = Math.sin(fc.theta);
        const cosT = Math.cos(fc.theta);
        fc.target.x -= (dx * cosT - dy * sinT) * panSpeed;
        fc.target.z -= (dx * sinT + dy * cosT) * panSpeed;
      }

      mouse.current.lx = e.clientX;
      mouse.current.ly = e.clientY;
    };

    const onWheel = (e) => {
      if (propsRef.current.viewMode === "free") {
        freeCamera.current.dist = Math.max(2, Math.min(300, freeCamera.current.dist + e.deltaY * 0.05));
      }
    };

    const onKeyDown = (e) => { keysPressed.current[e.key.toLowerCase()] = true; };
    const onKeyUp = (e) => { keysPressed.current[e.key.toLowerCase()] = false; };

    el.addEventListener("contextmenu", onContextMenu);
    el.addEventListener("mousedown", onDown);
    el.addEventListener("mouseup", onUp);
    el.addEventListener("mouseleave", onUp);
    el.addEventListener("mousemove", onMove);
    el.addEventListener("wheel", onWheel);
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);

    const onResize = () => {
      const w2 = container.clientWidth;
      const h2 = container.clientHeight;
      if (w2 && h2) {
        camera.aspect = w2 / h2;
        camera.updateProjectionMatrix();
        renderer.setSize(w2, h2);
        composer?.setSize(w2, h2);
      }
    };
    window.addEventListener("resize", onResize);

    let running = true;
    let displayGroundZ = null; // 车辆所在处的点云地面高度（平滑后）
    let lastFrameAt = 0;
    let lastArrowUpdateAt = -Infinity;
    let smoothedGuideOffsets = null;
    const tmpVec = new THREE.Vector3();
    const lookVec = new THREE.Vector3();
    const cinemaLook = new THREE.Vector3();
    const egoWorld = new THREE.Vector3();
    // 平滑后的镜头航向 / 镜头地面高度（只影响镜头，不影响车辆模型）
    let camYaw = null;
    let camGround = null;
    let displayDelay = null; // 当前实际使用的回看时长（ms）
    const poseSmoother = createPoseSmoother();

    const animate = () => {
      if (!running) return;
      requestAnimationFrame(animate);
      const frameNow = performance.now();
      if (frameNow - lastFrameAt < FRAME_GATE_MS) return;
      // 帧间隔（秒）：所有平滑都按真实时间计算，掉帧或高刷屏下跟随速度一致，不会“掉一帧就猛追一下”
      const dt = lastFrameAt ? Math.min(0.1, (frameNow - lastFrameAt) / 1000) : 1 / 60;
      lastFrameAt = frameNow;
      // 按时间常数 tau（秒）求本帧插值系数
      const ease = (tau) => 1 - Math.exp(-dt / tau);
      // 把原来“每帧 lerp 固定比例”（按 60 FPS 调的手感）换算成与帧率无关的系数
      const perFrame = (rate) => 1 - Math.pow(1 - rate, dt * 60);
      const s = simRef.current;
      const p = propsRef.current;
      if (!s) return;

      // 先更新统一地图根节点。所有 ROS map 元素均为它的子节点，避免各图层
      // 分别补偿 pcdConfig 后出现重复偏移或漏偏移。
      pointCloudGroup.visible = true;
      pointCloudGroup.position.set(p.pcdConfig.x, p.pcdConfig.y, p.pcdConfig.z);
      pointCloudGroup.rotation.y = (p.pcdConfig.rotation * Math.PI) / 180;
      pointCloudGroup.updateMatrixWorld(true);

      // 车辆位姿：从原始定位缓冲中按“略微延迟的时刻”插值，帧到达不均匀时也不会停顿
      // 回看时长向自适应目标缓慢靠拢：变化率限制在实际时间的 ±(30%/5%)，
      // 即播放速度最多短暂变为 0.7~1.05 倍，肉眼看不出“变速”。加长快（遇到卡顿要尽快稳住），缩短慢。
      const targetDelay = s.poseTimelineRef?.current?.targetDelay ?? INTERP_DELAY_MS;
      if (displayDelay == null) displayDelay = targetDelay;
      else {
        const diff = targetDelay - displayDelay;
        const maxStep = dt * 1000 * (diff > 0 ? POSE_DELAY_GROW_RATE : POSE_DELAY_SHRINK_RATE);
        displayDelay += Math.max(-maxStep, Math.min(maxStep, diff));
      }
      let ep = sampleEgoPose(s.poseBufferRef?.current, frameNow, s.ego, displayDelay);
      const latestPose = s.poseBufferRef?.current?.[s.poseBufferRef.current.length - 1];
      if (latestPose && Math.hypot(ep.x - latestPose.x, ep.z - latestPose.z) > 2) ep = latestPose;
      ep = smoothDisplayPose(poseSmoother, ep, dt); // 外推误差修正摊到 0.15 s 内，不在一帧里“抽动”

      // 全局路线：只有换了终点或重新规划时才重建条带
      if (p.routePoints !== lastRouteProp) {
        lastRouteProp = p.routePoints;
        if (!sameRoute(builtRoutePts, p.routePoints)) {
          builtRoutePts = p.routePoints && p.routePoints.length > 1 ? p.routePoints : null;
          routeStartSeg = 0;
          buildRoute(builtRoutePts);
        }
      }
      // 条带起点每帧跟随车辆：隐藏车后已驶过的部分（不重建几何，只改绘制范围）
      if (builtRoutePts && routeMesh.visible) {
        routeLocal.set(ep.x, 0, ep.z);
        let best = routeStartSeg;
        let bestD = Infinity;
        const hi = Math.min(builtRoutePts.length - 1, routeStartSeg + 80);
        for (let i = Math.max(0, routeStartSeg - 5); i <= hi; i++) {
          const d = Math.hypot(builtRoutePts[i].x - routeLocal.x, builtRoutePts[i].z - routeLocal.z);
          if (d < bestD) { bestD = d; best = i; }
        }
        routeStartSeg = best;
        routeMesh.geometry.setDrawRange(best * 6, Infinity);
        routeEdges.geometry.setDrawRange(best * 4, Infinity);
      }

      if (internals.current && internals.current.loadedCarId !== p.currentCar.id) {
        internals.current.loadedCarId = p.currentCar.id;
        internals.current.loadVehicleModel(p.currentCar);
      }

      if (internals.current?.pointCloudGroup) {
        if (internals.current?.pcdPointsNode?.material) {
          internals.current.pcdPointsNode.material.size = p.pointSize;
        }
      }

      // 车辆与路径共用车道高度：优先读取车辆前进方向上的同向车道中心高度。
      // 只有附近没有车道时才回退到点云地面/定位 z，避免点云离群点把车抬离路面。
      const locZ = Number.isFinite(ep.y) ? ep.y : 0;
      const groundHere = groundAt(ep.z, ep.x); // Three: z=ROS.x, x=ROS.y
      const forwardX = Math.sin(ep.yaw);
      const forwardZ = Math.cos(ep.yaw);
      const egoLaneGround = laneHeightForPoint(
        { x: ep.x, z: ep.z },
        { x: ep.x - forwardX, z: ep.z - forwardZ },
        { x: ep.x + forwardX, z: ep.z + forwardZ }
      );
      if (egoLaneGround != null) {
        // 路径也直接使用该车道高度，因此车辆无需再做独立的点云高度平滑。
        displayGroundZ = egoLaneGround;
      } else if (groundHere != null) {
        // 低通平滑，避免跨越地面网格时高度一跳一跳
        displayGroundZ = displayGroundZ == null || Math.abs(groundHere - displayGroundZ) > 3 ? groundHere : displayGroundZ + (groundHere - displayGroundZ) * perFrame(0.15);
      }
      const egoZ = VEHICLE_ON_GROUND && displayGroundZ != null ? displayGroundZ : locZ;
      vehicle.position.set(ep.x, egoZ + ROAD_SURFACE_Y, ep.z);
      guideGroup.position.y = 0;
      vehicle.rotation.y = ep.yaw;
      egoWorld.set(ep.x, egoZ, ep.z);
      pointCloudGroup.localToWorld(egoWorld);
      const mapGround = egoWorld.y;
      const displayYaw = ep.yaw + pointCloudGroup.rotation.y;
      const stale = !s.connected;
      if (stale !== lastStale) {
        lastStale = stale;
        applyVehicleFade(stale);
      }
      egoFx.visible = !stale;
      guideGroup.visible = !stale;

      // ── 车底特效：柔光 + 扩散光环 ──
      const fxTime = performance.now();
      egoFx.position.set(ep.x, egoZ + FX_Y, ep.z);
      scanRings.forEach((ring, i) => {
        const phase = ((fxTime / 2200) + i * 0.5) % 1;
        ring.scale.setScalar(0.7 + phase * 4.8);
        ring.material.opacity = (1 - phase) * 0.55;
      });
      glowMaterial.opacity = 0.85 + Math.sin(fxTime * 0.004) * 0.15;

      // ── 点云雷达扫描波 ──
      const scanPhase = (fxTime % SCAN_WAVE.periodMs) / SCAN_WAVE.periodMs;
      scanUniforms.uScanCenter.value.set(ep.x, ep.z);
      scanUniforms.uScanRadius.value = scanPhase * SCAN_WAVE.maxRadius;
      scanUniforms.uScanAlpha.value = Math.pow(1 - scanPhase, 0.6);

      const markerTime = performance.now();
      updatePoseSign(startSign, s.startPose || (showMarkerDemo ? demoStartPose : null), markerTime, egoZ);
      updatePoseSign(goalSign, s.goalPose || (showMarkerDemo ? demoGoalPose : null), markerTime + 700, egoZ);

      const current = { x: ep.x, z: ep.z };
      const path = s.path || [];

      let closestIndex = 0;
      let closestDistance = Infinity;
      path.forEach((point, index) => {
        const distance = Math.hypot(point.x - current.x, point.z - current.z);
        if (distance < closestDistance) {
          closestDistance = distance;
          closestIndex = index;
        }
      });

      const hasActivePlan = path.length > 1 && closestDistance < PATH_ATTACH_DISTANCE;
      // 引导带只显示前方 10 m，无需每帧处理整条高频 Trajectory。
      const planned = hasActivePlan ? path.slice(closestIndex + 1, closestIndex + 101) : [];
      const fallback = [{ x: current.x + Math.sin(ep.yaw) * GUIDE_WINDOW_LENGTH, z: current.z + Math.cos(ep.yaw) * GUIDE_WINDOW_LENGTH }];

      const rawGuidePoints = makeFixedLengthGuide(
        roundGuideCorners([current, ...(planned.length ? planned : fallback)], GUIDE_CORNER_RADIUS),
        GUIDE_WINDOW_LENGTH,
        GUIDE_SAMPLE_COUNT
      );

      // 在车辆局部偏移上做低通平滑：起点始终紧跟车辆，其余点渐进变化，
      // 不会因为最近点索引或规划消息切换而整条路径瞬移。
      const targetOffsets = rawGuidePoints.map((point) => ({ x: point.x - current.x, z: point.z - current.z }));
      if (!smoothedGuideOffsets || smoothedGuideOffsets.length !== targetOffsets.length) {
        smoothedGuideOffsets = targetOffsets;
      } else {
        smoothedGuideOffsets = targetOffsets.map((target, index) => ({
          x: THREE.MathUtils.lerp(smoothedGuideOffsets[index].x, target.x, perFrame(GUIDE_SMOOTHING)),
          z: THREE.MathUtils.lerp(smoothedGuideOffsets[index].z, target.z, perFrame(GUIDE_SMOOTHING)),
        }));
      }
      if (smoothedGuideOffsets.length) smoothedGuideOffsets[0] = { x: 0, z: 0 };

      // 平滑后再次规范化，保证最终显示长度严格为 GUIDE_WINDOW_LENGTH。
      const guidePoints = makeFixedLengthGuide(
        smoothedGuideOffsets.map((offset) => ({ x: current.x + offset.x, z: current.z + offset.z })),
        GUIDE_WINDOW_LENGTH,
        GUIDE_SAMPLE_COUNT
      );

      updateRibbon(guidePoints);

      const guideLength = guidePoints.reduce((sum, point, i) => i ? sum + Math.hypot(point.x - guidePoints[i - 1].x, point.z - guidePoints[i - 1].z) : sum, 0);
      const now = performance.now();

      if (now - lastArrowUpdateAt >= ARROW_UPDATE_INTERVAL_MS) {
        lastArrowUpdateAt = now;
        const flowTime = now * 0.001;
        const fadeDistance = Math.min(1.1, guideLength * 0.2);
        for (let i = 0; i < guideArrows.length; i++) {
          const arrow = guideArrows[i];
          if (guideLength < 1) { arrow.visible = false; continue; }
          const progress = ((flowTime * 1.35 + (i / guideArrows.length) * guideLength) % guideLength);
          const placement = pointOnGuide(guidePoints, progress);
          if (!placement) { arrow.visible = false; continue; }
          const fadeIn = THREE.MathUtils.smoothstep(progress, 0, fadeDistance);
          const fadeOut = 1 - THREE.MathUtils.smoothstep(progress, guideLength - fadeDistance, guideLength);
          const opacity = fadeIn * fadeOut;
          arrow.visible = opacity > 0.01;
          const arrowGround = laneHeightForPoint(placement);
          arrow.position.set(placement.x, (arrowGround != null ? arrowGround : egoZ) + GUIDE_TOP_Y + 0.02, placement.z);
          arrow.rotation.y = placement.yaw - Math.PI / 2;
          arrow.material.opacity = opacity * 0.88;
        }
      }

      if (modelInnerGroup) {
        modelInnerGroup.position.set(p.carOffset.x, 0, p.carOffset.z);
      }

      const delta = s.ego.steering;
      const L = REAL_VEHICLE_PARAM.wheelbase;
      const W = REAL_VEHICLE_PARAM.trackWidth;

      if (Math.abs(delta) > 0.001) {
        const tanDelta = Math.tan(delta);
        const deltaLeft = Math.atan((L * tanDelta) / (L - 0.5 * W * tanDelta));
        const deltaRight = Math.atan((L * tanDelta) / (L + 0.5 * W * tanDelta));

        if (frontLeftWheel) frontLeftWheel.rotation.y = deltaLeft;
        if (frontRightWheel) frontRightWheel.rotation.y = deltaRight;
      } else {
        if (frontLeftWheel) frontLeftWheel.rotation.y = 0;
        if (frontRightWheel) frontRightWheel.rotation.y = 0;
      }

      // ── 静态障碍物 ──
      // 障碍物高度与定位在同一坐标系里，定位 z 有偏差时障碍物也同样偏，跟车辆一起修正
      const egoGround = egoZ;

      // 已放置的仿真障碍物：列表变化时重建，位置贴点云地面
      if (s.placedObstacles !== placedBuiltFor) {
        placedBuiltFor = s.placedObstacles;
        rebuildPlaced(s.placedObstacles);
      }
      for (const it of placedItems) {
        const gz = groundAt(Number(it.o.x), Number(it.o.y));
        it.g.position.set(Number(it.o.y), (gz != null ? gz : egoGround) + it.h / 2, Number(it.o.x));
      }
      const objZFix = egoZ - locZ;

      // 收到新消息时才做关联：先按 ID 匹配，ID 变了就按位置就近匹配（欧式聚类的跟踪 ID 常会变）
      if (s.objectsAt && s.objectsAt !== objMsgAt) {
        objMsgAt = s.objectsAt;
        const used = new Set();
        const byId = new Map(objEntries.map((e) => [e.id, e]));
        for (const o of s.objects || []) {
          const dEgo = Math.hypot(o.x - ep.x, o.z - ep.z);
          if (dEgo > OBJ_MAX_DIST || dEgo < 0.6) continue; // 太远，或是本车车身被扫到
          let e = byId.get(o.id);
          if (e && used.has(e)) e = null;
          if (!e) {
            let bd = OBJ_MATCH_M;
            for (const c of objEntries) {
              if (used.has(c) || !c.tgt) continue;
              const dd = Math.hypot(c.tgt.x - o.x, c.tgt.z - o.z);
              if (dd < bd) { bd = dd; e = c; }
            }
          }
          if (!e) { e = createObjEntry(); objEntries.push(e); }
          used.add(e);
          e.id = o.id;
          e.hits += 1;
          e.lastSeen = frameNow;
          e.tgt = { x: o.x, y: o.y, z: o.z };
          if (!e.cur) e.cur = { x: o.x, z: o.z };
          e.fp = o.fp;
          e.H = o.H;
          const box = fpBox(o.fp, o.H);
          // 轮廓变化明显、且距上次重建超过 300 ms 才重建，避免聚类抖动导致闪烁
          if (!e.bbox || (frameNow - e.shapeAt > 300 && box.some((v, k) => Math.abs(v - e.bbox[k]) > OBJ_SHAPE_EPS))) {
            rebuildObjShape(e, o.fp, o.H);
            e.bbox = box;
            e.shapeAt = frameNow;
          }
        }
      }

      const objFresh = s.objectsAt && Date.now() - s.objectsAt < OBJ_STALE_MS;
      const passHalf = REAL_VEHICLE_PARAM.width / 2 + 0.25; // 本车半宽 + 余量
      // ── 挡路判断只看“车前方”的规划轨迹 ──
      // 以前用整条轨迹判断：轨迹里约有 4~5 m 在车后方，路线结束后旧轨迹也一直保留，
      // 于是车旁边、车后面的障碍物都会被报成“前方有障碍物”。
      // 现在：轨迹必须是新的（PATH_STALE_MS 内收到）且贴着车（hasActivePlan），
      // 从车的位置沿轨迹往前取 OBJ_AHEAD_MIN_M 或按车速 OBJ_AHEAD_TIME_S 秒的距离，每 0.2 m 一个检查点，记下沿轨迹的距离。
      const aheadPts = [];
      const pathFresh = Boolean(s.pathAt) && Date.now() - s.pathAt < PATH_STALE_MS;
      if (hasActivePlan && pathFresh) {
        const lookahead = Math.max(OBJ_AHEAD_MIN_M, Math.abs(Number(s.ego?.speed) || 0) * OBJ_AHEAD_TIME_S);
        let prev = { x: current.x, z: current.z };
        let along = 0;
        aheadPts.push({ x: prev.x, z: prev.z, s: 0 });
        for (let i = closestIndex + 1; i < path.length && along < lookahead; i++) {
          const seg = Math.hypot(path[i].x - prev.x, path[i].z - prev.z);
          const n = Math.max(1, Math.ceil(seg / 0.2));
          for (let k = 1; k <= n && along < lookahead; k++) {
            aheadPts.push({ x: prev.x + ((path[i].x - prev.x) * k) / n, z: prev.z + ((path[i].z - prev.z) * k) / n, s: along + (seg * k) / n });
          }
          along += seg;
          prev = path[i];
        }
      }
      const egoFront = REAL_VEHICLE_PARAM.length - REAL_VEHICLE_PARAM.rearOverhang; // 定位原点（后轴）到车头的距离
      let alertDist = null;

      for (let k = objEntries.length - 1; k >= 0; k--) {
        const e = objEntries[k];
        const age = frameNow - e.lastSeen;
        const show = objFresh && e.hits >= OBJ_SHOW_AFTER && age < OBJ_LINGER_MS;
        e.alpha += ((show ? 1 : 0) - e.alpha) * 0.15; // 淡入淡出
        if (!show && e.alpha < 0.02) {
          if (age > OBJ_LINGER_MS) {
            disposeObjEntry(e);
            objEntries.splice(k, 1);
          } else {
            e.g.visible = false;
          }
          continue;
        }

        e.cur.x += (e.tgt.x - e.cur.x) * OBJ_SMOOTH;
        e.cur.z += (e.tgt.z - e.cur.z) * OBJ_SMOOTH;
        let bottom = Number.isFinite(e.tgt.y) ? e.tgt.y - e.H / 2 + objZFix : egoGround;
        if (Math.abs(bottom - egoGround) > 2) bottom = egoGround; // 高程对不上（如仿真 z=0）时贴到本车地面
        e.g.position.set(e.cur.x, bottom + e.shapeH / 2, e.cur.z);
        e.g.visible = true;

        const dist = distToFootprint(ep.x, ep.z, e.cur.x, e.cur.z, e.fp);
        let danger = false;
        let aheadDist = null; // 沿轨迹、从车头算起的距离
        let bestGap = Infinity;
        for (const pt of aheadPts) {
          const gap = distToFootprint(pt.x, pt.z, e.cur.x, e.cur.z, e.fp);
          if (!danger) {
            if (gap < passHalf) { danger = true; bestGap = gap; aheadDist = pt.s; }
          } else if (gap < bestGap) {
            bestGap = gap; aheadDist = pt.s; // 继续往前找离障碍物最近的位置（轨迹压到障碍物边缘处）
            if (gap <= 0) break;
          } else break;
        }
        if (danger) aheadDist = Math.max(0, aheadDist - egoFront); // 换算成车头到障碍物的距离
        const near = dist < OBJ_NEAR_M;
        const color = danger ? OBJ_COLOR.danger : near ? OBJ_COLOR.near : OBJ_COLOR.far;
        e.fillMat.color.setHex(color);
        e.edgeMat.color.setHex(color);
        e.fillMat.opacity = (danger ? 0.3 + 0.12 * Math.sin(frameNow * 0.012) : 0.2) * e.alpha;
        e.edgeMat.opacity = 0.95 * e.alpha;

        const lb = e.label;
        lb.sprite.visible = danger || near;
        if (lb.sprite.visible) {
          if (frameNow - lb.at > 200) { // 距离文字最多 5 次/秒重绘
            lb.at = frameNow;
            drawObjLabel(lb, `${(danger ? aheadDist : dist).toFixed(1)} m`, danger ? "danger" : "near");
          }
          lb.sprite.position.y = e.shapeH / 2 + 0.4;
          lb.sprite.material.opacity = e.alpha;
        }
        if (danger && (alertDist == null || aheadDist < alertDist)) alertDist = aheadDist;
      }

      // 把最近的挡路障碍物距离报给界面（变化时才回调，最多约 3 次/秒）
      const alertKey = alertDist == null ? "" : alertDist.toFixed(1);
      if (alertKey !== lastAlertKey && frameNow - lastAlertAt > 300) {
        lastAlertKey = alertKey;
        lastAlertAt = frameNow;
        onObstacleAlertRef.current?.(alertDist);
      }

      // ── 镜头航向与高度单独平滑 ──
      // 车辆模型照常按定位航向实时转动；只有镜头的“机位方位”做低通。
      // 定位航向 1° 的抖动在 12 m 机位上就是约 21 cm 的横摆，整幅画面都跟着晃，这是跟随视角一跳一跳的主因。
      // 低速时（停车、刚起步）航向基本是噪声，再放慢跟随。
      const camSpeed = Math.abs(Number(s.ego?.speed) || 0);
      if (camYaw == null) camYaw = displayYaw;
      else camYaw = normalizeAngle(camYaw + normalizeAngle(displayYaw - camYaw) * ease(camSpeed > CAM_YAW_MIN_SPEED ? CAM_YAW_TAU_S : CAM_YAW_SLOW_TAU_S));
      if (camGround == null || Math.abs(mapGround - camGround) > 5) camGround = mapGround;
      else camGround += (mapGround - camGround) * ease(CAM_HEIGHT_TAU_S);

      if (p.intro) {
        // 开场：高空缓慢盘旋俯瞰车辆；开场结束后各视角的插值会形成“俯冲入场”
        const a = fxTime * 0.00012;
        tmpVec.set(egoWorld.x + Math.sin(a) * 55, mapGround + 60, egoWorld.z + Math.cos(a) * 55);
        camera.position.lerp(tmpVec, perFrame(0.05));
        camera.lookAt(egoWorld.x, mapGround, egoWorld.z);
        cinemaLook.set(egoWorld.x, mapGround, egoWorld.z);
      } else if (p.viewMode === "follow") {
        const distance = 12;
        const height = 6;
        // 车辆前向为 (sin(yaw), cos(yaw))，相机应位于该向量的正后方。
        // 机位方位用平滑后的 camYaw；车辆本身仍实时转向，所以转弯时能看到车头先转、镜头随后跟上。
        const bx = egoWorld.x - Math.sin(camYaw) * distance;
        const bz = egoWorld.z - Math.cos(camYaw) * distance;

        tmpVec.set(bx, camGround + height, bz);
        camera.position.lerp(tmpVec, ease(CAM_POS_TAU_S));
        camera.lookAt(egoWorld.x, camGround + 1, egoWorld.z);

        freeCamera.current.target.set(egoWorld.x, mapGround, egoWorld.z);
      } else if (p.viewMode === "top") {
        tmpVec.set(egoWorld.x, camGround + 45, egoWorld.z + 0.01);
        camera.position.lerp(tmpVec, perFrame(0.12));
        camera.lookAt(egoWorld.x, mapGround, egoWorld.z);

        freeCamera.current.target.set(egoWorld.x, mapGround, egoWorld.z);
      } else if (p.viewMode === "cinema") {
        // ── 巡航镜头：低机位追随 → 环绕 → 高空斜俯，循环平滑切换（演示用） ──
        const shot = Math.floor(fxTime / CINEMA_SHOT_MS) % 3;
        const fx = Math.sin(camYaw);
        const fz = Math.cos(camYaw);
        if (shot === 0) {
          tmpVec.set(egoWorld.x - fx * 7.5, camGround + 2.8, egoWorld.z - fz * 7.5);
        } else if (shot === 1) {
          const a = fxTime * 0.00022;
          tmpVec.set(egoWorld.x + Math.sin(a) * 9, camGround + 4.2, egoWorld.z + Math.cos(a) * 9);
        } else {
          tmpVec.set(egoWorld.x - fx * 16, camGround + 20, egoWorld.z - fz * 16);
        }
        camera.position.lerp(tmpVec, perFrame(0.08));
        lookVec.set(egoWorld.x + fx * 2.5, camGround + 0.8, egoWorld.z + fz * 2.5);
        cinemaLook.lerp(lookVec, perFrame(0.08));
        camera.lookAt(cinemaLook);

        freeCamera.current.target.set(egoWorld.x, mapGround, egoWorld.z);
      } else {
        const fc = freeCamera.current;
        const moveSpeed = 0.5;
        // 键盘遥控开启时，方向键/WASD 交给小车，不再移动自由视角
        const keys = teleopRef.current ? {} : keysPressed.current;
        const sinT = Math.sin(fc.theta);
        const cosT = Math.cos(fc.theta);

        if (keys["w"] || keys["arrowup"]) {
          fc.target.x -= sinT * moveSpeed;
          fc.target.z -= cosT * moveSpeed;
        }
        if (keys["s"] || keys["arrowdown"]) {
          fc.target.x += sinT * moveSpeed;
          fc.target.z += cosT * moveSpeed;
        }
        if (keys["a"] || keys["arrowleft"]) {
          fc.target.x -= cosT * moveSpeed;
          fc.target.z += sinT * moveSpeed;
        }
        if (keys["d"] || keys["arrowright"]) {
          fc.target.x += cosT * moveSpeed;
          fc.target.z -= sinT * moveSpeed;
        }
        if (keys["q"]) fc.target.y -= moveSpeed * 0.5;
        if (keys["e"]) fc.target.y += moveSpeed * 0.5;

        camera.position.set(
          fc.target.x + fc.dist * Math.sin(fc.theta) * Math.cos(fc.phi),
          fc.target.y + fc.dist * Math.sin(fc.phi),
          fc.target.z + fc.dist * Math.cos(fc.theta) * Math.cos(fc.phi)
        );
        camera.lookAt(fc.target);
      }

      if (composer) composer.render();
      else renderer.render(scene, camera);
    };
    animate();

    return () => {
      running = false;
      osmAbortController.abort();
      laneMapGroup.traverse((child) => child.geometry?.dispose());
      laneMaterial.dispose();
      roadMaterial.dispose();
      routeMaterial.dispose();
      routeEdgeMaterial.dispose();
      objEntries.forEach(disposeObjEntry);
      disposePlaced();
      pointCloudGroup.remove(placedGroup);
      placedEdgeMat.dispose();
      placedFillMat.dispose();
      objEntries.length = 0;
      pointCloudGroup.remove(poseArrow);
      poseArrow.dispose();
      [startSign, goalSign].forEach((sign) => {
        pointCloudGroup.remove(sign);
        sign.traverse((child) => child.geometry?.dispose());
        sign.userData.materials.forEach((material) => material.dispose());
        sign.userData.texture.dispose();
      });
      glowDisc.geometry.dispose();
      glowMaterial.dispose();
      glowTexture.dispose();
      scanRingGeometry.dispose();
      scanRings.forEach((ring) => ring.material.dispose());
      composer?.dispose?.();
      window.removeEventListener("resize", onResize);
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      el.removeEventListener("contextmenu", onContextMenu);
      el.removeEventListener("mousedown", onDown);
      el.removeEventListener("mouseup", onUp);
      el.removeEventListener("mouseleave", onUp);
      el.removeEventListener("mousemove", onMove);
      el.removeEventListener("wheel", onWheel);
      container.removeChild(renderer.domElement);
      renderer.dispose();
    };
  }, []);


  return (
    <div ref={containerRef} style={{ width: "100%", height: "100%", position: "absolute", top: 0, left: 0, cursor: poseMode ? "crosshair" : "default" }}>
      {poseMode && poseMode !== "obstacle" && (
        <div className={`dt-pose-hint ${poseMode === "start" ? "is-start" : "is-goal"}`}>
          {poseMode === "start" ? "设置起点" : "设置终点"}：点击吸附最近车道；按住拖动可指定朝向
        </div>
      )}
    </div>
  );
}

// ── 仿真障碍物工具条（放置模式时显示在 3D 画面上方） ──
function ObstacleToolbar({ preset, setPreset, count, online, connected, onClear, onDone }) {
  return (
    <div className="dt-obs-bar dt-glass">
      <div className="dt-obs-row">
        <span className="dt-obs-title">放置障碍物 · UNKNOWN</span>
        <div className="dt-seg">
          {OBSTACLE_PRESETS.map((pr) => (
            <button key={pr.id} className={preset.id === pr.id ? "on" : ""} onClick={() => setPreset(pr)} title={`尺寸 ${pr.size[0]} × ${pr.size[1]} × ${pr.size[2]} m（前后 × 左右 × 高），类别 UNKNOWN`}>{pr.name}</button>
          ))}
        </div>
        <span className="dt-obs-count dt-num">已放置 {count}</span>
        <button className="dt-btn bad" onClick={onClear} disabled={!count}>清除全部</button>
        <button className="dt-btn ok" onClick={onDone}>完成</button>
      </div>
      <div className={`dt-obs-tip ${online ? "" : "warn"}`}>
        {!connected ? "车辆未连接，放置不会生效"
          : online ? "点击地图放置（按住拖动可指定朝向）· 点击已放置的障碍物可删除 · 已同步到 Autoware"
          : "未收到工控机障碍物节点的回应：请在工控机上运行 click_to_obstacle 节点"}
      </div>
    </div>
  );
}

// ── 操作按键（位于“智能驾驶实况”标题栏右侧） ──
// 所有按键都显示图标 + 文字；窗口很窄时只略微缩小内边距和字号
function ActionBar({ connected = true, teleopConnected = connected, driveMode, setDriveMode, onGnssInit, gnssBusy, poseMode, setPoseMode, obstacleCount, teleopActive, onToggleTeleop, isAutonomous, autoAvailable, autoTransitioning, autoRequestStatus, requestAutonomousMode, hasGoal, onClearRoute, routeClearing, onStop, stopping }) {
  const autoLabel = autoTransitioning ? "切换中…" : autoRequestStatus === "failed" ? "重试启动" : "启动自动驾驶";
  const autoState = autoTransitioning ? "is-busy" : autoAvailable && connected ? "is-ready" : "is-off";
  const off = !connected;
  const tip = (text) => (off ? "未连接车辆" : text);
  return (
    <div className="dt-hactions">
      <div className="dt-seg dt-mode-seg" title="仿真：手动设起点；实车：向工控机请求定位服务">
        <button className={driveMode === "sim" ? "on" : ""} onClick={() => setDriveMode("sim")}>仿真</button>
        <button className={driveMode === "real" ? "on" : ""} onClick={() => setDriveMode("real")}>实车</button>
      </div>
      {driveMode === "real" ? (
        <button
          className={`dt-action dt-compact ${gnssBusy ? "is-start" : ""}`}
          title={tip("请求定位服务：由工控机（Autoware）根据卫星位置匹配点云地图完成初始化")}
          onClick={onGnssInit}
          disabled={off || isAutonomous || gnssBusy}
        >
          <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round"><circle cx="12" cy="12" r="3.2" /><path d="M12 2.5v3.2M12 18.3v3.2M2.5 12h3.2M18.3 12h3.2" /><circle cx="12" cy="12" r="7.3" /></svg>
          <span>{gnssBusy ? "初始化中…" : "请求定位服务"}</span>
        </button>
      ) : (
        <button
          className={`dt-action dt-compact ${poseMode === "start" ? "is-start" : ""}`}
          title={tip("设起点：在地图中点击或拖动，设置起点和车辆朝向")}
          onClick={() => setPoseMode(poseMode === "start" ? null : "start")}
          disabled={off || isAutonomous}
        >
          <svg viewBox="0 0 24 24" width="18" height="18"><circle cx="12" cy="12" r="4" fill="currentColor" /><circle cx="12" cy="12" r="8.5" fill="none" stroke="currentColor" strokeWidth="1.8" /></svg>
          <span>设起点</span>
        </button>
      )}
      <div className="dt-action-group">
        <button
          className={`dt-action dt-compact ${poseMode === "goal" ? "is-goal" : ""}`}
          title={tip("设终点：在地图中点击或拖动，设置目的地和到达朝向")}
          onClick={() => setPoseMode(poseMode === "goal" ? null : "goal")}
          disabled={off}
        >
          <svg viewBox="0 0 24 24" width="18" height="18"><path d="M6 21V4M6 4h11l-2.5 4L17 12H6" fill="none" stroke="currentColor" strokeWidth="1.9" strokeLinejoin="round" strokeLinecap="round" /></svg>
          <span>设终点</span>
        </button>
        {hasGoal && (
          <button
            className="dt-action dt-compact dt-action-cancel"
            title={tip(isAutonomous ? "自动驾驶中不能取消路线，请先停车" : "清除当前路线")}
            onClick={onClearRoute}
            disabled={off || isAutonomous || routeClearing}
          >
            <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round"><path d="M6 6l12 12M18 6L6 18" /></svg>
            <span>{routeClearing ? "取消中…" : "取消路线"}</span>
          </button>
        )}
      </div>
      {driveMode === "sim" && (
        <button
          className={`dt-action dt-compact ${poseMode === "obstacle" ? "is-obstacle" : ""}`}
          title={tip("障碍物：在地图上放置仿真障碍物，同步到 Autoware（需工控机运行 click_to_obstacle 节点）")}
          onClick={() => setPoseMode(poseMode === "obstacle" ? null : "obstacle")}
          disabled={off && poseMode !== "obstacle"}
        >
          <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinejoin="round"><path d="M12 3l7 16H5z" /><path d="M8.2 12.5h7.6M6.8 15.8h10.4" /></svg>
          <span>障碍物{obstacleCount ? ` ${obstacleCount}` : ""}</span>
        </button>
      )}
      <button
        className={`dt-action dt-compact ${teleopActive ? "is-teleop" : ""}`}
        title={teleopActive ? "退出底盘手动控制（Esc）" : (teleopConnected ? "云乐 CAN 直连：方向键（或 WASD）控制，空格停车，Esc 退出" : "未连接底盘 CAN 网关")}
        onClick={onToggleTeleop}
        disabled={!teleopActive && (!teleopConnected || autoTransitioning)}
      >
        <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round"><rect x="2.5" y="7" width="19" height="10.5" rx="5.2" /><path d="M7.5 10.5v3.5M5.75 12.25h3.5" /><circle cx="15.5" cy="11" r="0.9" fill="currentColor" /><circle cx="17.8" cy="13.4" r="0.9" fill="currentColor" /></svg>
        <span>{teleopActive ? "退出手动" : "底盘手动"}</span>
      </button>
      {isAutonomous ? (
        <button
          className="dt-action dt-action-auto is-stop"
          onClick={onStop}
          disabled={off || stopping}
          title="停车：退出自动驾驶，车辆平稳减速并停住"
        >
          <svg viewBox="0 0 24 24" width="18" height="18"><rect x="6.5" y="6.5" width="11" height="11" rx="1.5" fill="currentColor" /></svg>
          <span>{stopping ? "停车中…" : "停车"}</span>
        </button>
      ) : (
        <button
          className={`dt-action dt-action-auto ${autoState}`}
          onClick={requestAutonomousMode}
          disabled={off || autoTransitioning || !autoAvailable}
          title={tip(!autoAvailable ? "暂不满足启动条件（定位或车辆状态）" : "启动自动驾驶")}
        >
          <svg viewBox="0 0 24 24" width="18" height="18"><path d="M8 5.5v13l10.5-6.5z" fill="currentColor" /></svg>
          <span>{autoLabel}</span>
        </button>
      )}
    </div>
  );
}

// ── 通用面板标题栏 ──
function PanelHeader({ title, sub, live = true, right }) {
  return (
    <div className="dt-panel-header">
      <span className={`dt-dot ${live ? "on" : "off"}`} />
      <span className="dt-panel-title">{title}</span>
      {sub && <span className="dt-panel-sub">{sub}</span>}
      <div className="dt-panel-header-right">{right}</div>
    </div>
  );
}

function StatusChip({ tone, children }) {
  return (
    <span className={`dt-chip ${tone}`}>
      <i />
      {children}
    </span>
  );
}

// ── 顶部状态播报：把系统状态翻译成一句话 ──
function StatusBanner({ tone, title, sub }) {
  return (
    <div className={`dt-banner ${tone}`}>
      <span className="dt-banner-dot" />
      <div className="dt-banner-text">
        <div className="dt-banner-title">{title}</div>
        {sub && <div className="dt-banner-sub">{sub}</div>}
      </div>
    </div>
  );
}

// ── 左下角驾驶舱：速度表 + 行驶状态 + 转向 + 本次任务数据 ──
function DriveDashboard({ speed, steering, odometer, autoSeconds, distToGoal, hasGoal, stale, battery, batteryStale }) {
  const kmh = Math.max(0, (speed || 0) * 3.6);
  const MAX_KMH = 15;
  const frac = Math.min(1, kmh / MAX_KMH);
  const r = 44;
  const c = 2 * Math.PI * r;
  const arc = c * (260 / 360);
  const deg = (steering || 0) * 57.2958;
  const steerText = Math.abs(deg) < 2 ? "直行" : deg > 0 ? `左转 ${Math.abs(deg).toFixed(0)}°` : `右转 ${Math.abs(deg).toFixed(0)}°`;
  const steerPos = Math.max(-1, Math.min(1, (steering || 0) / 0.6)); // ROS 转向角：正值为左转
  const odo = formatDistance(odometer);
  const remain = formatDistance(distToGoal);
  const rawSoc = Number(battery?.bms_soc);
  const hasBattery = !batteryStale && Number.isFinite(rawSoc);
  const batterySoc = hasBattery ? Math.max(0, Math.min(100, rawSoc)) : 0;
  const batteryTone = !hasBattery ? "offline" : batterySoc <= 10 ? "critical" : batterySoc <= 20 ? "low" : "ok";
  const batteryVoltage = Number(battery?.bms_voltage);
  const batteryCurrent = Number(battery?.bms_current);
  const batteryTitle = hasBattery
    ? `底盘电池：${batterySoc.toFixed(0)}%${Number.isFinite(batteryVoltage) ? ` · ${batteryVoltage.toFixed(1)} V` : ""}${Number.isFinite(batteryCurrent) ? ` · ${batteryCurrent.toFixed(1)} A` : ""}`
    : "未收到 /yunle_chassis/feedback/bms_status，或数据已超过 5 秒未更新";

  return (
    <div className={`dt-dash dt-glass ${stale ? "stale" : ""}`} title={stale ? "车辆未连接，显示的是断线前的数据" : undefined}>
      <div className="dt-gauge">
        <svg viewBox="0 0 120 120" width="120" height="120">
          <defs>
            <linearGradient id="dtSpeedGrad" x1="0" y1="1" x2="1" y2="0">
              <stop offset="0%" stopColor="#1f7bff" />
              <stop offset="100%" stopColor="#3fd4ff" />
            </linearGradient>
          </defs>
          <circle cx="60" cy="60" r={r} fill="none" stroke="rgba(120,160,220,.14)" strokeWidth="8" strokeLinecap="round" strokeDasharray={`${arc} ${c}`} transform="rotate(140 60 60)" />
          <circle cx="60" cy="60" r={r} fill="none" stroke="url(#dtSpeedGrad)" strokeWidth="8" strokeLinecap="round" strokeDasharray={`${stale ? 0.001 : Math.max(0.001, arc * frac)} ${c}`} transform="rotate(140 60 60)" style={{ transition: "stroke-dasharray .35s ease", filter: "drop-shadow(0 0 6px rgba(63,212,255,.6))" }} />
        </svg>
        <div className="dt-gauge-center">
          <div className="dt-num dt-gauge-val">{stale ? "--" : kmh.toFixed(1)}</div>
          <div className="dt-gauge-unit">km/h</div>
        </div>
      </div>
      <div className="dt-dash-side">
        <div className="dt-dash-label">转向</div>
        <div className="dt-steer-track">
          <div className="dt-steer-center" />
          {!stale && <div className="dt-steer-knob" style={{ left: `${50 - steerPos * 45}%` }} />}
        </div>
        <div className="dt-steer-text">{stale ? "--" : steerText}</div>
        <div className={`dt-battery ${batteryTone}`} title={batteryTitle}>
          <div className="dt-battery-head">
            <span className="dt-battery-label">电量</span>
            <span className="dt-battery-value dt-num">{hasBattery ? `${batterySoc.toFixed(0)}%` : "--"}</span>
          </div>
          <div className="dt-battery-row">
            <span className="dt-battery-icon"><i style={{ width: `${batterySoc}%` }} /></span>
            <span className="dt-battery-detail">
              {hasBattery && Number.isFinite(batteryVoltage) ? `${batteryVoltage.toFixed(1)} V` : "无数据"}
            </span>
          </div>
        </div>
      </div>
      <div className="dt-dash-divider" />
      <div className="dt-dash-stats">
        <div>
          <div className="dt-stat-label">本次行驶</div>
          <div className="dt-stat-val dt-num">{odo.value}<small>{odo.unit}</small></div>
        </div>
        <div>
          <div className="dt-stat-label">自动驾驶时长</div>
          <div className="dt-stat-val dt-num">{formatDuration(autoSeconds)}</div>
        </div>
        <div>
          <div className="dt-stat-label">距目的地</div>
          <div className="dt-stat-val dt-num">{!stale && hasGoal && distToGoal != null ? <>{remain.value}<small>{remain.unit}</small></> : "--"}</div>
        </div>
      </div>
    </div>
  );
}

// ── 系统运行状态（顶栏“系统”下拉）：自动驾驶 / 障碍物识别 / 车辆定位 / 路线规划 / 方向与速度控制 ──
// 车辆连接在顶栏单独显示，这里不重复。
// 状态分四级：ok 正常（绿）/ slow 变慢（黄）/ lost 中断（红）/ idle 未启用（灰，当前无需运行，不算故障）
const HEALTH_LEVEL_TEXT = { ok: "正常", slow: "变慢", lost: "中断", idle: "未启用" };

const HEALTH_ICONS = {
  link: <path d="M4.5 9.5a11 11 0 0 1 15 0M7.3 12.6a7 7 0 0 1 9.4 0M10 15.6a3 3 0 0 1 4 0M12 19h.01" />,
  core: <><rect x="6" y="6" width="12" height="12" rx="2" /><rect x="9.5" y="9.5" width="5" height="5" rx="1" /><path d="M9 3v3M15 3v3M9 18v3M15 18v3M3 9h3M3 15h3M18 9h3M18 15h3" /></>,
  perception: <><circle cx="12" cy="12" r="8.5" /><circle cx="12" cy="12" r="4.5" /><path d="M12 12L18 6" /><circle cx="16.5" cy="14.5" r="1" fill="currentColor" /></>,
  loc: <><path d="M12 21s-6.5-6-6.5-11a6.5 6.5 0 0 1 13 0c0 5-6.5 11-6.5 11z" /><circle cx="12" cy="10" r="2.4" /></>,
  planning: <><circle cx="6" cy="18" r="2" /><circle cx="18" cy="6" r="2" /><path d="M8 18h5a3 3 0 0 0 0-6h-2a3 3 0 0 1 0-6h5" /></>,
  control: <><circle cx="12" cy="12" r="8.5" /><circle cx="12" cy="12" r="2" /><path d="M3.8 10.5h6.3M13.9 10.5h6.3M12 14v6.4" /></>,
  chassis: <><rect x="4" y="7" width="16" height="9" rx="2" /><circle cx="8" cy="18" r="2" /><circle cx="16" cy="18" r="2" /><path d="M8 10.5h8" /></>,
};

// 话题当前的到达情况 → 状态等级
function topicLevel(h, expectHz) {
  if (!h || h.ageMs == null) return { level: "never", hz: 0 };
  if (h.ageMs > HEALTH_LOST_MS) return { level: "lost", hz: 0 };
  if (h.ageMs > HEALTH_SLOW_MS || (expectHz && h.hz < expectHz / 2)) return { level: "slow", hz: h.hz };
  return { level: "ok", hz: h.hz };
}

function computeHealth({ health, connected, operationMode, autoAvailable, isAutonomous, objectCount, hasGoal, controlCmd, driveMode }) {
  const hzText = (hz) => (hz > 0 ? `${hz >= 10 ? hz.toFixed(0) : hz.toFixed(1)} Hz` : "--");
  const offline = !connected;

  const core = (() => {
    if (offline) return { level: "idle", value: "--", detail: "未连接车辆" };
    if (!operationMode) return { level: "idle", value: "--", detail: "Autoware 未启动" };
    const m = operationMode.mode;
    const modeText = isAutonomous ? "自动驾驶" : m === 1 ? "停车" : m === 3 ? "人工驾驶" : m === 4 ? "远程驾驶" : "就绪";
    return { level: "ok", value: modeText, detail: isAutonomous ? "自动驾驶运行中" : autoAvailable ? "满足启动条件" : "不满足启动条件（定位或车辆状态）" };
  })();

  const perception = (() => {
    const t = topicLevel(health?.[OBJECTS_TOPIC], HEALTH_EXPECT_HZ.perception);
    if (offline) return { level: "idle", value: "--", detail: "未连接车辆" };
    if (t.level === "never") return { level: "idle", value: "--", detail: "等待数据" };
    if (t.level === "lost") return { level: "lost", value: "--", detail: "无识别输出" };
    return { level: t.level, value: hzText(t.hz), detail: objectCount ? `检测到 ${objectCount} 个目标` : "无目标" };
  })();

  const loc = (() => {
    const t = topicLevel(health?.[LOC_TOPIC], HEALTH_EXPECT_HZ.loc);
    if (offline) return { level: "idle", value: "--", detail: "未连接车辆" };
    if (t.level === "never") return { level: "slow", value: "--", detail: "等待数据", short: "等待定位" };
    if (t.level === "lost") return { level: "lost", value: "--", detail: "无定位输出" };
    // 车辆定位健康度以实际定位数据的新鲜度和频率为准；初始化服务状态只用于过程提示，
    // 避免回放或定位已正常输出时仍因 initialization_state 显示黄色。
    if (t.level === "slow") return { level: "slow", value: hzText(t.hz), detail: "定位频率偏低", short: "定位频率偏低" };
    return { level: "ok", value: hzText(t.hz), detail: "定位正常" };
  })();

  const planning = (() => {
    const t = topicLevel(health?.[TRAJ_TOPIC], HEALTH_EXPECT_HZ.planning);
    const bp = topicLevel(health?.[BEHAVIOR_PATH_TOPIC], 0);
    if (offline) return { level: "idle", value: "--", detail: "未连接车辆" };
    if (t.level === "ok" || t.level === "slow") return { level: t.level, value: hzText(t.hz), detail: "规划运行中" };
    if (!hasGoal) return { level: "idle", value: "--", detail: "未设置目的地" };
    if (bp.level === "ok" || bp.level === "slow") return { level: "slow", value: "--", detail: "等待轨迹输出" };
    return { level: t.level === "lost" ? "lost" : "slow", value: "--", detail: t.level === "lost" ? "无规划输出" : "路线规划中" };
  })();

  const control = (() => {
    const t = topicLevel(health?.[CONTROL_TOPIC], HEALTH_EXPECT_HZ.control);
    if (offline) return { level: "idle", value: "--", detail: "未连接车辆" };
    if (t.level === "ok" || t.level === "slow") {
      const deg = controlCmd ? (controlCmd.steer * 180) / Math.PI : 0;
      const steerText = Math.abs(deg) < 1 ? "直行" : `${deg > 0 ? "左转" : "右转"} ${Math.abs(deg).toFixed(0)}°`;
      const velText = controlCmd ? `${(controlCmd.vel * 3.6).toFixed(1)} km/h` : "--";
      return { level: t.level, value: hzText(t.hz), detail: `${velText} · ${steerText}` };
    }
    if (!isAutonomous) return { level: "idle", value: "--", detail: "仅自动驾驶时输出" };
    return { level: "lost", value: "--", detail: "无控制输出" };
  })();

  const chassis = (() => {
    const t = topicLevel(health?.[YUNLE_FEEDBACK_TOPIC], HEALTH_EXPECT_HZ.chassis);
    if (offline) return { level: "idle", value: "--", detail: "未连接车辆" };
    if (t.level === "never") return { level: "idle", value: "--", detail: driveMode === "real" ? "未收到底盘反馈" : "仿真模式无需底盘" };
    if (t.level === "lost") {
      const gap = Math.round((health?.[YUNLE_FEEDBACK_TOPIC]?.ageMs || 0) / 1000);
      return driveMode === "real" ? { level: "lost", value: "--", detail: `已中断 ${gap} 秒`, short: "底盘反馈中断" } : { level: "idle", value: "--", detail: `已 ${gap} 秒未收到` };
    }
    return { level: t.level, value: hzText(t.hz), detail: "chassis_driver 在线" };
  })();

  const items = [
    { key: "core", name: "自动驾驶", ...core },
    { key: "perception", name: "障碍物识别", ...perception },
    { key: "loc", name: "车辆定位", ...loc },
    { key: "planning", name: "路线规划", ...planning },
    { key: "control", name: "方向与速度控制", ...control },
    { key: "chassis", name: "底盘反馈", ...chassis },
  ];
  const lost = items.filter((i) => i.level === "lost");
  const slow = items.filter((i) => i.level === "slow");
  const label = (i) => i.short || `${i.name}${HEALTH_LEVEL_TEXT[i.level]}`;
  let headline;
  let tone;
  if (!connected) { headline = "未连接"; tone = "idle"; }
  else if (lost.length) { headline = `${label(lost[0])}${lost.length > 1 ? ` 等 ${lost.length} 项` : ""}`; tone = "lost"; }
  else if (slow.length) { headline = `${label(slow[0])}${slow.length > 1 ? ` 等 ${slow.length} 项` : ""}`; tone = "slow"; }
  else { headline = "正常"; tone = "ok"; }
  return { items, headline, tone };
}

// ── 顶栏信息下拉：点开显示详情；点外面或按 Esc 关闭 ──
function InfoDropdown({ trigger, triggerTitle, title, children, align = "left", width = 340 }) {
  const [open, setOpen] = useState(false);
  const ref = useRef(null);
  useEffect(() => {
    if (!open) return;
    const onDown = (e) => { if (ref.current && !ref.current.contains(e.target)) setOpen(false); };
    const onKey = (e) => { if (e.key === "Escape") setOpen(false); };
    document.addEventListener("mousedown", onDown);
    window.addEventListener("keydown", onKey);
    return () => {
      document.removeEventListener("mousedown", onDown);
      window.removeEventListener("keydown", onKey);
    };
  }, [open]);
  return (
    <div className="dt-dd" ref={ref}>
      <button type="button" className={`dt-dd-trigger ${open ? "open" : ""}`} title={triggerTitle} onClick={() => setOpen((v) => !v)}>
        {trigger}
        <span className="dt-dd-caret">▾</span>
      </button>
      {open && (
        <div className={`dt-dd-menu dt-glass ${align}`} style={{ width }}>
          <div className="dt-dd-head"><span>{title}</span></div>
          {children}
        </div>
      )}
    </div>
  );
}

const LEVEL_TO_CHIP = { ok: "ok", slow: "warn", lost: "bad", idle: "" };

// 车辆定位的细节：位置、朝向；实车模式加卫星（初始化状态已显示在该行的状态标签上）
function locationDetails({ ego, driveMode, gnss }) {
  const yawDeg = Number.isFinite(ego?.yaw) ? (((ego.yaw * 180) / Math.PI) % 360 + 360) % 360 : null;
  const rows = [
    ["当前位置", Number.isFinite(ego?.lat) ? `${ego.lat.toFixed(6)}, ${ego.lng.toFixed(6)}` : "--"],
    ["车头朝向", yawDeg != null ? `${yawDeg.toFixed(0)}°（正东为 0°，逆时针）` : "--"],
  ];
  if (driveMode === "real") {
    const fresh = gnss && Date.now() - gnss.at <= GNSS_STALE_MS;
    let gnssText = "无数据";
    if (fresh) gnssText = gnss.format === "gins" ? `有信号 · 状态码 ${gnss.status}` : gnss.status < 0 ? "无解" : `${GNSS_STATUS_TEXT[gnss.status] || "已定位"}`;
    rows.push(["卫星", gnssText]);
    if (fresh) rows.push(["卫星经纬度", `${gnss.lat.toFixed(6)}, ${gnss.lon.toFixed(6)}`]);
  }
  return rows;
}

// ── 顶栏“诊断”：UDP 实时质量 + 原系统运行状态；仅在点开时占用下拉区域 ──
function SystemMenu(props) {
  const { items, tone } = computeHealth(props);
  const locRows = locationDetails(props);
  const udp = props.udpDiagnostics;
  const fmtMs = (value) => (Number.isFinite(value) && value > 0 ? `${Math.round(value)} ms` : "--");
  const fmtAge = (value) => value == null ? "未收到" : value < 1000 ? `${Math.round(value)} ms` : `${(value / 1000).toFixed(1)} s`;
  const udpRows = Object.entries(UDP_DIAG_SPECS).map(([type, spec]) => {
    const row = udp?.types?.[type] || {};
    const expectedMs = 1000 / spec.expectedHz;
    let level = "idle";
    if (row.framesSeen > 0) {
      const stale = row.ageMs != null && row.ageMs > Math.max(3000, expectedMs * 4);
      if (stale || row.lossPct > 2 || (row.p95GapMs > 0 && row.p95GapMs > expectedMs * 3)) level = "lost";
      else if (row.lossPct > 0.5 || (row.p95GapMs > 0 && row.p95GapMs > expectedMs * 1.8)) level = "slow";
      else level = "ok";
    }
    const expectedFrames = (row.framesSeen || 0) + (row.missingFrames || 0);
    const lostFrames = (row.missingFrames || 0) + (row.incompleteFrames || 0);
    return { type, spec, row, level, expectedFrames, lostFrames };
  });
  const udpWorst = udpRows.some((item) => item.level === "lost") ? "lost"
    : udpRows.some((item) => item.level === "slow") ? "slow"
      : udpRows.some((item) => item.level === "ok") ? "ok" : "idle";
  const severity = { idle: 0, ok: 1, slow: 2, lost: 3 };
  const combinedTone = severity[udpWorst] > severity[tone] ? udpWorst : tone;
  return (
    <InfoDropdown
      triggerTitle="点击查看 UDP 丢帧、抖动、流量及系统运行状态"
      title="实时诊断"
      width={520}
      trigger={<StatusChip tone={LEVEL_TO_CHIP[combinedTone]}>诊断</StatusChip>}
    >
      <div className="dt-diag-toolbar">
        <span>UDP {ROS_UDP_TELEMETRY.listenPort} · 来源 {ROS_UDP_TELEMETRY.allowedSource || "不限"}</span>
        <button type="button" onClick={props.onResetUdpDiagnostics}>重置统计</button>
      </div>
      <div className="dt-diag-summary">
        <span><b className="dt-num">{udp?.lossPct?.toFixed(2) || "0.00"}%</b><small>综合丢帧</small></span>
        <span><b className="dt-num">{udp?.totalKbps?.toFixed(1) || "0.0"}</b><small>接收 Kbit/s</small></span>
        <span><b className="dt-num">{udp?.totalLost || 0}</b><small>丢失/不完整</small></span>
        <span><b className="dt-num">{Math.round((udp?.elapsedMs || 0) / 1000)}</b><small>统计秒数</small></span>
      </div>
      <div className="dt-diag-list">
        {udpRows.map(({ type, spec, row, level, expectedFrames, lostFrames }) => (
          <div key={type} className={`dt-diag-row ${level}`}>
            <i />
            <span className="dt-diag-name"><b>{spec.label}</b><small>{row.source || "等待 UDP 数据"}</small></span>
            <span className="dt-diag-rate"><b className="dt-num">{row.hz > 0 ? row.hz.toFixed(1) : "--"} Hz</b><small className="dt-num">{row.kbps > 0 ? row.kbps.toFixed(1) : "0.0"} Kbit/s</small></span>
            <span className="dt-diag-detail">
              <small>丢帧</small><b className="dt-num">{lostFrames}/{expectedFrames || 0} · {(row.lossPct || 0).toFixed(2)}%</b>
              <small>P95 间隔</small><b className="dt-num">{fmtMs(row.p95GapMs)}</b>
              <small>分包失败</small><b className="dt-num">{row.incompleteFrames || 0}</b>
              <small>最近收帧</small><b className="dt-num">{fmtAge(row.ageMs)}</b>
            </span>
          </div>
        ))}
      </div>
      <div className="dt-diag-note">丢帧按 UDP seq 跳号统计；P95 是可用帧到达间隔。rosbag 时间戳不是当前时间，因此不显示误导性的绝对单向延迟。</div>
      <div className="dt-diag-section">系统运行状态</div>
      {items.map((it) => (
        <div key={it.key} className={`dt-sys-item ${it.level}`}>
          <div className="dt-sys-row">
            <svg className="dt-sys-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">{HEALTH_ICONS[it.key]}</svg>
            <span className="dt-sys-main">
              <b>{it.name}</b>
              <small>{it.detail}</small>
            </span>
            <span className="dt-sys-side">
              <em>{it.pill || HEALTH_LEVEL_TEXT[it.level]}</em>
              <small className="dt-num">{it.value}</small>
            </span>
          </div>
          {it.key === "loc" && props.connected && (
            <div className="dt-kv dt-sys-sub">
              {locRows.map(([k, v]) => (
                <div key={k}><span>{k}</span><b className="dt-num">{v}</b></div>
              ))}
            </div>
          )}
        </div>
      ))}
      <div className="dt-sys-foot">“未启用”表示当前无需运行，不属于故障</div>
    </InfoDropdown>
  );
}

// ── 开场检查：连接车辆 / 车辆定位 / 相机画面 / 地图数据，每项带具体信息；失败时显示原因和可点的操作 ──
// 每项 state：ok 完成 / warn 完成但需注意 / wait 进行中 / fail 失败
function BootOverlay({ items, leaving, timedOut, autoEnterIn, onEnter }) {
  const finished = items.filter((i) => i.state !== "wait").length;
  const pct = Math.round((finished / items.length) * 100);
  const failed = items.some((i) => i.state === "fail");
  return (
    <div className={`dt-boot ${leaving ? "leaving" : ""}`}>
      <div className="dt-boot-grid" />
      <div className="dt-boot-card">
        <div className="dt-boot-logo"><BrandLogo size={64} /></div>
        <div className="dt-boot-name">南京核星力</div>
        <div className="dt-boot-sub">智能无人车 · 数字孪生平台</div>
        <div className={`dt-boot-bar ${failed ? "warn" : ""}`}><i style={{ width: `${Math.max(6, pct)}%` }} /></div>
        <div className="dt-boot-list">
          {items.map((it) => (
            <div key={it.label} className={`dt-boot-row ${it.state}`}>
              <span className="dt-boot-check">{{ ok: "✓", warn: "!", fail: "✕", wait: "" }[it.state]}</span>
              <span className="dt-boot-text">
                <b>{it.label}</b>
                <small>{it.detail}</small>
              </span>
              {it.action && <button type="button" className="dt-btn" onClick={it.action.onClick}>{it.action.label}</button>}
            </div>
          ))}
        </div>
        <div className="dt-boot-foot">
          <span>{timedOut ? (failed ? `有项目未就绪，${autoEnterIn} 秒后自动进入` : `${autoEnterIn} 秒后自动进入`) : "正在检查…"}</span>
          <button type="button" className="dt-btn ok" onClick={onEnter}>直接进入</button>
        </div>
      </div>
    </div>
  );
}

function BrandLogo({ size = 30 }) {
  return (
    <svg viewBox="0 0 32 32" width={size} height={size} aria-hidden="true">
      <defs>
        <linearGradient id="dtLogoGrad" x1="0" y1="0" x2="1" y2="1">
          <stop offset="0%" stopColor="#3fd4ff" />
          <stop offset="100%" stopColor="#34f5a4" />
        </linearGradient>
      </defs>
      <circle cx="16" cy="16" r="14" fill="none" stroke="url(#dtLogoGrad)" strokeWidth="1.6" opacity="0.55" />
      <ellipse cx="16" cy="16" rx="14" ry="5.5" fill="none" stroke="url(#dtLogoGrad)" strokeWidth="1.4" transform="rotate(-30 16 16)" />
      <circle cx="16" cy="16" r="4.2" fill="url(#dtLogoGrad)" />
    </svg>
  );
}

// ── 全局样式（配色、动画、面板、地图标记） ──
const GLOBAL_CSS = `
.dt-root{--bg:#060a14;--panel:rgba(9,15,28,.93);--line:rgba(96,165,250,.14);--line-strong:rgba(63,212,255,.35);--text:#e6f1ff;--sub:#93a7c3;--dim:#5b6b84;--accent:#3fd4ff;--green:#34f5a4;--warn:#ffb547;--danger:#ff5b6b;
  font-family:"PingFang SC","Microsoft YaHei","Noto Sans SC",system-ui,sans-serif;color:var(--text);background:radial-gradient(1200px 600px at 20% -10%,#0d1a33 0%,var(--bg) 60%);}
.dt-root *{box-sizing:border-box;}
.dt-num{font-variant-numeric:tabular-nums;font-family:"DIN Alternate","Bahnschrift","Roboto Mono","SF Mono",ui-monospace,monospace;}
/* 面板不做 backdrop-filter：它会强制每帧对面板下方的 WebGL 画面重新采样模糊，
   在整屏 3D 场景上是明确的掉帧来源；改用更实的底色来保证文字可读。 */
.dt-glass{background:var(--panel);border:1px solid var(--line);border-radius:12px;box-shadow:0 8px 32px rgba(0,0,0,.35),inset 0 1px 0 rgba(255,255,255,.04);}

/* 顶栏 */
.dt-header{position:absolute;top:0;left:0;right:0;height:52px;z-index:40;display:flex;align-items:center;gap:18px;padding:0 16px;
  background:linear-gradient(180deg,rgba(8,14,28,.96),rgba(8,14,28,.86));border-bottom:1px solid var(--line);}
.dt-header::after{content:"";position:absolute;left:0;right:0;bottom:-1px;height:1px;background:linear-gradient(90deg,transparent,rgba(63,212,255,.55),transparent);}
.dt-brand{display:flex;align-items:center;gap:10px;min-width:0;}
.dt-brand-name{font-size:16px;font-weight:800;letter-spacing:2px;background:linear-gradient(90deg,#e6f1ff,#9fe9ff);-webkit-background-clip:text;background-clip:text;color:transparent;white-space:nowrap;}
.dt-brand-sub{font-size:11px;color:var(--sub);letter-spacing:1px;white-space:nowrap;}
.dt-chips{position:relative;display:flex;gap:8px;flex-wrap:nowrap;}
.dt-ip-label{font-size:12px;color:var(--sub);white-space:nowrap;}
.dt-chip{display:inline-flex;align-items:center;gap:6px;padding:4px 10px;border-radius:999px;font-size:12px;white-space:nowrap;border:1px solid var(--line);background:rgba(255,255,255,.03);color:var(--sub);}
.dt-chip i{width:7px;height:7px;border-radius:50%;background:var(--dim);}
.dt-chip.ok{color:#bff8df;border-color:rgba(52,245,164,.3);background:rgba(52,245,164,.08);}
.dt-chip.ok i{background:var(--green);box-shadow:0 0 8px var(--green);}
.dt-chip.active{color:#c8f1ff;border-color:rgba(63,212,255,.35);background:rgba(63,212,255,.1);}
.dt-chip.active i{background:var(--accent);box-shadow:0 0 8px var(--accent);animation:dt-breathe 1.6s ease-in-out infinite;}
.dt-chip.warn{color:#ffe0ad;border-color:rgba(255,181,71,.35);background:rgba(255,181,71,.08);}
.dt-chip.warn i{background:var(--warn);}
.dt-chip.bad{color:#ffc2c8;border-color:rgba(255,91,107,.35);background:rgba(255,91,107,.08);}
.dt-chip.bad i{background:var(--danger);animation:dt-breathe 1s ease-in-out infinite;}
.dt-header-right{margin-left:auto;display:flex;align-items:center;gap:8px;}
.dt-report-btn{display:inline-flex;align-items:center;gap:6px;height:32px;padding:0 12px;border-radius:8px;border:1px solid var(--line);background:rgba(255,255,255,.04);color:var(--sub);font-size:13px;cursor:pointer;transition:all .15s;}
.dt-report-btn:hover{color:#fff;border-color:var(--line-strong);background:rgba(63,212,255,.1);}
.dt-input{background:rgba(255,255,255,.04);border:1px solid var(--line);color:var(--accent);padding:5px 10px;border-radius:8px;font-size:12px;outline:none;width:170px;}
.dt-input:focus{border-color:var(--line-strong);}
.dt-btn{padding:5px 12px;border-radius:8px;border:1px solid var(--line);background:rgba(255,255,255,.04);color:var(--sub);font-size:12px;cursor:pointer;white-space:nowrap;transition:all .15s;}
.dt-btn:hover:not(:disabled){border-color:var(--line-strong);color:var(--text);}
.dt-btn:disabled{opacity:.5;cursor:not-allowed;}
.dt-btn.ok{color:#9ff5cf;border-color:rgba(52,245,164,.35);}
.dt-btn.busy{color:#ffd28a;border-color:rgba(255,181,71,.4);}
.dt-btn.bad{color:#ff9aa4;border-color:rgba(255,91,107,.4);}
.dt-seg{display:flex;padding:3px;border-radius:9px;background:rgba(255,255,255,.04);border:1px solid var(--line);}
.dt-seg button{border:0;background:transparent;color:var(--sub);font-size:12px;padding:4px 12px;border-radius:6px;cursor:pointer;transition:all .15s;white-space:nowrap;}
.dt-seg button.on{background:linear-gradient(135deg,rgba(63,212,255,.25),rgba(52,245,164,.18));color:#fff;box-shadow:inset 0 0 0 1px rgba(63,212,255,.35);}
.dt-clock{text-align:right;line-height:1.15;padding-left:6px;}
.dt-clock b{display:block;font-size:17px;color:var(--text);font-weight:600;}
.dt-clock span{font-size:10px;color:var(--dim);}

/* 面板 */
.dt-panel-header{height:36px;display:flex;align-items:center;gap:8px;padding:0 14px;background:linear-gradient(90deg,rgba(63,212,255,.08),rgba(8,14,28,.9) 40%);border-bottom:1px solid var(--line);flex-shrink:0;}
.dt-panel-title{font-size:13px;font-weight:700;color:#d6ecff;letter-spacing:1px;white-space:nowrap;flex-shrink:0;}
.dt-panel-header-right{margin-left:auto;display:flex;align-items:center;gap:8px;flex-shrink:0;}
.dt-meta{font-size:11px;color:var(--dim);font-family:ui-monospace,monospace;}
.dt-dot{width:7px;height:7px;border-radius:50%;flex-shrink:0;}
.dt-dot.on{background:var(--green);box-shadow:0 0 8px var(--green);}
.dt-dot.off{background:#4a5a70;}
.dt-card-title{font-size:12px;color:var(--sub);letter-spacing:1px;margin-bottom:10px;display:flex;align-items:center;gap:6px;}
.dt-card-title::before{content:"";width:3px;height:12px;border-radius:2px;background:linear-gradient(var(--accent),var(--green));}

/* 3D 视图叠加层 */
.dt-views{position:absolute;right:14px;bottom:14px;z-index:10;display:flex;flex-direction:row;gap:4px;padding:4px;}
.dt-views button{border:1px solid transparent;background:transparent;color:var(--sub);font-size:12px;font-weight:600;padding:5px 12px;border-radius:8px;cursor:pointer;transition:all .15s;}
.dt-views button:hover{color:var(--text);}
.dt-views button.on{color:#fff;background:rgba(63,212,255,.14);border-color:rgba(63,212,255,.35);}
.dt-banner{position:absolute;top:14px;left:50%;transform:translateX(-50%);z-index:12;display:flex;align-items:center;gap:12px;padding:9px 20px 9px 16px;border-radius:14px;
  background:rgba(9,15,28,.93);border:1px solid var(--line);box-shadow:0 8px 28px rgba(0,0,0,.35);pointer-events:none;max-width:calc(100% - 220px);min-width:220px;}
.dt-banner-dot{width:10px;height:10px;border-radius:50%;background:var(--dim);flex-shrink:0;position:relative;}
.dt-banner-dot::after{content:"";position:absolute;inset:0;border-radius:50%;background:inherit;animation:dt-pulse 1.8s ease-out infinite;}
.dt-banner-title{font-size:15px;font-weight:700;letter-spacing:1px;white-space:nowrap;}
.dt-banner-sub{font-size:11px;color:var(--sub);margin-top:2px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}
.dt-banner.ok{border-color:rgba(52,245,164,.35);} .dt-banner.ok .dt-banner-dot{background:var(--green);} .dt-banner.ok .dt-banner-title{color:#bff8df;}
.dt-banner.active{border-color:rgba(63,212,255,.35);} .dt-banner.active .dt-banner-dot{background:var(--accent);} .dt-banner.active .dt-banner-title{color:#c8f1ff;}
.dt-banner.warn{border-color:rgba(255,181,71,.35);} .dt-banner.warn .dt-banner-dot{background:var(--warn);} .dt-banner.warn .dt-banner-title{color:#ffe0ad;}
.dt-banner.danger{border-color:rgba(255,91,107,.55);} .dt-banner.danger .dt-banner-dot{background:var(--danger);} .dt-banner.danger .dt-banner-title{color:#ffc2c8;}
.dt-banner.idle .dt-banner-dot::after{animation:none;}
.dt-lanedev{position:absolute;top:72px;left:50%;transform:translateX(-50%);z-index:8;pointer-events:none;padding:3px 12px;border-radius:999px;background:rgba(9,15,28,.72);border:1px solid var(--line);color:var(--sub);font-size:11px;white-space:nowrap;}
.dt-lanedev span{color:#9fe9ff;}
.dt-pose-hint{position:absolute;top:76px;left:50%;transform:translateX(-50%);z-index:30;padding:7px 14px;border-radius:10px;white-space:nowrap;pointer-events:none;background:rgba(9,15,28,.88);font-size:12px;}
.dt-pose-hint.is-start{border:1px solid rgba(63,212,255,.45);color:#9fe9ff;}
.dt-pose-hint.is-goal{border:1px solid rgba(255,91,107,.45);color:#ffb0b8;}
.dt-action.is-obstacle{color:#fff;background:rgba(255,160,40,.28);border-color:rgba(255,181,71,.7);box-shadow:0 0 16px rgba(255,181,71,.3);}
.dt-obs-bar{position:absolute;top:76px;left:50%;transform:translateX(-50%);z-index:30;padding:8px 12px;display:flex;flex-direction:column;gap:6px;border-color:rgba(255,181,71,.45);max-width:calc(100% - 28px);}
.dt-obs-row{display:flex;align-items:center;gap:8px;white-space:nowrap;}
.dt-obs-title{font-size:13px;font-weight:700;color:#ffe0ad;letter-spacing:1px;}
.dt-obs-bar .dt-seg button{padding:3px 10px;}
.dt-obs-count{font-size:12px;color:var(--sub);}
.dt-obs-tip{font-size:11px;color:var(--sub);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}
.dt-obs-tip.warn{color:#ffd28a;}
.dt-actions{position:absolute;right:14px;top:50%;transform:translateY(-50%);z-index:30;display:flex;flex-direction:column;gap:6px;padding:6px;border-radius:14px;background:rgba(9,15,28,.92);border:1px solid var(--line);box-shadow:0 8px 28px rgba(0,0,0,.35);}
.dt-action{display:flex;align-items:center;gap:8px;width:136px;height:42px;padding:0 14px;border-radius:10px;border:1px solid rgba(255,255,255,.06);background:rgba(255,255,255,.03);color:var(--sub);font-size:13px;font-weight:600;cursor:pointer;transition:all .15s;white-space:nowrap;}
.dt-action:hover:not(:disabled){color:var(--text);border-color:var(--line-strong);}
.dt-action:disabled{opacity:.45;cursor:not-allowed;}
/* 标题栏内的紧凑按键 */
.dt-hactions{display:flex;align-items:center;gap:6px;}
.dt-hactions .dt-action{width:auto;height:28px;padding:0 12px;gap:6px;font-size:12px;border-radius:8px;}
.dt-hactions .dt-action svg{width:15px;height:15px;}
.dt-hactions .dt-live-dot{width:8px;height:8px;}
.dt-mode-seg{padding:2px;margin-right:4px;}
.dt-mode-seg button{padding:3px 10px;font-size:12px;}
.dt-panel-header{min-height:44px;height:auto;}
.dt-action.is-start{color:#fff;background:rgba(0,150,230,.35);border-color:rgba(63,212,255,.6);}
.dt-action.is-goal{color:#fff;background:rgba(220,50,70,.35);border-color:rgba(255,91,107,.6);}
.dt-action-auto.is-ready{color:#062014;background:linear-gradient(135deg,#34f5a4,#3fd4ff);border-color:transparent;box-shadow:0 0 18px rgba(52,245,164,.35);}
.dt-action-auto.is-ready:hover:not(:disabled){filter:brightness(1.08);color:#062014;}
.dt-action-auto.is-on{color:#bff8df;background:rgba(52,245,164,.14);border-color:rgba(52,245,164,.5);cursor:default;}
.dt-action-auto.is-busy{color:#ffe0ad;background:rgba(255,181,71,.12);border-color:rgba(255,181,71,.45);cursor:progress;}
.dt-action-auto.is-off{opacity:.55;cursor:not-allowed;}
.dt-live-dot{width:9px;height:9px;border-radius:50%;background:var(--green);box-shadow:0 0 10px var(--green);animation:dt-breathe 1.2s ease-in-out infinite;}

/* 右上信息卡 */
.dt-side{position:absolute;top:14px;right:14px;width:min(250px,calc(100% - 28px));max-height:calc(100% - 100px);display:flex;flex-direction:column;gap:8px;z-index:10;overflow-y:auto;scrollbar-width:thin;scrollbar-color:#2a3d55 transparent;}
.dt-mission{padding:14px;}
.dt-mission-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px 10px;}
.dt-stat-label{font-size:11px;color:var(--dim);margin-bottom:4px;}
.dt-stat-val{font-size:22px;font-weight:700;color:var(--text);line-height:1;}
.dt-stat-val small{font-size:11px;color:var(--sub);margin-left:3px;font-weight:500;}
.dt-stat-val.ok{color:var(--green);} .dt-stat-val.warn{color:var(--warn);}
.dt-pose{padding:12px;}
.dt-pose-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px 6px;}
.dt-pose-grid .k{font-size:10px;color:var(--dim);margin-bottom:3px;}
.dt-pose-grid .v{font-size:12px;color:#e1f5ff;font-weight:600;white-space:nowrap;}
.dt-pose-foot{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px;padding-top:8px;border-top:1px solid rgba(255,255,255,.06);font-size:10px;color:var(--dim);}
.dt-pose-foot b{color:#88d8ff;font-weight:500;}
.dt-calib{padding:12px;display:flex;flex-direction:column;gap:9px;font-size:11px;color:var(--sub);}
.dt-calib h4{margin:4px 0 0;font-size:12px;color:var(--accent);font-weight:700;border-bottom:1px solid var(--line);padding-bottom:5px;}
.dt-calib label{display:flex;flex-direction:column;gap:4px;}
.dt-calib input[type=range]{width:100%;accent-color:#3fd4ff;}

/* 左下仪表 */
.dt-dash{position:absolute;left:14px;bottom:14px;z-index:10;display:flex;align-items:center;gap:14px;padding:10px 18px 10px 10px;}
.dt-gauge{position:relative;width:120px;height:120px;}
.dt-gauge-center{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;padding-top:4px;}
.dt-gauge-val{font-size:32px;font-weight:700;color:#fff;line-height:1;text-shadow:0 0 18px rgba(63,212,255,.45);}
.dt-gauge-unit{font-size:11px;color:var(--sub);margin-top:4px;}
.dt-dash-side{min-width:110px;}
.dt-dash-label{font-size:11px;color:var(--dim);margin-bottom:5px;}
.dt-steer-track{position:relative;height:6px;border-radius:3px;background:rgba(120,160,220,.14);margin:6px 0 6px;}
.dt-steer-center{position:absolute;left:50%;top:-3px;width:1px;height:12px;background:rgba(255,255,255,.25);}
.dt-steer-knob{position:absolute;top:50%;width:12px;height:12px;margin:-6px 0 0 -6px;border-radius:50%;background:var(--accent);box-shadow:0 0 10px var(--accent);transition:left .2s ease;}
.dt-steer-text{font-size:12px;color:var(--sub);}
.dt-battery{margin-top:12px;padding-top:9px;border-top:1px solid rgba(255,255,255,.07);color:var(--green);}
.dt-battery-head,.dt-battery-row{display:flex;align-items:center;justify-content:space-between;gap:8px;}
.dt-battery-head{margin-bottom:6px;}
.dt-battery-label{font-size:11px;color:var(--dim);}
.dt-battery-value{font-size:15px;font-weight:700;color:currentColor;line-height:1;}
.dt-battery-row{justify-content:flex-start;}
.dt-battery-icon{position:relative;width:34px;height:14px;padding:2px;border:1px solid currentColor;border-radius:3px;filter:drop-shadow(0 0 3px currentColor);flex-shrink:0;}
.dt-battery-icon::after{content:"";position:absolute;right:-5px;top:3px;width:3px;height:6px;border-radius:0 2px 2px 0;background:currentColor;}
.dt-battery-icon i{display:block;height:100%;min-width:0;border-radius:1px;background:currentColor;transition:width .45s ease;}
.dt-battery-detail{font-size:10px;color:var(--sub);white-space:nowrap;}
.dt-battery.low{color:var(--warn);}
.dt-battery.critical{color:var(--danger);animation:dt-breathe 1.2s ease-in-out infinite;}
.dt-battery.offline{color:var(--dim);}


/* 右下：相机 / 系统状态 */
.dt-cam-empty{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px;color:var(--dim);font-size:12px;text-align:center;overflow:hidden;
  background:repeating-linear-gradient(0deg,rgba(63,212,255,.03) 0 1px,transparent 1px 4px),#050910;}
.dt-cam-empty::after{content:"";position:absolute;left:0;right:0;height:40%;background:linear-gradient(180deg,transparent,rgba(63,212,255,.08),transparent);animation:dt-scan 3.2s linear infinite;}
.dt-cam-stale{position:absolute;top:10px;left:10px;padding:4px 10px;border-radius:8px;background:rgba(110,24,34,.85);border:1px solid rgba(255,120,130,.45);color:#ffdadd;font-size:12px;}
.dt-cam-empty small{font-size:10px;color:#34445a;font-family:ui-monospace,monospace;}
/* 相机通道开关 */
.dt-cam-switch button{display:inline-flex;align-items:center;gap:5px;}
.dt-cam-switch button small{font-size:10px;color:var(--dim);margin-left:1px;}
.dt-cam-switch button.on small{color:#bfeeff;}
.dt-cam-dot{display:inline-block;width:7px;height:7px;border-radius:50%;background:var(--dim);flex-shrink:0;}
.dt-cam-dot.ok{background:var(--green);box-shadow:0 0 6px var(--green);}
.dt-cam-dot.bad{background:var(--danger);box-shadow:0 0 6px var(--danger);}
.dt-cam-dot.wait{background:var(--warn);animation:dt-breathe 1.2s ease-in-out infinite;}
/* 通道下拉菜单（顶栏车辆连接 / 视频右上角相机） */
.dt-dd{position:relative;display:flex;}
.dt-dd-trigger{display:inline-flex;align-items:center;gap:4px;padding:0;border:0;background:transparent;color:inherit;font:inherit;cursor:pointer;}
.dt-dd-caret{font-size:10px;color:var(--dim);transition:transform .15s;}
.dt-dd-trigger.open .dt-dd-caret{transform:rotate(180deg);}
.dt-dd-trigger:hover .dt-chip,.dt-dd-trigger.open .dt-chip,.dt-dd-trigger:hover .dt-link,.dt-dd-trigger.open .dt-link{border-color:var(--line-strong);}
/* 顶栏车辆连接：信号格 + 通道名 */
.dt-link{display:inline-flex;align-items:center;gap:7px;padding:4px 10px;border-radius:999px;font-size:12px;white-space:nowrap;border:1px solid var(--line);background:rgba(255,255,255,.03);color:var(--text);}
.dt-link.wait{color:var(--sub);}
.dt-link.bad{color:#ffc2c8;border-color:rgba(255,91,107,.35);background:rgba(255,91,107,.08);}
.dt-sig{display:inline-flex;align-items:flex-end;gap:2px;height:12px;}
.dt-sig i{display:block;width:3px;border-radius:1px;background:rgba(147,167,195,.25);transition:background .3s;}
.dt-sig i:nth-child(1){height:4px}.dt-sig i:nth-child(2){height:7px}.dt-sig i:nth-child(3){height:10px}.dt-sig i:nth-child(4){height:12px}
.dt-link.ok .dt-sig i.on{background:var(--green);}
.dt-link.warn .dt-sig i.on{background:var(--warn);}
.dt-link.bad .dt-sig i.on{background:var(--danger);}
.dt-link.wait .dt-sig i{animation:dt-sig-wave 1.2s ease-in-out infinite;}
.dt-link.wait .dt-sig i:nth-child(2){animation-delay:.15s}.dt-link.wait .dt-sig i:nth-child(3){animation-delay:.3s}.dt-link.wait .dt-sig i:nth-child(4){animation-delay:.45s}
@keyframes dt-sig-wave{0%,60%,100%{background:rgba(147,167,195,.25)}30%{background:var(--sub)}}
.dt-dd-menu{position:absolute;top:calc(100% + 8px);z-index:200;width:340px;max-height:calc(100vh - 88px);overflow:auto;padding:8px;display:flex;flex-direction:column;gap:4px;animation:dt-rise .18s ease;background:rgba(9,15,28,.97);}
.dt-dd-menu.left{left:0;} .dt-dd-menu.right{right:0;}
.dt-dd-head{display:flex;align-items:center;justify-content:space-between;padding:2px 4px 6px;font-size:12px;color:var(--sub);border-bottom:1px solid var(--line);margin-bottom:2px;}
.dt-dd-head button{border:1px solid var(--line);background:rgba(255,255,255,.04);color:var(--sub);font-size:11px;padding:3px 8px;border-radius:6px;cursor:pointer;}
.dt-dd-head button:hover:not(:disabled){color:#fff;border-color:var(--line-strong);}
.dt-dd-head button:disabled{opacity:.6;cursor:progress;}
.dt-dd-row{display:flex;align-items:center;gap:10px;width:100%;padding:8px 10px;border-radius:8px;border:1px solid transparent;background:transparent;color:var(--text);text-align:left;cursor:pointer;font:inherit;}
.dt-dd-row:hover{background:rgba(63,212,255,.08);border-color:var(--line);}
.dt-dd-row.cur{background:rgba(63,212,255,.12);border-color:rgba(63,212,255,.35);cursor:default;}
.dt-dd-main{flex:1;min-width:0;display:flex;flex-direction:column;gap:2px;}
.dt-dd-main b{font-size:13px;font-weight:700;display:flex;align-items:center;gap:6px;}
.dt-dd-main em{font-style:normal;font-size:10px;font-weight:600;color:#9fe9ff;border:1px solid rgba(63,212,255,.45);border-radius:999px;padding:0 6px;}
.dt-dd-main small,.dt-dd-stat small{font-size:10px;color:var(--dim);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}
.dt-dd-stat{display:flex;flex-direction:column;align-items:flex-end;gap:2px;max-width:160px;min-width:0;}
.dt-dd-stat b{font-size:13px;font-weight:700;white-space:nowrap;}
.dt-dd-stat b.ok{color:var(--green);} .dt-dd-stat b.bad{color:var(--danger);} .dt-dd-stat b.wait{color:var(--warn);}
.dt-dd-foot{display:flex;gap:6px;padding:8px 4px 2px;margin-top:2px;border-top:1px solid var(--line);}
.dt-dd-foot .dt-input{flex:1;width:auto;min-width:0;}
.dt-cam-channel{position:absolute;top:10px;right:10px;z-index:5;display:flex;align-items:center;gap:6px;}
.dt-cam-tool{padding:4px 10px;border-radius:999px;background:rgba(5,9,16,.78);border:1px solid var(--line);color:var(--sub);font-size:12px;cursor:pointer;white-space:nowrap;}
.dt-cam-tool:hover{color:#fff;border-color:var(--line-strong);}
.dt-cam-box{position:absolute;inset:0;background:#000;touch-action:none;user-select:none;}
.dt-cam-box.draggable{cursor:ns-resize;}
.dt-cam-box .dt-cam-channel{cursor:default;}
/* 相机标题栏右侧：与左侧操作按键同一尺寸 */
.dt-cam-head{display:flex;align-items:center;gap:6px;}
.dt-cam-head .dt-action{width:auto;height:28px;padding:0 10px;gap:5px;font-size:12px;border-radius:8px;}
.dt-cam-head .dt-mode-seg{margin-right:0;}
.dt-cam-head .dt-dd-pill{height:28px;padding-top:0;padding-bottom:0;border-radius:8px;background:rgba(255,255,255,.03);border-color:rgba(255,255,255,.06);}
.dt-cam-box.full{position:fixed;inset:0;z-index:9000;}
.dt-cam-fullhint{position:absolute;top:12px;left:50%;transform:translateX(-50%);padding:5px 14px;border-radius:999px;background:rgba(5,9,16,.75);border:1px solid var(--line);color:var(--sub);font-size:12px;pointer-events:none;animation:dt-hint-fade 3s ease forwards;}
@keyframes dt-hint-fade{0%,70%{opacity:1}100%{opacity:0}}
/* 停车 / 取消路线 / 窄屏图标化 */
.dt-action-auto.is-stop{color:#fff;background:linear-gradient(135deg,#ff4d5e,#ff7a45);border-color:transparent;box-shadow:0 0 18px rgba(255,77,94,.4);}
.dt-action-auto.is-stop:hover:not(:disabled){filter:brightness(1.08);color:#fff;}
.dt-action-group{display:flex;align-items:center;gap:6px;}
.dt-action-cancel:hover:not(:disabled){color:#ffb0b8;border-color:rgba(255,91,107,.5);}
@media (max-width:1399px){
  .dt-hactions{gap:4px;}
  .dt-hactions .dt-action{padding:0 8px;gap:4px;font-size:11px;}
  .dt-hactions .dt-action svg{width:13px;height:13px;}
  .dt-mode-seg button{padding:3px 7px;}
}
/* 断线：仪表盘显示旧数据时置灰 */
.dt-dash.stale{opacity:.55;filter:grayscale(.7);}
/* 顶栏“系统”下拉内容 */
.dt-sys-item{border-radius:8px;}
.dt-sys-item.slow{background:rgba(255,181,71,.06);} .dt-sys-item.lost{background:rgba(255,91,107,.08);}
.dt-sys-sub{margin:0 10px 6px 40px;padding:2px 0;border-top:1px dashed rgba(255,255,255,.08);}
.dt-sys-sub div{padding:4px 0;font-size:11px;}
.dt-sys-row{display:flex;align-items:center;gap:10px;padding:8px 10px;}
.dt-sys-item{--c:var(--dim);}
.dt-sys-item.ok{--c:var(--green);} .dt-sys-item.slow{--c:var(--warn);} .dt-sys-item.lost{--c:var(--danger);} .dt-sys-item.idle{--c:#5b6b84;}
.dt-sys-icon{width:20px;height:20px;flex-shrink:0;color:var(--c);}
.dt-sys-main{flex:1;min-width:0;display:flex;flex-direction:column;gap:2px;}
.dt-sys-main b{font-size:13px;color:var(--text);}
.dt-sys-main small{font-size:11px;color:var(--sub);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}
.dt-sys-item.lost .dt-sys-main small{color:#ffb3ba;} .dt-sys-item.slow .dt-sys-main small{color:#ffd9a0;}
.dt-sys-side{display:flex;flex-direction:column;align-items:flex-end;gap:3px;flex-shrink:0;}
.dt-sys-side em{font-style:normal;font-size:11px;font-weight:700;padding:1px 8px;border-radius:999px;color:var(--c);border:1px solid var(--c);}
.dt-sys-side small{font-size:10px;color:var(--dim);}
.dt-sys-foot{font-size:10px;color:var(--dim);padding:6px 4px 2px;border-top:1px solid var(--line);margin-top:4px;line-height:1.5;}
.dt-diag-toolbar{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:4px 4px 6px;color:var(--sub);font-size:11px;}
.dt-diag-toolbar button{border:1px solid var(--line);background:rgba(63,212,255,.06);color:var(--accent);border-radius:6px;padding:4px 9px;font:inherit;cursor:pointer;}
.dt-diag-toolbar button:hover{border-color:var(--line-strong);background:rgba(63,212,255,.12);}
.dt-diag-summary{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:4px;}
.dt-diag-summary>span{display:flex;flex-direction:column;align-items:center;gap:2px;padding:8px 4px;border:1px solid var(--line);border-radius:8px;background:rgba(5,9,16,.52);}
.dt-diag-summary b{font-size:15px;color:var(--text);}.dt-diag-summary small{font-size:9px;color:var(--dim);}
.dt-diag-list{display:flex;flex-direction:column;gap:4px;}
.dt-diag-row{--c:var(--dim);display:grid;grid-template-columns:8px 88px 86px 1fr;align-items:center;gap:8px;padding:8px;border:1px solid rgba(255,255,255,.04);border-radius:8px;background:rgba(5,9,16,.34);}
.dt-diag-row.ok{--c:var(--green);}.dt-diag-row.slow{--c:var(--warn);background:rgba(255,181,71,.05);}.dt-diag-row.lost{--c:var(--danger);background:rgba(255,91,107,.07);}
.dt-diag-row>i{width:7px;height:7px;border-radius:50%;background:var(--c);box-shadow:0 0 9px color-mix(in srgb,var(--c) 70%,transparent);}
.dt-diag-name,.dt-diag-rate{display:flex;min-width:0;flex-direction:column;gap:2px;}.dt-diag-name b{font-size:12px;color:var(--text);}.dt-diag-name small,.dt-diag-rate small{font-size:9px;color:var(--dim);overflow:hidden;text-overflow:ellipsis;white-space:nowrap;}
.dt-diag-rate{align-items:flex-end;}.dt-diag-rate b{font-size:11px;color:var(--c);}
.dt-diag-detail{display:grid;grid-template-columns:auto 1fr auto 1fr;gap:2px 6px;min-width:0;padding-left:8px;border-left:1px solid var(--line);}
.dt-diag-detail small{font-size:9px;color:var(--dim);}.dt-diag-detail b{font-size:9px;color:var(--sub);font-weight:600;text-align:right;white-space:nowrap;}
.dt-diag-note{padding:6px 5px;color:var(--dim);font-size:9px;line-height:1.5;}
.dt-diag-section{margin-top:3px;padding:7px 4px 4px;border-top:1px solid var(--line);color:var(--sub);font-size:11px;font-weight:700;}
.dt-kv{display:flex;flex-direction:column;}
.dt-kv div{display:flex;justify-content:space-between;gap:12px;padding:7px 6px;border-bottom:1px solid rgba(255,255,255,.04);font-size:12px;}
.dt-kv div:last-child{border-bottom:0;}
.dt-kv span{color:var(--sub);flex-shrink:0;}
.dt-kv b{color:var(--text);font-weight:600;text-align:right;}
.dt-dd-pill{display:inline-flex;align-items:center;gap:6px;padding:4px 10px;border-radius:999px;background:rgba(5,9,16,.78);border:1px solid var(--line);color:var(--text);font-size:12px;white-space:nowrap;}
.dt-dd-pill b{font-size:11px;color:var(--sub);font-weight:600;}
.dt-dd-camera .dt-dd-trigger{padding:0;}
.dt-dd-camera .dt-dd-trigger:hover .dt-dd-pill,.dt-dd-camera .dt-dd-trigger.open .dt-dd-pill{border-color:var(--line-strong);}
.dt-dd-camera .dt-dd-caret{margin-left:-22px;margin-right:9px;position:relative;z-index:1;}
.dt-dd-camera .dt-dd-pill{padding-right:22px;}

/* 地图 */
/* 卫星图不加 CSS filter：滤镜会让每块瓦片多走一遍 GPU 合成，拖动/缩放时明显掉帧 */
.dt-map-vignette{position:absolute;inset:0;pointer-events:none;z-index:400;box-shadow:inset 0 0 36px rgba(3,6,14,.45);}
.dt-map-coords{position:absolute;left:12px;bottom:12px;z-index:1000;padding:7px 11px;border-radius:9px;background:rgba(9,15,28,.93);border:1px solid var(--line);font-size:11px;color:var(--sub);}
.dt-map-legend{display:flex;align-items:center;gap:12px;color:var(--sub);font-size:11px;white-space:nowrap;}
.dt-map-legend i{display:inline-block;width:14px;height:4px;border-radius:2px;margin-right:6px;vertical-align:middle;}
.dt-map-legend i.dot{width:9px;height:9px;border-radius:50%;border:1.5px solid #fff;}
.dt-map-coords b{color:var(--green);font-weight:500;margin-left:4px;}
.dt-ego-marker,.dt-pin-marker{background:transparent;border:0;}
.dt-ego{position:relative;width:40px;height:40px;display:flex;align-items:center;justify-content:center;}
.dt-ego-pulse{position:absolute;inset:6px;border-radius:50%;background:rgba(52,245,164,.35);animation:dt-pulse 1.8s ease-out infinite;}
.dt-ego-core{position:absolute;inset:11px;border-radius:50%;background:rgba(6,20,14,.75);border:1px solid rgba(52,245,164,.6);}
.dt-ego-arrow{position:relative;filter:drop-shadow(0 0 6px rgba(52,245,164,.9));transition:transform .15s linear;}
.dt-pin{width:28px;height:28px;border-radius:50% 50% 50% 0;transform:rotate(-45deg);display:flex;align-items:center;justify-content:center;border:2px solid #fff;}
.dt-pin span{transform:rotate(45deg);color:#fff;font-size:12px;font-weight:800;}
.dt-pin-start{background:#1f9bff;box-shadow:0 0 14px rgba(31,155,255,.8);}
.dt-pin-goal{background:#ff4d5e;box-shadow:0 0 14px rgba(255,77,94,.8);}

/* 提示条 */
.dt-notice{position:absolute;top:52px;left:0;right:0;z-index:35;display:flex;align-items:center;gap:10px;padding:7px 16px;background:rgba(110,24,34,.94);border-bottom:1px solid rgba(255,120,130,.35);color:#ffdadd;font-size:12px;}
.dt-notice button{margin-left:auto;padding:3px 10px;border-radius:6px;border:1px solid rgba(255,150,160,.5);background:transparent;color:#ffdadd;font-size:11px;cursor:pointer;flex-shrink:0;}
.dt-reload-banner{position:absolute;top:62px;left:50%;transform:translateX(-50%);z-index:80;display:flex;align-items:center;gap:10px;padding:7px 14px;border-radius:10px;background:rgba(70,40,0,.94);border:1px solid var(--warn);color:#ffe2a8;font-size:12px;white-space:nowrap;box-shadow:0 4px 18px rgba(0,0,0,.4);}
.dt-reload-banner button{padding:3px 10px;border-radius:6px;border:1px solid var(--warn);background:transparent;color:#ffcf6e;font-size:12px;cursor:pointer;}

/* 面板副标题 */
.dt-panel-sub{font-size:11px;color:var(--dim);letter-spacing:1px;margin-left:4px;flex:1 1 0;min-width:0;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}

/* 驾驶舱中的任务数据 */
.dt-dash-divider{width:1px;align-self:stretch;margin:6px 2px;background:linear-gradient(transparent,rgba(63,212,255,.35),transparent);}
.dt-dash-stats{display:flex;flex-direction:column;gap:9px;min-width:104px;}
.dt-dash-stats .dt-stat-val{font-size:19px;}


/* 手动遥控 */
.dt-action.is-teleop{color:#fff;background:rgba(255,160,40,.28);border-color:rgba(255,181,71,.7);box-shadow:0 0 16px rgba(255,181,71,.3);}
.dt-live-dot.warn{background:var(--warn);box-shadow:0 0 10px var(--warn);}
.dt-teleop{--tx:-50%;position:absolute;left:50%;bottom:168px;transform:translateX(-50%);z-index:25;padding:12px 16px;border-color:rgba(255,181,71,.4);animation:dt-rise .35s ease;}
.dt-teleop-title{display:flex;align-items:center;gap:8px;font-size:13px;font-weight:700;color:#ffe0ad;letter-spacing:1px;margin-bottom:10px;}
.dt-teleop-body{display:flex;align-items:center;gap:16px;}
.dt-keypad{display:grid;grid-template-columns:repeat(3,38px);grid-template-rows:repeat(2,38px);gap:5px;}
.dt-key{display:flex;align-items:center;justify-content:center;border-radius:8px;border:1px solid rgba(255,255,255,.14);background:rgba(255,255,255,.05);color:var(--sub);font-size:16px;font-weight:700;box-shadow:inset 0 -2px 0 rgba(0,0,0,.35);transition:all .08s;}
.dt-key.wide{height:38px;padding:0 16px;font-size:13px;white-space:nowrap;}
.dt-key.on{color:#1a1000;background:linear-gradient(135deg,#ffcf6e,#ffb547);border-color:transparent;box-shadow:0 0 16px rgba(255,181,71,.7);transform:translateY(1px);}
.dt-teleop-side{display:flex;flex-direction:column;gap:8px;}
.dt-teleop-tip{font-size:11px;color:var(--dim);}

/* 设置抽屉 */
.dt-icon-btn{width:34px;height:34px;display:flex;align-items:center;justify-content:center;border-radius:9px;border:1px solid var(--line);background:rgba(255,255,255,.04);color:var(--sub);cursor:pointer;transition:all .15s;}
.dt-icon-btn:hover,.dt-icon-btn.on{color:#fff;border-color:var(--line-strong);background:rgba(63,212,255,.12);}
.dt-settings{position:absolute;top:62px;right:14px;z-index:60;width:300px;max-height:calc(100vh - 80px);overflow-y:auto;padding:14px;display:flex;flex-direction:column;gap:10px;font-size:12px;color:var(--sub);animation:dt-rise .25s ease;scrollbar-width:thin;scrollbar-color:#2a3d55 transparent;}
.dt-settings-head{display:flex;align-items:center;justify-content:space-between;font-size:14px;font-weight:700;color:var(--text);}
.dt-settings-head button{border:0;background:transparent;color:var(--sub);font-size:20px;cursor:pointer;line-height:1;}
.dt-settings h4{margin:6px 0 0;font-size:12px;color:var(--accent);font-weight:700;border-bottom:1px solid var(--line);padding-bottom:5px;}
.dt-settings label{display:flex;flex-direction:column;gap:4px;}
.dt-settings input[type=range]{width:100%;accent-color:#3fd4ff;}
.dt-settings .dt-input{width:100%;}
.dt-settings-row{display:flex;gap:8px;}
.dt-settings-row .dt-btn{flex:1;padding:7px 8px;}

/* 开场自检 */
.dt-boot{position:absolute;inset:0;z-index:9999;display:flex;align-items:center;justify-content:center;background:radial-gradient(900px 500px at 50% 45%,#0d2242,#04070e 70%);transition:opacity .9s ease;}
.dt-boot.leaving{opacity:0;pointer-events:none;}
.dt-boot-grid{position:absolute;inset:0;opacity:.35;background-image:linear-gradient(rgba(63,212,255,.12) 1px,transparent 1px),linear-gradient(90deg,rgba(63,212,255,.12) 1px,transparent 1px);background-size:48px 48px;mask-image:radial-gradient(circle at 50% 50%,#000 10%,transparent 65%);-webkit-mask-image:radial-gradient(circle at 50% 50%,#000 10%,transparent 65%);}
.dt-boot-card{position:relative;display:flex;flex-direction:column;align-items:center;width:min(500px,calc(100% - 32px));animation:dt-rise .6s ease;}
.dt-boot-logo{animation:dt-spin 6s linear infinite;filter:drop-shadow(0 0 18px rgba(63,212,255,.6));}
.dt-boot-name{margin-top:14px;font-size:26px;font-weight:800;letter-spacing:8px;background:linear-gradient(90deg,#e6f1ff,#9fe9ff,#34f5a4);-webkit-background-clip:text;background-clip:text;color:transparent;}
.dt-boot-sub{margin-top:6px;font-size:12px;color:var(--sub);letter-spacing:4px;}
.dt-boot-bar{width:100%;height:4px;margin:22px 0 14px;border-radius:2px;background:rgba(120,160,220,.15);overflow:hidden;}
.dt-boot-bar i{display:block;height:100%;border-radius:2px;background:linear-gradient(90deg,#1f7bff,#3fd4ff,#34f5a4);box-shadow:0 0 12px rgba(63,212,255,.8);transition:width .5s ease;}
.dt-boot-bar.warn i{background:linear-gradient(90deg,#1f7bff,#3fd4ff,#ffb547);}
.dt-boot-list{width:100%;display:flex;flex-direction:column;gap:6px;}
.dt-boot-row{display:flex;align-items:center;gap:12px;padding:9px 12px;border-radius:10px;border:1px solid var(--line);background:rgba(9,15,28,.6);transition:border-color .3s,background .3s;}
.dt-boot-row.ok{border-color:rgba(52,245,164,.25);}
.dt-boot-row.warn{border-color:rgba(255,181,71,.4);background:rgba(255,181,71,.06);}
.dt-boot-row.fail{border-color:rgba(255,91,107,.45);background:rgba(255,91,107,.07);}
.dt-boot-check{width:20px;height:20px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:800;border:1.5px solid rgba(120,160,220,.3);flex-shrink:0;}
.dt-boot-row.wait .dt-boot-check{border-top-color:var(--accent);animation:dt-spin 1s linear infinite;}
.dt-boot-row.ok .dt-boot-check{background:var(--green);border-color:var(--green);color:#062014;box-shadow:0 0 10px rgba(52,245,164,.6);}
.dt-boot-row.warn .dt-boot-check{background:var(--warn);border-color:var(--warn);color:#1a1000;}
.dt-boot-row.fail .dt-boot-check{background:var(--danger);border-color:var(--danger);color:#fff;}
.dt-boot-text{flex:1;min-width:0;display:flex;flex-direction:column;gap:2px;}
.dt-boot-text b{font-size:13px;color:var(--text);font-weight:700;}
.dt-boot-text small{font-size:11px;color:var(--sub);line-height:1.4;}
.dt-boot-row.ok .dt-boot-text small{color:#9fdcc0;}
.dt-boot-row.warn .dt-boot-text small{color:#ffd9a0;}
.dt-boot-row.fail .dt-boot-text small{color:#ffb3ba;}
.dt-boot-row .dt-btn{flex-shrink:0;}
.dt-boot-foot{width:100%;display:flex;align-items:center;justify-content:space-between;margin-top:14px;font-size:12px;color:var(--sub);}
.dt-boot-foot .dt-btn{padding:6px 16px;}


@keyframes dt-rise{from{opacity:0;transform:translate(var(--tx,0),10px)}to{opacity:1;transform:translate(var(--tx,0),0)}}
@keyframes dt-spin{to{transform:rotate(360deg)}}
@keyframes dt-pulse{0%{transform:scale(1);opacity:.75}100%{transform:scale(2.6);opacity:0}}
@keyframes dt-breathe{0%,100%{opacity:1}50%{opacity:.4}}
@keyframes dt-scan{0%{top:-40%}100%{top:100%}}
`;

// ── 读取 public/config.json：缺失、超时或写错的项都回退到代码里的默认值，不会白屏 ──
function applyConfig(cfg) {
  const warn = (m) => console.warn(`⚠️ config.json：${m}，该项使用代码里的默认值`);
  const channels = (v, key) => {
    if (v === undefined) return null;
    const ok = Array.isArray(v) && v.length > 0 && v.every((c) => c && typeof c.name === "string" && c.name.trim() && typeof c.url === "string" && c.url.trim());
    if (!ok) { warn(`${key} 格式不对（应为 [{ "name": "...", "url": "..." }]）`); return null; }
    const list = v.map((c) => ({ name: c.name.trim(), url: c.url.trim() }));
    if (new Set(list.map((c) => c.name)).size !== list.length) { warn(`${key} 里有重名的通道`); return null; }
    return list;
  };
  const vc = channels(cfg.vehicleChannels, "vehicleChannels");
  if (vc && !LAN_ONLY_MODE) VEHICLE_CHANNELS = vc;
  const cc = channels(cfg.cameraChannels, "cameraChannels");
  if (cc && !LAN_ONLY_MODE) CAMERA_CHANNELS = cc;
  if (cfg.chassisGateway !== undefined) {
    const g = cfg.chassisGateway;
    const enabled = g && typeof g.enabled === "boolean" ? g.enabled : CHASSIS_GATEWAY.enabled;
    const urlValid = g?.url === undefined || (typeof g.url === "string" && /^wss?:\/\//i.test(g.url.trim()));
    const url = typeof g?.url === "string" && urlValid ? g.url.trim() : CHASSIS_GATEWAY.url;
    const token = typeof g?.token === "string" ? g.token : CHASSIS_GATEWAY.token;
    if (!g || typeof g !== "object" || !urlValid) {
      warn("chassisGateway 格式不对（应为 { enabled, url, token }，url 使用 ws:// 或 wss://）");
    } else {
      CHASSIS_GATEWAY = { enabled, url, token };
    }
  }
  if (cfg.rosUdpTelemetry !== undefined) {
    const u = cfg.rosUdpTelemetry;
    if (!u || typeof u !== "object") {
      warn("rosUdpTelemetry 格式不对（应包含 enabled/listenPort 及 objects/trajectory/path 的启用与回退配置）");
    } else {
      const next = { ...ROS_UDP_TELEMETRY };
      if (typeof u.enabled === "boolean") next.enabled = u.enabled;
      if (typeof u.listenAddress === "string" && u.listenAddress.trim()) next.listenAddress = u.listenAddress.trim();
      const port = Number(u.listenPort);
      if (Number.isInteger(port) && port >= 1 && port <= 65535) next.listenPort = port;
      else if (u.listenPort !== undefined) warn("rosUdpTelemetry.listenPort 必须是 1~65535 的整数");
      if (typeof u.allowedSource === "string") next.allowedSource = u.allowedSource.trim();
      const fallbackMs = Number(u.fallbackMs);
      if (Number.isFinite(fallbackMs) && fallbackMs >= 500) next.fallbackMs = Math.round(fallbackMs);
      else if (u.fallbackMs !== undefined) warn("rosUdpTelemetry.fallbackMs 必须不小于 500 ms");
      if (typeof u.objectsEnabled === "boolean") next.objectsEnabled = u.objectsEnabled;
      const objectsFallbackMs = Number(u.objectsFallbackMs);
      if (Number.isFinite(objectsFallbackMs) && objectsFallbackMs >= 500) next.objectsFallbackMs = Math.round(objectsFallbackMs);
      else if (u.objectsFallbackMs !== undefined) warn("rosUdpTelemetry.objectsFallbackMs 必须不小于 500 ms");
      if (typeof u.trajectoryEnabled === "boolean") next.trajectoryEnabled = u.trajectoryEnabled;
      const trajectoryFallbackMs = Number(u.trajectoryFallbackMs);
      if (Number.isFinite(trajectoryFallbackMs) && trajectoryFallbackMs >= 500) next.trajectoryFallbackMs = Math.round(trajectoryFallbackMs);
      else if (u.trajectoryFallbackMs !== undefined) warn("rosUdpTelemetry.trajectoryFallbackMs 必须不小于 500 ms");
      if (typeof u.pathEnabled === "boolean") next.pathEnabled = u.pathEnabled;
      const pathFallbackMs = Number(u.pathFallbackMs);
      if (Number.isFinite(pathFallbackMs) && pathFallbackMs >= 500) next.pathFallbackMs = Math.round(pathFallbackMs);
      else if (u.pathFallbackMs !== undefined) warn("rosUdpTelemetry.pathFallbackMs 必须不小于 500 ms");
      ROS_UDP_TELEMETRY = next;
    }
  }
  if (cfg.mapOrigin !== undefined) {
    const lat = Number(cfg.mapOrigin?.lat);
    const lng = Number(cfg.mapOrigin?.lng);
    if (Number.isFinite(lat) && Number.isFinite(lng) && Math.abs(lat) <= 90 && Math.abs(lng) <= 180) MAP_ORIGIN = { lat, lng };
    else warn("mapOrigin 格式不对（应为 { \"lat\": 纬度, \"lng\": 经度 }）");
  }
  const types = ["MGRS", "LocalCartesianUTM", "LocalCartesian"];
  let type = MAP_PROJECTION.type;
  let originAlt = MAP_PROJECTION.originAlt;
  if (cfg.mapProjection !== undefined) {
    if (types.includes(cfg.mapProjection)) type = cfg.mapProjection;
    else warn(`mapProjection 只能是 ${types.join(" / ")}`);
  }
  if (cfg.mapOriginAlt !== undefined) {
    const a = Number(cfg.mapOriginAlt);
    if (Number.isFinite(a)) originAlt = a;
    else warn("mapOriginAlt 应为数字");
  }
  MAP_PROJECTION = { type, originLat: MAP_ORIGIN.lat, originLon: MAP_ORIGIN.lng, originAlt };
  if (cfg.tiandituKey !== undefined) {
    if (typeof cfg.tiandituKey === "string" && cfg.tiandituKey.trim()) TIANDITU_TK = cfg.tiandituKey.trim();
    else warn("tiandituKey 应为字符串");
  }
}

export default function App() {
  const [ready, setReady] = useState(false);
  useEffect(() => {
    let done = false;
    const finish = () => { if (!done) { done = true; setReady(true); } };
    const timer = setTimeout(() => { console.warn("⚠️ 读取 config.json 超时，使用代码里的默认配置"); finish(); }, 3000);
    fetch(CONFIG_URL, { cache: "no-store" })
      .then((r) => { if (!r.ok) throw new Error(`HTTP ${r.status}`); return r.json(); })
      .then((cfg) => {
        if (done) return;
        applyConfig(cfg && typeof cfg === "object" ? cfg : {});
        console.info("⚙️ 已读取 config.json", cfg);
      })
      .catch((err) => console.warn(`⚠️ 未读取到 config.json（${err.message}），使用代码里的默认配置`))
      .finally(() => { clearTimeout(timer); finish(); });
    return () => clearTimeout(timer);
  }, []);
  if (!ready) return <div style={{ width: "100%", height: "100vh", background: "#04070e" }} />;
  return <MainApp />;
}

function MainApp() {
  // 默认使用低延迟跟随视角；巡航视角仍可手动选择。
  const [viewMode, setViewMode] = useState("follow");
  const [poseMode, setPoseMode] = useState(null);
  const [pointSize] = useState(0.05);
  const [clock, setClock] = useState(() => new Date());

  // 车辆（rosbridge）地址：网址 ?ip= 优先，其次是用户手动设置过并保存的地址，最后是默认值。
  // 只连这一个地址，不会自动换通道；在“智能驾驶实况”标题栏的通道开关上手动切换。
  // 记住的是通道名称（如“公网”），config.json 里改了地址也能对上；自定义地址单独记
  const [appliedIp, setAppliedIp] = useState(() => {
    if (LAN_ONLY_MODE) {
      try {
        localStorage.removeItem("dt_ip");
        localStorage.removeItem("dt_vehicle_channel");
        localStorage.removeItem("dt_vehicle_custom");
      } catch { /* 隐私模式等忽略 */ }
      return VEHICLE_CHANNELS[0].url;
    }
    const q = new URLSearchParams(window.location.search).get("ip");
    if (q && q.trim()) return q.trim();
    try {
      localStorage.removeItem("dt_ip"); // 旧版本按地址记忆，已不再使用
      const name = localStorage.getItem("dt_vehicle_channel");
      const ch = VEHICLE_CHANNELS.find((c) => c.name === name);
      if (ch) return ch.url;
      const custom = localStorage.getItem("dt_vehicle_custom");
      if (name === "@custom" && custom) return custom;
    } catch { /* 隐私模式等忽略 */ }
    return VEHICLE_CHANNELS[0].url;
  });
  // 通道开关：点哪个就连哪个，并记住选择
  const selectVehicleChannel = useCallback((url) => {
    setAppliedIp(url);
    try {
      const ch = VEHICLE_CHANNELS.find((c) => c.url === url);
      if (ch) {
        localStorage.setItem("dt_vehicle_channel", ch.name);
      } else {
        localStorage.setItem("dt_vehicle_channel", "@custom");
        localStorage.setItem("dt_vehicle_custom", url);
      }
    } catch { /* 忽略 */ }
  }, []);
  const [currentCar] = useState(CAR_MODELS[0]);

  // 地图/车身校准：沿用之前在浏览器中保存的校准结果
  const [pcdConfig] = useState(() => {
    try {
      const saved = JSON.parse(localStorage.getItem("dt_pcdConfig") || "null");
      if (saved && Number.isFinite(saved.x) && Number.isFinite(saved.z) && Number.isFinite(saved.rotation)) {
        return { x: saved.x, y: Number.isFinite(saved.y) ? saved.y : 0, z: saved.z, rotation: saved.rotation };
      }
    } catch { /* 忽略损坏数据 */ }
    return { x: 0, y: 0, z: 0, rotation: 0 };
  });
  const [carOffset] = useState(() => {
    try {
      const saved = JSON.parse(localStorage.getItem("dt_carOffset") || "null");
      if (saved && Number.isFinite(saved.x) && Number.isFinite(saved.z)) return { x: saved.x, z: saved.z };
    } catch { /* 忽略损坏数据 */ }
    return { x: 0, z: 0 };
  });

  // ── 相机画面（MediaMTX WebRTC），右下角只显示相机 ──
  // 铺满（裁剪上下）/ 完整（留黑边）：记在浏览器里
  const [camFit, setCamFitState] = useState(() => {
    try { return localStorage.getItem("dt_cam_fit") === "contain" ? "contain" : "cover"; } catch { return "cover"; }
  });
  const setCamFit = useCallback((next) => {
    setCamFitState(next);
    try { localStorage.setItem("dt_cam_fit", next); } catch { /* 忽略 */ }
  }, []);
  // 铺满时的纵向位置：按住画面上下拖动调整，小窗和全屏共用，记在浏览器里
  const [camFocusY, setCamFocusY] = useState(() => {
    try {
      const v = Number(localStorage.getItem("dt_cam_focus"));
      if (localStorage.getItem("dt_cam_focus") !== null && Number.isFinite(v)) return Math.min(100, Math.max(0, v));
    } catch { /* 忽略 */ }
    return CAMERA_FOCUS_Y;
  });
  const camDragRef = useRef(null);
  const saveCamFocus = (v) => { try { localStorage.setItem("dt_cam_focus", String(Math.round(v))); } catch { /* 忽略 */ } };
  const onCamPointerDown = (e) => {
    if (camFit !== "cover" || e.button !== 0 || e.target.closest(".dt-cam-channel")) return;
    const box = e.currentTarget;
    const video = box.querySelector("video");
    if (!video || !video.videoWidth) return;
    // 画面按“铺满”放大后，比显示区域高出的部分就是可以上下移动的范围
    const scaledH = Math.max(box.clientWidth / video.videoWidth, box.clientHeight / video.videoHeight) * video.videoHeight;
    const overflow = scaledH - box.clientHeight;
    if (overflow < 2) return;
    camDragRef.current = { y: e.clientY, start: camFocusY, overflow, moved: false, value: camFocusY };
    box.setPointerCapture?.(e.pointerId);
  };
  const onCamPointerMove = (e) => {
    const d = camDragRef.current;
    if (!d) return;
    const dy = e.clientY - d.y;
    if (!d.moved && Math.abs(dy) < 4) return; // 移动超过几个像素才算拖动，不影响双击
    d.moved = true;
    d.value = Math.min(100, Math.max(0, d.start - (dy / d.overflow) * 100));
    setCamFocusY(d.value);
  };
  const onCamPointerUp = () => {
    const d = camDragRef.current;
    camDragRef.current = null;
    if (d?.moved) saveCamFocus(d.value);
  };
  const resetCamFocus = () => { setCamFocusY(CAMERA_FOCUS_Y); saveCamFocus(CAMERA_FOCUS_Y); };

  // 双击放大到整个窗口，再双击或按 Esc 恢复（播放器不重新连接）
  const [camFull, setCamFull] = useState(false);
  useEffect(() => {
    if (!camFull) return;
    const onKey = (e) => { if (e.key === "Escape") setCamFull(false); };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [camFull]);
  const [camStatus, setCamStatus] = useState("idle");
  const [camStats, setCamStats] = useState(null);
  // 相机通道：网址 ?cam=地址1,地址2 优先，否则用 CAMERA_CHANNELS；在标题栏手动切换，不会自动切换
  const cameraChannels = useMemo(() => {
    if (LAN_ONLY_MODE) return CAMERA_CHANNELS;
    const q = new URLSearchParams(window.location.search).get("cam");
    const list = q && q.trim()
      ? q.split(",").map((u) => u.trim()).filter(Boolean).map((url, i) => ({ name: `通道${i + 1}`, url }))
      : CAMERA_CHANNELS;
    return list.length ? list : CAMERA_CHANNELS;
  }, []);
  const cameraActive = true;
  // 当前视频已正常播放后不再自动拉起备用 WebRTC 流，避免周期性抢占同一链路带宽。
  const camChannel = useCameraChannel(cameraChannels, cameraActive && camStatus !== "playing");
  const cameraUrl = useMemo(() => normalizeCameraUrl(camChannel.current.url), [camChannel.current]);
  const camChannelMenu = (
    <CameraChannelMenu
      channels={cameraChannels}
      current={camChannel.current}
      results={camChannel.results}
      probing={camChannel.probing}
      liveStatus={camStatus}
      liveStats={camStats}
      onSelect={camChannel.select}
      onProbe={camChannel.probeOne}
    />
  );

  useEffect(() => {
    const id = setInterval(() => setClock(new Date()), 1000);
    return () => clearInterval(id);
  }, []);

  const rosSim = useSimulation(appliedIp);
  const directCan = useDirectCanGateway(CHASSIS_GATEWAY);
  const directCcuFresh = Boolean(directCan.ccuFeedbackAt) && Date.now() - directCan.ccuFeedbackAt < CHASSIS_STALE_MS;
  const directTopicHealth = CHASSIS_GATEWAY.enabled ? {
    ...(rosSim.topicHealth || {}),
    [YUNLE_FEEDBACK_TOPIC]: {
      ageMs: directCan.status?.ccu_age_ms >= 0 ? directCan.status.ccu_age_ms : (directCan.ccuFeedbackAt ? Date.now() - directCan.ccuFeedbackAt : null),
      hz: Number(directCan.status?.ccu_hz) || 0,
    },
  } : rosSim.topicHealth;
  // 只替换底盘相关数据与动作；定位、规划、路线等仍由 Autoware/rosbridge 提供。
  const sim = CHASSIS_GATEWAY.enabled ? {
    ...rosSim,
    ego: directCcuFresh && directCan.ccuStatus ? {
      ...rosSim.ego,
      speed: (Number(directCan.ccuStatus.ccu_vehicle_speed) || 0) / 3.6,
      // CAN/CCU 原生方向为右正左负；界面与 Autoware 统一使用左正右负。
      steering: -((Number(directCan.ccuStatus.ccu_steering_wheel_angle) || 0) * Math.PI) / 180,
    } : rosSim.ego,
    chassisStatus: directCan.ccuStatus,
    chassisFeedbackAt: directCan.ccuFeedbackAt,
    bmsStatus: directCan.bmsStatus,
    bmsFeedbackAt: directCan.bmsFeedbackAt,
    topicHealth: directTopicHealth,
    requestRemoteMode: directCan.requestRemoteMode,
    publishCmdVel: directCan.publishCmdVel,
    manualError: directCan.manualError,
  } : rosSim;
  // 当前通道已连上后停止并取消备用通道的自动重数据测速；用户仍可在菜单中手动测试。
  const vehicleProbe = useVehicleChannelProbe(VEHICLE_CHANNELS, appliedIp, !sim.connected);
  // 断线（或从未连上）多久了：5 秒内显示黄色“连接中”，超过显示红色“未连接”
  const [disconnectedAt, setDisconnectedAt] = useState(() => Date.now());
  useEffect(() => {
    setDisconnectedAt(sim.connected ? null : Date.now());
  }, [sim.connected, appliedIp]);
  const connectingLong = !sim.connected && disconnectedAt != null && clock.getTime() - disconnectedAt > 5000;
  const objFresh = Boolean(sim.objectsAt) && Date.now() - sim.objectsAt < OBJ_STALE_MS;
  // 车辆连接下拉里的参考信息：最近约 5 秒定位到达抖动的 80 分位（没有定位数据时不显示）
  const linkJitterMs = (() => {
    const ex = sim.poseTimelineRef?.current?.excess;
    if (!sim.connected || !ex || ex.length < 10) return null;
    const recent = ex.slice(-50).sort((a, b) => a - b);
    return recent[Math.floor(recent.length * 0.8)];
  })();
  const { ego, startPoint, goalPoint, globalPath, globalPath3d, startPose, goalPose, startStatus, initFromGnss, gnss, publishPose, operationMode, autoRequestStatus, autoFail, requestAutonomousMode, requestRemoteMode, publishCmdVel, connected, topicHealth, locInitState, controlCmd, manualError } = sim;
  const teleopConnected = CHASSIS_GATEWAY.enabled ? directCan.connected : connected;
  const isAutonomous = operationMode?.mode === 2 || operationMode?.mode === 1002;
  const autoAvailable = operationMode?.is_autonomous_mode_available ?? operationMode?.is_autonomous_available ?? true;
  const autoTransitioning = operationMode?.is_in_transition || autoRequestStatus === "pending";

  // 规划路线上最近的障碍物距离（由 3D 场景计算后回报；null 表示路线上没有障碍物）
  const [obstacleAhead, setObstacleAhead] = useState(null);


  // ── 仿真 / 实车模式（网址 ?mode=real 可直接进入实车模式） ──
  const [driveMode, setDriveModeState] = useState(() => {
    const q = new URLSearchParams(window.location.search).get("mode");
    if (q === "real" || q === "sim") return q;
    try { return localStorage.getItem("dt_driveMode") === "real" ? "real" : "sim"; } catch { return "sim"; }
  });
  const setDriveMode = useCallback((m) => {
    setDriveModeState(m);
    try { localStorage.setItem("dt_driveMode", m); } catch { /* 忽略 */ }
  }, []);
  // 仿真障碍物：当前预设、放置 / 删除 / 清空
  const [obstaclePreset, setObstaclePreset] = useState(OBSTACLE_PRESETS[0]);
  const placeObstacle = useCallback((x, y, yaw) => sim.addObstacle(x, y, yaw, obstaclePreset), [sim.addObstacle, obstaclePreset]); // eslint-disable-line react-hooks/exhaustive-deps
  const removeObstacle = sim.removeObstacle;
  // 切到实车模式时退出障碍物放置（dummy 障碍物只用于仿真）
  useEffect(() => {
    if (driveMode !== "sim" && poseMode === "obstacle") setPoseMode(null);
  }, [driveMode, poseMode]);
  const onGnssInit = useCallback(() => {
    setPoseMode(null);
    initFromGnss(); // 工控机自行初始化，完成后读取工控机的小车位置显示
  }, [initFromGnss]);
  const gnssBusy = startStatus?.kind === "gnss" && (startStatus.state === "pending" || startStatus.state === "verifying");
  // 断线时退出设起点 / 设终点 / 放障碍物（这些操作要发给车）
  useEffect(() => {
    if (!connected) setPoseMode(null);
  }, [connected]);

  // 卫星图与“距目的地”使用 Autoware 规划路径
  const mapPath = globalPath;

  // ── 开场检查：各项随真实状态更新；全部就绪后淡出，有失败项时显示原因并倒计时进入 ──
  const [assets, setAssets] = useState({ map: null, pcd: null, model: null, tiles: null });
  const onAssetStatus = useCallback((key, info) => setAssets((a) => (a[key] ? a : { ...a, [key]: info || true })), []);
  const [bootPhase, setBootPhase] = useState("show"); // show → leaving → gone
  const [bootStartAt, setBootStartAt] = useState(() => performance.now());
  const [bootNow, setBootNow] = useState(() => performance.now());
  useEffect(() => {
    if (bootPhase !== "show") return;
    const id = setInterval(() => setBootNow(performance.now()), 250);
    return () => clearInterval(id);
  }, [bootPhase]);
  const bootElapsed = bootNow - bootStartAt;
  const bootTimedOut = bootElapsed > BOOT_MAX_MS;
  const restartBoot = () => { const t = performance.now(); setBootStartAt(t); setBootNow(t); };
  const otherVehicleChannel = VEHICLE_CHANNELS.find((c) => c.url !== appliedIp);
  const curVehicleChannel = VEHICLE_CHANNELS.find((c) => c.url === appliedIp);
  const otherCamChannel = cameraChannels.find((c) => c.name !== camChannel.current.name);
  const locT = topicLevel(topicHealth?.[LOC_TOPIC], HEALTH_EXPECT_HZ.loc);

  const bootItems = (() => {
    const vName = curVehicleChannel ? curVehicleChannel.name : "自定义地址";
    const vehicle = connected
      ? { state: "ok", detail: `${vName} · ${sim.activeUrl}` }
      : sim.connectionError
        ? { state: "fail", detail: sim.connectionError,
            action: otherVehicleChannel ? { label: `改用${otherVehicleChannel.name}`, onClick: () => { selectVehicleChannel(otherVehicleChannel.url); restartBoot(); } } : null }
      : bootTimedOut
        ? { state: "fail", detail: `${vName}（${appliedIp}）连接失败（车辆未开机、网络不通或 rosbridge 未启动）`,
            action: otherVehicleChannel ? { label: `改用${otherVehicleChannel.name}`, onClick: () => { selectVehicleChannel(otherVehicleChannel.url); restartBoot(); } } : null }
        : { state: "wait", detail: `正在连接 ${vName}（${sim.activeUrl || appliedIp}）…` };

    let loc;
    // 底盘反馈：按每条消息的到达统计（topicHealth）判断，3 秒内收到过即在线，并显示实收频率便于排查时断时续
    const chassisH = topicHealth?.[YUNLE_FEEDBACK_TOPIC];
    const chassisFresh = chassisH?.ageMs != null && chassisH.ageMs < CHASSIS_STALE_MS;
    const chassisLinkConnected = CHASSIS_GATEWAY.enabled ? directCan.connected : connected;
    if (!chassisLinkConnected) loc = bootTimedOut
      ? { state: "fail", detail: CHASSIS_GATEWAY.enabled ? (directCan.connectionError || "未连接底盘 CAN 网关") : "未连接车辆" }
      : { state: "wait", detail: CHASSIS_GATEWAY.enabled ? `正在连接底盘 CAN 网关 ${CHASSIS_GATEWAY.url}…` : "等待连接车辆" };
    else if (driveMode === "real" && chassisFresh) {
      const c = sim.chassisStatus;
      loc = { state: "ok", detail: `云乐底盘在线 · ${chassisH.hz} Hz · 模式 ${c?.ccu_drive_mode ?? "?"} · ${Number(c?.ccu_vehicle_speed || 0).toFixed(1)} km/h` };
    } else if (driveMode === "real") {
      const gapS = chassisH?.ageMs != null ? Math.round(chassisH.ageMs / 1000) : null;
      loc = gapS != null
        ? { state: bootTimedOut ? "fail" : "warn", detail: `底盘反馈中断 ${gapS} 秒（之前收到过）：检查 chassis_driver 是否在运行、CAN 是否掉线` }
        : bootTimedOut
          ? { state: "fail", detail: "从未收到云乐底盘反馈，请检查 chassis_driver 与 CAN" }
          : { state: "wait", detail: "等待 /yunle_chassis/feedback/ccu_status…" };
    } else if (locT.level === "ok" || locT.level === "slow") {
      if (locInitState === 1) loc = { state: "warn", detail: driveMode === "real" ? "未初始化，进入后点击「请求定位服务」" : "未设置起点，进入后点击「设起点」" };
      else if (locInitState === 2) loc = { state: "wait", detail: "定位初始化中…" };
      else loc = { state: "ok", detail: `定位正常 · ${locT.hz.toFixed(0)} Hz` };
    } else loc = bootTimedOut ? { state: "fail", detail: "无定位数据，请确认 Autoware 已启动" } : { state: "wait", detail: "等待定位数据…" };

    const camName = camChannel.current.name;
    const cam = camStatus === "playing"
      ? { state: "ok", detail: `${camName} · ${camStats?.kbps != null ? `${(camStats.kbps / 1000).toFixed(1)} Mbps` : "已出画面"}` }
      : bootTimedOut
        ? { state: "fail", detail: `${camName}无画面（推流未启动或网络不通）`,
            action: otherCamChannel ? { label: `改用${otherCamChannel.name}`, onClick: () => { camChannel.select(otherCamChannel.name); restartBoot(); } } : null }
        : { state: "wait", detail: `正在连接${camName}相机…` };

    const parts = [];
    const missing = [];
    if (assets.map) parts.push(`车道 ${assets.map.lanes ?? "?"} 条`); else missing.push("车道地图");
    if (assets.pcd) parts.push(`点云 ${assets.pcd.points ? `${(assets.pcd.points / 10000).toFixed(0)} 万点` : "已加载"}`); else missing.push("点云");
    if (!assets.model) missing.push("车辆模型");
    if (assets.tiles) parts.push("卫星图已加载"); else missing.push("卫星图");
    const map = !missing.length
      ? { state: "ok", detail: parts.join(" · ") }
      : bootTimedOut
        ? { state: "fail", detail: `${missing.join("、")}加载失败${missing.includes("卫星图") ? "（卫星图需外网）" : ""}` }
        : { state: "wait", detail: `正在加载 ${missing.join("、")}…` };

    return [
      { label: "连接车辆", ...vehicle },
      { label: driveMode === "real" ? "底盘反馈" : "车辆定位", ...loc },
      { label: "相机画面", ...cam },
      { label: "地图数据", ...map },
    ];
  })();
  const bootAllDone = bootItems.every((i) => i.state === "ok" || i.state === "warn");
  const bootAutoEnterIn = Math.max(0, Math.ceil((BOOT_MAX_MS + BOOT_AUTO_ENTER_S * 1000 - bootElapsed) / 1000));
  useEffect(() => {
    if (bootPhase !== "show") return;
    if (bootAllDone && bootElapsed >= BOOT_MIN_MS) setBootPhase("leaving");
    else if (bootTimedOut && bootAutoEnterIn <= 0) setBootPhase("leaving");
  }, [bootPhase, bootAllDone, bootElapsed, bootTimedOut, bootAutoEnterIn]);
  useEffect(() => {
    if (bootPhase !== "leaving") return;
    const t = setTimeout(() => setBootPhase("gone"), 900);
    return () => clearTimeout(t);
  }, [bootPhase]);

  // ── 键盘遥控 ──
  const [teleop, setTeleop] = useState(false);
  const teleKeysRef = useRef(TELE_IDLE);
  const publishCmdVelRef = useRef(publishCmdVel);
  publishCmdVelRef.current = publishCmdVel;
  const releaseControlRef = useRef(directCan.releaseControl);
  releaseControlRef.current = directCan.releaseControl;

  useEffect(() => {
    if (teleop && CHASSIS_GATEWAY.enabled && directCan.manualError) setTeleop(false);
  }, [teleop, directCan.manualError]);

  useEffect(() => {
    if (!teleop) return;
    const KEYMAP = { arrowup: "up", w: "up", arrowdown: "down", s: "down", arrowleft: "left", a: "left", arrowright: "right", d: "right" };
    const setKeys = (next) => { teleKeysRef.current = next; };
    const isTyping = (e) => {
      const t = e.target;
      return t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.isContentEditable);
    };
    const onDown = (e) => {
      if (isTyping(e)) return;
      const k = e.key.toLowerCase();
      if (k === "escape") { setTeleop(false); return; }
      if (k === " ") {
        e.preventDefault();
        setKeys({ ...TELE_IDLE, brake: true });
        publishCmdVelRef.current(0, 0, true);
        return;
      }
      const dir = KEYMAP[k];
      if (!dir) return;
      e.preventDefault();
      const cur = teleKeysRef.current;
      if (!cur[dir] || cur.brake) setKeys({ ...cur, brake: false, [dir]: true });
    };
    const onUp = (e) => {
      const k = e.key.toLowerCase();
      if (k === " ") { setKeys({ ...teleKeysRef.current, brake: false }); return; }
      const dir = KEYMAP[k];
      if (dir) setKeys({ ...teleKeysRef.current, [dir]: false });
    };
    const onBlur = () => setKeys(TELE_IDLE); // 窗口失焦立即松开所有按键，避免“卡键”
    const timer = setInterval(() => {
      const k = teleKeysRef.current;
      const lin = k.brake ? 0 : ((k.up ? 1 : 0) - (k.down ? 1 : 0)) * TELEOP.speed;
      const ang = k.brake ? 0 : ((k.left ? 1 : 0) - (k.right ? 1 : 0)) * TELEOP.steering;
      // 激活期间始终以 20 Hz 发布 Autoware Control + GearCommand。
      // 云乐桥接收到后生成 ScuControlCommand，驱动再以 10 ms 周期发送 0x121。
      // 无运动键时持续发送零速；空格额外请求切 N。
      publishCmdVelRef.current(lin, ang, k.brake);
    }, TELEOP.periodMs);
    window.addEventListener("keydown", onDown);
    window.addEventListener("keyup", onUp);
    window.addEventListener("blur", onBlur);
    return () => {
      clearInterval(timer);
      window.removeEventListener("keydown", onDown);
      window.removeEventListener("keyup", onUp);
      window.removeEventListener("blur", onBlur);
      publishCmdVelRef.current(0, 0, true);
      if (CHASSIS_GATEWAY.enabled) releaseControlRef.current();
      teleKeysRef.current = TELE_IDLE;
    };
  }, [teleop]);

  // 自动驾驶被启动时自动退出遥控
  const prevAutoRef = useRef(isAutonomous);
  useEffect(() => {
    if (isAutonomous && !prevAutoRef.current) setTeleop(false);
    prevAutoRef.current = isAutonomous;
  }, [isAutonomous]);

  const toggleTeleop = useCallback(() => {
    if (teleop) {
      publishCmdVel(0, 0, true);
      if (CHASSIS_GATEWAY.enabled) directCan.releaseControl();
      setTeleop(false);
      return;
    }
    if (!requestRemoteMode()) return;
    setPoseMode(null);
    setTeleop(true);
  }, [teleop, publishCmdVel, requestRemoteMode, directCan.releaseControl]);

  const startAutonomous = useCallback(() => {
    // 手动控制中先停车退出；再次点击时才请求 Autoware 自动驾驶，避免两个控制源重叠。
    if (teleop) {
      publishCmdVel(0, 0, true);
      setTeleop(false);
      return false;
    }
    return requestAutonomousMode();
  }, [teleop, publishCmdVel, requestAutonomousMode, sim]);

  // 设置起/终点时：退出遥控；巡航镜头下临时切到俯瞰，方便点选，完成后恢复
  const viewBeforePoseRef = useRef(null);
  useEffect(() => {
    if (poseMode) {
      setTeleop(false);
      if (viewMode === "cinema") { viewBeforePoseRef.current = viewMode; setViewMode("top"); }
    } else if (viewBeforePoseRef.current) {
      setViewMode(viewBeforePoseRef.current);
      viewBeforePoseRef.current = null;
    }
  }, [poseMode]); // eslint-disable-line react-hooks/exhaustive-deps

  // ── 本次任务统计：里程（过滤静止抖动与跳变）、已行驶轨迹、自动驾驶时长 ──
  const odoRef = useRef({ last: null, dist: 0, trailLast: null });
  const [odometer, setOdometer] = useState(0);
  const [trail, setTrail] = useState([]);
  const [autoSeconds, setAutoSeconds] = useState(0);

  useEffect(() => {
    // 设置新起点或新终点视为新任务：清零统计
    odoRef.current = { last: null, dist: 0, trailLast: null };
    setOdometer(0);
    setTrail([]);
    setAutoSeconds(0);
  }, [startPose, goalPose]);

  useEffect(() => {
    if (!Number.isFinite(ego?.x) || !Number.isFinite(ego?.z)) return;
    const r = odoRef.current;
    const cur = { x: ego.x, z: ego.z };
    if (!r.last) {
      r.last = cur;
    } else {
      const d = Math.hypot(cur.x - r.last.x, cur.z - r.last.z);
      if (d >= 3) {
        r.last = cur; // 设起点/重定位造成的跳变不计入里程
      } else if (d >= 0.05) {
        r.dist += d;
        r.last = cur;
        setOdometer(r.dist);
      }
    }
    if (Number.isFinite(ego?.lat) && Number.isFinite(ego?.lng)) {
      if (!r.trailLast || Math.hypot(cur.x - r.trailLast.x, cur.z - r.trailLast.z) >= 0.5) {
        const jump = r.trailLast && Math.hypot(cur.x - r.trailLast.x, cur.z - r.trailLast.z) >= 5;
        r.trailLast = cur;
        setTrail((t) => (jump ? [{ lat: ego.lat, lng: ego.lng }] : [...t.slice(-2999), { lat: ego.lat, lng: ego.lng }]));
      }
    }
  }, [ego?.x, ego?.z, ego?.lat, ego?.lng]);

  const isAutonomousRef = useRef(false);
  isAutonomousRef.current = isAutonomous;
  useEffect(() => {
    const id = setInterval(() => {
      if (isAutonomousRef.current) setAutoSeconds((s) => s + 1);
    }, 1000);
    return () => clearInterval(id);
  }, []);

  // 距目的地：优先沿规划路线计算剩余长度，没有路线时用直线距离
  const distToGoal = useMemo(() => {
    if (!goalPoint) return null;
    const egoXY = latLonToCartesian(ego.lat, ego.lng, MAP_ORIGIN.lat, MAP_ORIGIN.lng);
    const goalXY = latLonToCartesian(goalPoint.lat, goalPoint.lng, MAP_ORIGIN.lat, MAP_ORIGIN.lng);
    const straight = Math.hypot(goalXY.x - egoXY.x, goalXY.y - egoXY.y);
    if (!mapPath || mapPath.length < 2) return straight;
    const pts = mapPath.map((p) => latLonToCartesian(p.lat, p.lng, MAP_ORIGIN.lat, MAP_ORIGIN.lng));
    let best = 0;
    let bestD = Infinity;
    pts.forEach((p, i) => {
      const d = Math.hypot(p.x - egoXY.x, p.y - egoXY.y);
      if (d < bestD) { bestD = d; best = i; }
    });
    let remain = bestD;
    for (let i = best + 1; i < pts.length; i++) remain += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    const last = pts[pts.length - 1];
    remain += Math.hypot(goalXY.x - last.x, goalXY.y - last.y);
    return Math.max(remain, straight);
  }, [goalPoint, mapPath, ego.lat, ego.lng]);

  // ── 状态播报 ──
  // 路线状态以 Autoware 为准（/api/routing/state），收不到时再按终点距离和路线折线推断
  const routeState = sim.routeState;
  const arrived = routeState === 3 || (goalPoint && distToGoal != null && distToGoal < 1.5 && (ego.speed || 0) < 0.1 && odometer > 1);
  const hasRoute = routeState === 2 || routeState === 4 || (globalPath && globalPath.length > 1);
  // 播报条：只显示一句结论；需要用户处理时才加一行简短说明
  const stopStatus = sim.stopStatus;
  const routeClear = sim.routeClear;
  const recent = (st, ms = 6000) => st && Date.now() - st.at < ms;
  let banner;
  if (teleop) banner = {
    tone: "warn",
    title: "手动遥控底盘",
    sub: "↑/W 前进 · ↓/S 后退 · ←/A 左转 · →/D 右转 · 空格停车 · Esc 退出",
  };
  else if (manualError) banner = { tone: "danger", title: "无法进入底盘手动控制", sub: manualError };
  else if (!connected && sim.connectionError) banner = { tone: "danger", title: "车辆连接失败", sub: sim.connectionError };
  else if (!connected) banner = { tone: "warn", title: "正在连接车辆…" };
  else if (stopStatus?.state === "pending") banner = { tone: "warn", title: "正在停车…" };
  else if (stopStatus?.state === "failed" && recent(stopStatus, 10000)) banner = { tone: "danger", title: "停车指令失败", sub: stopStatus.msg };
  else if (obstacleAhead != null && (isAutonomous || hasRoute)) banner = { tone: "danger", title: `前方有障碍物 · ${obstacleAhead.toFixed(1)} m` };
  else if (startStatus?.kind === "gnss" && (startStatus.state === "pending" || startStatus.state === "verifying")) banner = { tone: "active", title: "定位初始化中…" };
  else if (startStatus?.kind === "gnss" && startStatus.state === "ok") banner = { tone: "ok", title: "定位初始化完成" };
  else if (startStatus?.kind === "gnss" && startStatus.state === "failed") banner = { tone: "warn", title: "定位初始化失败", sub: startStatus.msg };
  else if (startStatus?.state === "pending") banner = { tone: "active", title: "正在设置起点…" };
  else if (startStatus?.state === "failed") banner = { tone: "warn", title: "起点设置失败", sub: startStatus.msg };
  else if (startStatus?.state === "ok" || startStatus?.state === "sent") banner = { tone: "ok", title: "起点已设置", sub: goalPoint ? undefined : "请设置目的地" };
  else if (routeClear?.state === "failed" && recent(routeClear, 10000)) banner = { tone: "warn", title: "取消路线失败", sub: routeClear.msg };
  else if (routeClear?.state === "ok" && recent(routeClear, 3000)) banner = { tone: "ok", title: "路线已取消" };
  else if (arrived) banner = { tone: "ok", title: "已到达目的地" };
  else if (!isAutonomous && autoFail && Date.now() - autoFail.at < 10000) banner = { tone: "warn", title: "无法启动自动驾驶", sub: autoFail.msg };
  else if (autoTransitioning) banner = { tone: "warn", title: "正在启动自动驾驶…" };
  else if (isAutonomous) banner = { tone: "ok", title: "自动驾驶中" };
  else if (stopStatus?.state === "ok" && recent(stopStatus, 4000)) banner = { tone: "ok", title: "已停车" };
  else if (hasRoute) banner = { tone: "active", title: "路线已规划" };
  else if (goalPoint) banner = { tone: "active", title: "正在规划路线…" };
  else banner = null; // 空闲（无路线、无事件）时不显示播报条，避免回放或待机时一直挂着“车辆待命中”

  return (
    <div className="dt-root" style={{ width: "100%", height: "100vh", position: "relative", overflow: "hidden" }}>
      <style>{GLOBAL_CSS}</style>

      <header className="dt-header">
        <div className="dt-brand">
          <BrandLogo />
          <div>
            <div className="dt-brand-name">南京核星力</div>
            <div className="dt-brand-sub">孵化园数字孪生监控系统</div>
          </div>
        </div>

        <div className="dt-chips">
          <VehicleChannelMenu
            channels={VEHICLE_CHANNELS}
            currentUrl={appliedIp}
            connected={connected}
            connectingLong={connectingLong}
            activeUrl={sim.activeUrl}
            liveKbps={sim.rxKbps}
            liveMs={sim.connectionMs}
            quality={linkQuality({ connected, connectingLong })}
            jitterMs={linkJitterMs}
            results={vehicleProbe.results}
            probing={vehicleProbe.probing}
            onSelect={selectVehicleChannel}
            onProbe={vehicleProbe.probeOne}
            onCustom={selectVehicleChannel}
          />
          <SystemMenu
            health={topicHealth}
            udpDiagnostics={sim.udpDiagnostics}
            onResetUdpDiagnostics={sim.resetUdpDiagnostics}
            connected={connected}
            ego={ego}
            driveMode={driveMode}
            gnss={gnss}
            operationMode={operationMode}
            autoAvailable={autoAvailable}
            isAutonomous={isAutonomous}
            locInitState={locInitState}
            objectCount={objFresh ? (sim.objects || []).length : 0}
            hasGoal={Boolean(goalPoint) || hasRoute}
            controlCmd={controlCmd}
          />
        </div>

        <div className="dt-header-right">
          {NETLOG_ENABLED && (
            <button
              className="dt-report-btn"
              title="下载网络时序记录（CSV），与同时段 bag 一起用于离线测试"
              onClick={() => netLog.download()}
            >
              <svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round"><path d="M12 4v11M7 10l5 5 5-5M4 20h16" /></svg>
              <span>下载网络记录 <span className="dt-num">{netLog.rows.length}</span></span>
            </button>
          )}
          <button
            className="dt-report-btn"
            title="在新标签页打开巡检运营报表（监控界面保持运行）"
            onClick={() => {
              const url = `${window.location.pathname}${window.location.search}#/report`;
              const w = window.open(url, "_blank");
              if (!w) window.location.hash = "#/report"; // 浏览器拦截新窗口时，在当前页打开
            }}
          >
            <svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round"><path d="M4 20V10M10 20V4M16 20v-7M22 20H2" /></svg>
            <span>运营报表</span>
          </button>
          <div className="dt-clock dt-num">
            <b>{clock.toLocaleTimeString("zh-CN", { hour12: false, timeZone: "Asia/Shanghai" })}</b>
            <span>{clock.toLocaleDateString("zh-CN", { month: "2-digit", day: "2-digit", weekday: "short", timeZone: "Asia/Shanghai" })} 北京时间</span>
          </div>
        </div>
      </header>

      <div style={{ position: "absolute", top: 52, bottom: 0, left: 0, right: 0, display: "flex" }}>

        <div style={{ width: "60%", minWidth: 0, height: "100%", display: "flex", flexDirection: "column", borderRight: "1px solid var(--line)", overflow: "hidden" }}>
          <PanelHeader
            title="智能驾驶实况"
            live={connected}
            right={
              <ActionBar
                connected={connected}
                teleopConnected={teleopConnected}
                driveMode={driveMode}
                setDriveMode={setDriveMode}
                onGnssInit={onGnssInit}
                gnssBusy={gnssBusy}
                poseMode={poseMode}
                setPoseMode={setPoseMode}
                teleopActive={teleop}
                onToggleTeleop={toggleTeleop}
                isAutonomous={isAutonomous}
                autoAvailable={autoAvailable}
                autoTransitioning={autoTransitioning}
                autoRequestStatus={autoRequestStatus}
                requestAutonomousMode={startAutonomous}
                obstacleCount={(sim.placedObstacles || []).length}
                hasGoal={Boolean(goalPoint) || hasRoute || arrived}
                onClearRoute={sim.clearRoute}
                routeClearing={routeClear?.state === "pending"}
                onStop={sim.requestStop}
                stopping={stopStatus?.state === "pending"}
              />
            }
          />

          <div style={{ flex: 1, position: "relative" }}>
            <Scene
              viewMode={viewMode}
              pointSize={pointSize}
              pcdConfig={pcdConfig}
              carOffset={carOffset}
              currentCar={currentCar}
              sim={sim}
              poseMode={poseMode}
              setPoseMode={setPoseMode}
              publishPose={publishPose}
              teleopActive={teleop}
              onAssetStatus={onAssetStatus}
              onObstacleAlert={setObstacleAhead}
              onPlaceObstacle={placeObstacle}
              onRemoveObstacle={removeObstacle}
              intro={bootPhase === "show"}
              routePoints={globalPath3d && globalPath3d.length > 1 ? globalPath3d : null}
            />

            {banner && <StatusBanner tone={banner.tone} title={banner.title} sub={banner.sub} />}

            {poseMode === "obstacle" && (
              <ObstacleToolbar
                preset={obstaclePreset}
                setPreset={setObstaclePreset}
                count={(sim.placedObstacles || []).length}
                online={sim.obstacleNodeOnline}
                connected={connected}
                onClear={sim.clearObstacles}
                onDone={() => setPoseMode(null)}
              />
            )}

            <div className="dt-views dt-glass">
              {["cinema", "follow", "top", "free"].map((v) => (
                <button key={v} className={viewMode === v ? "on" : ""} onClick={() => setViewMode(v)}>
                  {{ cinema: "巡航", follow: "跟随", top: "俯瞰", free: "自由" }[v]}
                </button>
              ))}
            </div>

            <DriveDashboard
              speed={ego.speed}
              steering={ego.steering}
              stale={CHASSIS_GATEWAY.enabled ? !directCcuFresh : !connected}
              battery={sim.bmsStatus}
              batteryStale={(CHASSIS_GATEWAY.enabled ? !directCan.connected : !connected) || !sim.bmsFeedbackAt || clock.getTime() - sim.bmsFeedbackAt > BMS_STALE_MS}
              odometer={odometer}
              autoSeconds={autoSeconds}
              distToGoal={distToGoal}
              hasGoal={!!goalPoint}
            />

          </div>
        </div>

        <div style={{ width: "40%", minWidth: 0, height: "100%", display: "flex", flexDirection: "column", overflow: "hidden" }}>

          <div style={{ width: "100%", height: "50%", position: "relative", borderBottom: "1px solid var(--line)", display: "flex", flexDirection: "column" }}>
            <PanelHeader title="园区实景地图" live={connected} right={<MapLegend />} />
            <div style={{ flex: 1, position: "relative" }}>
              <TiandituMap ego={ego} startPoint={startPoint} goalPoint={goalPoint} globalPath={mapPath} trail={trail} onLoaded={() => onAssetStatus("tiles")} />
            </div>
          </div>

          <div style={{ width: "100%", height: "50%", position: "relative", background: "#050910", display: "flex", flexDirection: "column" }}>
            <PanelHeader
              title="相机实时画面"
              live={camStatus === "playing"}
              right={
                <div className="dt-cam-head">
                  {camFit === "cover" && Math.round(camFocusY) !== CAMERA_FOCUS_Y && (
                    <button type="button" className="dt-action" onClick={resetCamFocus} title="显示位置恢复默认">
                      <svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round"><path d="M4 12a8 8 0 1 0 2.4-5.7M4 4v4h4" /></svg>
                      <span>复位</span>
                    </button>
                  )}
                  <div className="dt-seg dt-mode-seg" title="铺满：填满画面，上下有裁剪，可按住画面上下拖动调整；完整：显示整幅画面，两边留黑">
                    <button className={camFit === "cover" ? "on" : ""} onClick={() => setCamFit("cover")}>铺满</button>
                    <button className={camFit === "contain" ? "on" : ""} onClick={() => setCamFit("contain")}>完整</button>
                  </div>
                  <button type="button" className="dt-action" onClick={() => setCamFull(true)} title="全屏（也可双击画面），Esc 退出">
                    <svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round"><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5" /></svg>
                    <span>全屏</span>
                  </button>
                  {cameraChannels.length > 1 && camChannelMenu}
                </div>
              }
            />

            <div style={{ flex: 1, position: "relative", overflow: "hidden", background: "#050910" }}>
              <div
                className={`dt-cam-box ${camFull ? "full" : ""} ${camFit === "cover" ? "draggable" : ""}`}
                title={camFit === "cover" ? "按住上下拖动可调整显示位置 · 双击全屏" : "双击全屏"}
                onDoubleClick={() => setCamFull((v) => !v)}
                onPointerDown={onCamPointerDown}
                onPointerMove={onCamPointerMove}
                onPointerUp={onCamPointerUp}
                onPointerCancel={onCamPointerUp}
              >
                <WebRtcCamera baseUrl={cameraUrl} active={cameraActive} onStatus={setCamStatus} onStats={setCamStats} fit={camFit} focusY={camFocusY} />
                {camFull && (
                  <div className="dt-cam-channel" onDoubleClick={(e) => e.stopPropagation()}>
                    <button type="button" className="dt-cam-tool" onClick={() => setCamFull(false)} title="退出全屏（Esc）">退出全屏</button>
                    {cameraChannels.length > 1 && camChannelMenu}
                  </div>
                )}
                {camFull && <div className="dt-cam-fullhint">双击画面或按 Esc 退出全屏</div>}
              </div>
            </div>
          </div>

        </div>
      </div>

      {bootPhase !== "gone" && (
        <BootOverlay
          items={bootItems}
          leaving={bootPhase === "leaving"}
          timedOut={bootTimedOut}
          autoEnterIn={bootAutoEnterIn}
          onEnter={() => setBootPhase("leaving")}
        />
      )}
    </div>
  );
}
