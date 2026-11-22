"""
无人车业务 MCP server（只暴露业务工具，不开放任意话题/服务）

原理：
  大模型客户端（Claude Desktop / 上位机 Electron 主进程等） ⇄ MCP(stdio) ⇄ 本程序 ⇄ rosbridge WebSocket ⇄ 工控机 Autoware

  - 本程序对大模型只声明 10 个业务工具（其中 3 个只读），模型只能“选工具 + 填参数”；
  - 每个工具在本程序内部做：参数校验 → 前置条件检查（读缓存的车辆状态）→ 需要时请求人工确认 → 下发 → 等待结果 → 写审计日志；
  - 前置条件不满足时返回结构化的 {refused, code, reason, issues, hint, vehicle_state}，
    让模型能定位“哪一步、为什么”并给出处置建议，而不是只看到一句中文；
  - rosbridge 报文（话题名、服务名、消息类型）全部写死在代码里，模型碰不到。

运行：python server.py --config config.json
"""


import argparse
import asyncio
import itertools
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any, Literal, Optional

import websockets
from pydantic import BaseModel, Field

# 兼容 mcp 1.x（FastMCP）与 2.x（MCPServer）
try:
    from mcp.server.mcpserver import Context, MCPServer
except ImportError:  # mcp < 2
    from mcp.server.fastmcp import Context, FastMCP as MCPServer
from mcp.types import ToolAnnotations

# stdio 模式下 stdout 是协议通道，日志只能写 stderr
logging.basicConfig(stream=sys.stderr, level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("ros_agent_mcp")

# ─────────────────────────── Autoware AD API 常量（与上位机 App.jsx 保持一致） ───────────────────────────
OP_MODE_TOPIC = ("/api/operation_mode/state", "autoware_adapi_v1_msgs/msg/OperationModeState")
LOC_INIT_TOPIC = ("/api/localization/initialization_state", "autoware_adapi_v1_msgs/msg/LocalizationInitializationState")
ROUTE_STATE_TOPIC = ("/api/routing/state", "autoware_adapi_v1_msgs/msg/RouteState")
KINEMATIC_TOPIC = ("/localization/kinematic_state", "nav_msgs/msg/Odometry")
GOAL_TOPIC = ("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped")

SRV_INIT_LOC = ("/api/localization/initialize", "autoware_adapi_v1_msgs/srv/InitializeLocalization")
SRV_TO_AUTO = ("/api/operation_mode/change_to_autonomous", "autoware_adapi_v1_msgs/srv/ChangeOperationMode")
SRV_TO_STOP = ("/api/operation_mode/change_to_stop", "autoware_adapi_v1_msgs/srv/ChangeOperationMode")
SRV_CLEAR_ROUTE = ("/api/routing/clear_route", "autoware_adapi_v1_msgs/srv/ClearRoute")

OP_MODE_NAME = {0: "未知", 1: "停车(STOP)", 2: "自动驾驶", 3: "本地手动", 4: "远程"}
LOC_STATE_NAME = {0: "未知", 1: "未初始化", 2: "初始化中", 3: "已初始化"}
ROUTE_STATE_NAME = {0: "未知", 1: "无路线", 2: "已设置路线", 3: "已到达", 4: "路线变更中"}
CHASSIS_BRAKE_FIELDS = (  # 与 App.jsx publishCmdVel 中的安全联锁字段一致
    "ccu_parking_status",
    "emergency_brake_request_status",
    "remote_brake_request_status",
    "touch_brake_request_status",
    "handle_brake_request_status",
)

# /diagnostics 是 Autoware 各模块自己报告的健康信息（diagnostic_msgs/DiagnosticArray），
# 比翻日志精准得多：每条带级别、模块名、原因和键值对。
DIAG_TOPIC = ("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray")
DIAG_LEVEL_NAME = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
# 严重程度排序：ERROR > STALE（数据缺失）> WARN > OK
DIAG_LEVEL_RANK = {2: 0, 3: 1, 1: 2, 0: 3}


class ToolRefused(Exception):
    """前置条件或校验不通过：把原因结构化返回给模型，由模型解释给用户并给出处置建议。

    code   稳定错误码，模型据此映射到处置动作，不必解析中文措辞；
    issues 一次列全不满足的条件（例如同时缺定位和路线），避免模型只看到第一条就下结论。
    """

    def __init__(self, reason: str, code: str = "REFUSED", issues: Optional[list[str]] = None):
        super().__init__(reason)
        self.code = code
        self.issues = list(issues or [])


# 错误码 → 处置提示。写在服务端而不是提示词里，保证模型每次拿到的建议一致、可核对。
# 这里只描述“该去查什么”，不描述“自动去改什么”：修复动作仍须由操作员发起。
CODE_HINTS = {
    "ROSBRIDGE_DOWN": "检查工控机是否开机、rosbridge 是否在运行、上位机与工控机网络是否连通（默认 ws://工控机IP:9090）。",
    "SERVICE_TIMEOUT": "工控机未在限定时间内回应服务调用：确认对应 Autoware 节点仍在运行、网络无丢包；不要连续重试写操作。",
    "STATE_STALE": "关键状态话题超过 3 秒没有更新：确认 rosbridge 订阅正常、Autoware 未重启，可先在界面执行“查询车辆状态”。",
    "LOC_NOT_READY": "定位未初始化或不稳定：在车辆实际位置执行“设起点”，或检查 GNSS/NDT 是否收敛后重试。",
    "ROUTE_MISSING": "当前没有可用路线：先设置目的地，并等待路线状态变为“已设置路线”。",
    "ROUTE_CLEAR_FAILED": "旧路线没有清除成功：确认 routing 节点正常，路线状态是否停在“路线变更中”。",
    "PLAN_TIMEOUT": "mission_planner 未在限定时间内生成路线：检查目标点是否在地图范围内、地图与路线规划节点是否正常。",
    "AUTONOMY_NOT_READY": "Autoware 未开放自动驾驶切换：按 issues 逐条排查（定位可信度、车辆是否停稳、是否存在紧急或制动状态）。",
    "CHASSIS_BRAKE": "底盘仍有制动请求：确认急停、远程/触摸/手柄制动均已释放，再重新发起（驻车状态不参与联锁）。",
    "IN_TRANSITION": "运行模式正在切换：等待几秒后重新查询状态，不要重复下发切换指令。",
    "ALREADY_AUTONOMOUS": "车辆正在自动驾驶：如需改动作业，先平稳停车再执行该操作。",
    "VEHICLE_MOVING": "车辆尚未停稳：先停车并确认车速接近 0，再进行定位类操作。",
    "RATE_LIMITED": "上一条指令刚下发：等待约 1 秒后重试。",
    "STATE_CHANGED": "等待人工确认期间车辆状态发生变化：重新查询状态后重新发起整个流程。",
    "MODE_SWITCH_FAILED": "切换自动驾驶服务返回成功但模式未变化：检查运行模式节点与紧急状态，不要反复强制切换。",
    "CONFIRM_DECLINED": "操作员未确认、主动取消或确认超时：确认现场安全后重新发起；这属于正常拦截，不是系统故障。",
    "CONFIRM_UNSUPPORTED": "当前客户端没有人工确认通道：需在支持 elicitation 的上位机中执行该操作。",
    "SERVICE_FAILED": "工控机返回失败：查看返回的 message，并结合 get_diagnostics 中对应模块的状态。",
    "UNKNOWN_STATION": "站点不在站点表中：先调用 list_stations 核对，不要自行编造坐标或站点名。",
}
DEFAULT_HINT = "先调用 get_vehicle_status 查看当前状态；需要更细的模块健康状况时调用 get_diagnostics。"


# ─────────────────────────── 配置 ───────────────────────────
def load_config(path: str) -> dict:
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    cfg.setdefault("rosbridge_url", "ws://127.0.0.1:9090")
    cfg.setdefault("audit_log", "audit.jsonl")
    cfg.setdefault("stale_seconds", 3.0)
    cfg.setdefault("service_timeout_seconds", 8.0)
    cfg.setdefault("min_command_interval_seconds", 1.0)
    cfg.setdefault("route_wait_seconds", 15.0)
    cfg.setdefault("autonomous_ready_wait_seconds", 12.0)
    cfg.setdefault("autonomous_confirm_wait_seconds", 8.0)
    cfg.setdefault("chassis_feedback_topic", None)  # 例如 "/yunle_chassis/feedback/ccu_status"；走 CAN 直连网关时填 null
    cfg.setdefault("chassis_feedback_type", "chassis_interfaces/msg/CcuStatus")
    # 只有列在这里的底盘字段才算“制动请求”。驻车状态会在自动驾驶中途停车时自然置位，
    # 不属于外部制动请求，默认不参与联锁；急停/远程/触摸/手柄仍会拦截。
    cfg.setdefault("brake_interlock_fields", [f for f in CHASSIS_BRAKE_FIELDS if f != "ccu_parking_status"])
    interlock = cfg["brake_interlock_fields"]
    if not isinstance(interlock, list) or any(f not in CHASSIS_BRAKE_FIELDS for f in interlock):
        raise SystemExit("brake_interlock_fields 只能包含 CHASSIS_BRAKE_FIELDS 中的字段名")
    cfg["brake_interlock_fields"] = list(dict.fromkeys(interlock))
    cfg.setdefault("map_bounds", None)  # 可选 {"x_min":..,"x_max":..,"y_min":..,"y_max":..}

    stations = cfg.get("stations")
    if not isinstance(stations, list) or not stations:
        raise SystemExit("config.json 缺少 stations（至少一个站点）")
    seen = set()
    bounds = cfg["map_bounds"]
    for s in stations:
        sid = s.get("id")
        if not isinstance(sid, str) or not sid.strip():
            raise SystemExit(f"站点 id 无效：{s}")
        if sid in seen:
            raise SystemExit(f"站点 id 重复：{sid}")
        seen.add(sid)
        for k in ("x", "y", "yaw"):
            if not isinstance(s.get(k), (int, float)) or not math.isfinite(s[k]):
                raise SystemExit(f"站点 {sid} 的 {k} 必须是有限数字")
        if not -math.pi <= s["yaw"] <= math.pi:
            raise SystemExit(f"站点 {sid} 的 yaw 必须在 ±π 之间（弧度）")
        if bounds and not (bounds["x_min"] <= s["x"] <= bounds["x_max"] and bounds["y_min"] <= s["y"] <= bounds["y_max"]):
            raise SystemExit(f"站点 {sid} 超出 map_bounds")
        s.setdefault("name", sid)
        s.setdefault("aliases", [])
    return cfg


# ─────────────────────────── rosbridge 客户端 ───────────────────────────
class RosBridge:
    """单条 WebSocket：后台任务收消息，缓存订阅话题的最新值；服务调用按 id 等待 service_response。"""

    def __init__(self, url: str, subscriptions: list[tuple[str, str, int]]):
        self.url = url
        self.subscriptions = subscriptions  # (topic, type, throttle_ms)
        self.ws = None
        self.latest: dict[str, tuple[float, dict]] = {}  # topic -> (接收时间, msg)
        self.pending: dict[str, asyncio.Future] = {}
        self.ids = itertools.count(1)
        self.reader: Optional[asyncio.Task] = None
        self.lock = asyncio.Lock()
        self.advertised: set[str] = set()

    @property
    def connected(self) -> bool:
        return self.ws is not None and self.reader is not None and not self.reader.done()

    async def ensure_connected(self) -> None:
        async with self.lock:
            if self.connected:
                return
            try:
                self.ws = await asyncio.wait_for(websockets.connect(self.url, max_size=None), timeout=5)
            except Exception as e:  # noqa: BLE001
                self.ws = None
                raise ToolRefused(f"无法连接工控机 rosbridge（{self.url}）：{e}", code="ROSBRIDGE_DOWN") from e
            self.advertised.clear()
            # 断线重连后不能继续沿用旧连接的状态缓存；同一连接内的 AD API 状态则允许长期缓存。
            self.latest.clear()
            self.reader = asyncio.create_task(self._read_loop())
            for topic, typ, throttle in self.subscriptions:
                await self._send({"op": "subscribe", "id": f"mcp_sub_{topic}", "topic": topic, "type": typ,
                                  "throttle_rate": throttle, "queue_length": 1})
            log.info("已连接 rosbridge %s", self.url)
            await asyncio.sleep(1.0)  # 等第一批状态消息到达，避免刚连上就误判“数据过期”

    async def _send(self, obj: dict) -> None:
        await self.ws.send(json.dumps(obj))

    async def _read_loop(self) -> None:
        try:
            async for raw in self.ws:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                op = msg.get("op")
                if op == "publish" and "topic" in msg:
                    self.latest[msg["topic"]] = (time.monotonic(), msg.get("msg") or {})
                elif op == "service_response":
                    fut = self.pending.pop(msg.get("id"), None)
                    if fut and not fut.done():
                        fut.set_result(msg)
        except Exception as e:  # noqa: BLE001
            log.warning("rosbridge 连接中断：%s", e)
        finally:
            for fut in self.pending.values():
                if not fut.done():
                    fut.set_exception(ToolRefused("rosbridge 连接中断", code="ROSBRIDGE_DOWN"))
            self.pending.clear()
            self.ws = None

    def get(self, topic: str, stale_s: Optional[float]) -> Optional[dict]:
        item = self.latest.get(topic)
        if not item or (stale_s is not None and time.monotonic() - item[0] > stale_s):
            return None
        return item[1]

    async def call_service(self, service: str, typ: str, args: dict, timeout: float) -> dict:
        await self.ensure_connected()
        req_id = f"mcp_srv_{next(self.ids)}"
        fut = asyncio.get_running_loop().create_future()
        self.pending[req_id] = fut
        await self._send({"op": "call_service", "id": req_id, "service": service, "type": typ, "args": args})
        try:
            resp = await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError as e:
            self.pending.pop(req_id, None)
            raise ToolRefused(f"工控机在 {timeout:.0f} 秒内没有回复 {service}", code="SERVICE_TIMEOUT") from e
        values = resp.get("values")
        status = values.get("status", {}) if isinstance(values, dict) else {}
        ok = resp.get("result") is not False and status.get("success", True) is not False
        detail = status.get("message") if isinstance(status, dict) else None
        if not ok:
            detail = detail or (values if isinstance(values, str) else "")
            if "does not exist" in str(detail):
                detail = "Autoware 未启动或服务不存在"
        return {"success": ok, "message": detail or ""}

    async def publish(self, topic: str, typ: str, msg: dict) -> None:
        await self.ensure_connected()
        if topic not in self.advertised:
            await self._send({"op": "advertise", "topic": topic, "type": typ})
            self.advertised.add(topic)
        await self._send({"op": "publish", "topic": topic, "type": typ, "msg": msg})


# ─────────────────────────── 构建 MCP server ───────────────────────────
class ConfirmSchema(BaseModel):
    confirm: bool = Field(description="勾选表示操作员已确认现场安全，同意执行")


def build_server(cfg: dict) -> MCPServer:
    stations = {s["id"]: s for s in cfg["stations"]}
    stale = float(cfg["stale_seconds"])
    srv_timeout = float(cfg["service_timeout_seconds"])
    min_interval = float(cfg["min_command_interval_seconds"])
    route_wait = float(cfg["route_wait_seconds"])
    auto_ready_wait = float(cfg["autonomous_ready_wait_seconds"])
    auto_confirm_wait = float(cfg["autonomous_confirm_wait_seconds"])
    audit_path = Path(cfg["audit_log"])

    subs = [
        (*OP_MODE_TOPIC, 0),
        (*LOC_INIT_TOPIC, 0),
        (*ROUTE_STATE_TOPIC, 0),
        (*KINEMATIC_TOPIC, 500),
        (*DIAG_TOPIC, 500),  # 诊断约 1Hz；话题不存在时只是收不到消息，不影响其他订阅
    ]
    if cfg["chassis_feedback_topic"]:
        subs.append((cfg["chassis_feedback_topic"], cfg["chassis_feedback_type"], 200))
    ros = RosBridge(cfg["rosbridge_url"], subs)
    last_command_at = [0.0]

    station_lines = "\n".join(
        f"- {s['id']}：{s['name']}" + (f"（别名：{'、'.join(s['aliases'])}）" if s["aliases"] else "")
        for s in cfg["stations"]
    )
    mcp = MCPServer(
        name="ros-agent",
        instructions=(
            "你是无人车调度助手。只能通过本服务提供的工具操作车辆，不能直接控制速度或转向。\n"
            "用户要求车辆前往某站点并发车时，优先调用 dispatch_to_station；它会清理旧路线、诊断并尽量修复状态、请求人工确认后启动自动驾驶。\n"
            "工具返回 refused 时，把诊断原因用中文解释给用户，不要绕过人工确认或安全联锁。\n"
            "用户说“停”“停车”“急停”时立即调用 stop_vehicle。\n"
            f"可用站点：\n{station_lines}"
        ),
    )

    # ── 公共辅助 ──
    def audit(tool: str, args: dict, decision: str, result: Any) -> None:
        rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "tool": tool, "args": args, "decision": decision, "result": result}
        try:
            with audit_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except OSError as e:
            log.error("写审计日志失败：%s", e)

    def snapshot() -> dict:
        # 这三个是“状态”话题，定位/路线通常只在变化时发布；不能按 3 秒频率话题处理。
        # rosbridge 重连时 latest 已清空，因此在当前连接内缓存最后一帧是安全的。
        op = ros.get(OP_MODE_TOPIC[0], None)
        loc = ros.get(LOC_INIT_TOPIC[0], None)
        route = ros.get(ROUTE_STATE_TOPIC[0], None)
        kin = ros.get(KINEMATIC_TOPIC[0], stale)
        chassis = ros.get(cfg["chassis_feedback_topic"], stale) if cfg["chassis_feedback_topic"] else None
        snap: dict[str, Any] = {"connected": ros.connected}
        if op is not None:
            mode = int(op.get("mode", 0))
            snap["_mode"] = mode
            snap["operation_mode"] = OP_MODE_NAME.get(mode, str(mode))
            snap["is_autonomous"] = mode == 2
            snap["autonomous_available"] = bool(op.get("is_autonomous_mode_available", False))
            snap["in_transition"] = bool(op.get("is_in_transition", False))
        if loc is not None:
            snap["localization"] = LOC_STATE_NAME.get(int(loc.get("state", 0)), "未知")
            snap["_loc_state"] = int(loc.get("state", 0))
        if route is not None:
            snap["route"] = ROUTE_STATE_NAME.get(int(route.get("state", 0)), "未知")
            snap["_route_state"] = int(route.get("state", 0))
        if kin is not None:
            p = kin.get("pose", {}).get("pose", {}).get("position", {})
            v = kin.get("twist", {}).get("twist", {}).get("linear", {})
            snap["position_map"] = {"x": round(p.get("x", 0.0), 2), "y": round(p.get("y", 0.0), 2)}
            snap["speed_kmh"] = round(abs(float(v.get("x", 0.0))) * 3.6, 1)
            snap["nearest_station"] = min(
                stations.values(), key=lambda s: math.hypot(s["x"] - p.get("x", 0.0), s["y"] - p.get("y", 0.0))
            )["name"]
        if chassis is not None:
            snap["chassis_brake_active"] = [f for f in CHASSIS_BRAKE_FIELDS if chassis.get(f)]
            # 用于联锁判断：只统计配置白名单里的字段，驻车等停车自然产生的状态不算制动请求。
            snap["_brake_blocking"] = [f for f in cfg["brake_interlock_fields"] if chassis.get(f)]
        return snap

    def public_snap(snap: dict) -> dict:
        """去掉下划线开头的内部字段（_mode/_loc_state/_route_state），那些是给判断逻辑用的。"""
        return {k: v for k, v in snap.items() if not k.startswith("_")}

    def require(cond: bool, reason: str, code: str = "REFUSED", issues: Optional[list[str]] = None) -> None:
        if not cond:
            raise ToolRefused(reason, code=code, issues=issues)

    def require_fresh(snap: dict, *keys: str) -> None:
        missing = [k for k in keys if k not in snap]
        require(
            not missing,
            f"超过 {stale:.0f} 秒没收到车辆状态（{', '.join(missing)}），请检查工控机与网络",
            code="STATE_STALE",
        )

    def rate_limit() -> None:
        now = time.monotonic()
        require(now - last_command_at[0] >= min_interval, "指令过于频繁，请稍后再试", code="RATE_LIMITED")
        last_command_at[0] = now

    async def confirm(ctx: Context, message: str) -> None:
        """通过 MCP elicitation 请求人工确认。客户端不支持或用户未确认 → 拒绝执行（默认拒绝）。"""
        try:
            res = await ctx.elicit(message=message, schema=ConfirmSchema)
        except Exception as e:  # noqa: BLE001  客户端不支持 elicitation
            raise ToolRefused(
                f"该操作需要人工确认，但当前客户端不支持确认弹窗（{e}），已拒绝执行",
                code="CONFIRM_UNSUPPORTED",
            ) from e
        require(
            res.action == "accept" and getattr(res, "data", None) is not None and res.data.confirm,
            "操作员未确认，已取消",
            code="CONFIRM_DECLINED",
        )

    async def run_tool(name: str, args: dict, body, steps: Optional[list[str]] = None) -> dict:
        """统一外壳：连接 → 执行 → 拒绝原因结构化返回 → 审计。

        steps 传入正在执行的工作流步骤列表（如派车），拒绝时一并返回“已经做到哪一步”，
        避免模型把“中途失败”误判为“完全没有执行”。
        """
        try:
            await ros.ensure_connected()
            result = await body()
            audit(name, args, "executed", result)
            return result
        except ToolRefused as e:
            out: dict[str, Any] = {
                "refused": True,
                "code": e.code,
                "reason": str(e),
                "hint": CODE_HINTS.get(e.code, DEFAULT_HINT),
            }
            if e.issues:
                out["issues"] = e.issues
            # 现成的“一项一条”列表，提示词要求模型把这里每条单独占一行输出。
            out["check_items"] = list(e.issues) if e.issues else [str(e)]
            if steps:
                out["completed_steps"] = list(steps)
            try:
                out["vehicle_state"] = public_snap(snapshot())
            except Exception:  # noqa: BLE001  快照异常不能掩盖真正的拒绝原因
                pass
            audit(name, args, "refused", out)
            return out

    def make_pose(x: float, y: float, yaw: float) -> dict:
        now = time.time()
        return {
            "header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)}, "frame_id": "map"},
            "pose": {"position": {"x": x, "y": y, "z": 0.0},
                     "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}},
        }

    def make_initialization_args(x: float, y: float, yaw: float) -> dict:
        pose = make_pose(x, y, yaw)
        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.0685
        return {
            "pose": [{
                "header": pose["header"],
                "pose": {"pose": pose["pose"], "covariance": covariance},
            }]
        }

    async def wait_for(pred, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred(snapshot()):
                return True
            await asyncio.sleep(0.2)
        return False

    async def pace_command() -> None:
        """高层工作流内部按配置节流；与单工具的“过快则拒绝”不同，这里主动等待。"""
        remaining = min_interval - (time.monotonic() - last_command_at[0])
        if remaining > 0:
            await asyncio.sleep(remaining)
        last_command_at[0] = time.monotonic()

    def autonomous_issues(snap: dict, *, require_route: bool = True) -> list[str]:
        issues: list[str] = []
        for key, label in (("operation_mode", "运行模式"), ("localization", "定位"), ("route", "路线")):
            if key not in snap:
                issues.append(f"未收到{label}状态")
        if issues:
            return issues
        if snap.get("is_autonomous"):
            return []
        if snap.get("in_transition"):
            issues.append("车辆正在切换运行模式")
        if snap.get("_loc_state") != 3:
            issues.append(f"定位状态为“{snap.get('localization', '未知')}”")
        if require_route and snap.get("_route_state") != 2:
            issues.append(f"路线状态为“{snap.get('route', '未知')}”")
        if not snap.get("autonomous_available", False):
            issues.append("Autoware 尚未开放自动驾驶切换（常见原因：车辆未停稳、定位不可信或紧急状态）")
        if cfg["chassis_feedback_topic"]:
            if "_brake_blocking" not in snap:
                issues.append("没有收到底盘反馈")
            elif snap["_brake_blocking"]:
                issues.append(f"底盘存在制动请求：{', '.join(snap['_brake_blocking'])}")
        return issues

    StationId = Literal[tuple(stations.keys())]  # 生成 JSON Schema enum：模型只能从站点表里选

    READ = ToolAnnotations(read_only_hint=True, destructive_hint=False, open_world_hint=False)
    WRITE = ToolAnnotations(read_only_hint=False, destructive_hint=False, open_world_hint=True)
    DANGER = ToolAnnotations(read_only_hint=False, destructive_hint=True, open_world_hint=True)

    # ── 只读工具 ──
    @mcp.tool(description="查询车辆当前状态：连接、运行模式、定位、路线、位置、车速、最近站点、底盘制动。", annotations=READ)
    async def get_vehicle_status() -> dict:
        async def body():
            return public_snap(snapshot())
        return await run_tool("get_vehicle_status", {}, body)

    @mcp.tool(description="列出所有可前往的站点（id、名称、别名）。", annotations=READ)
    async def list_stations() -> list[dict]:
        return [{"id": s["id"], "name": s["name"], "aliases": s["aliases"]} for s in cfg["stations"]]

    @mcp.tool(
        description=(
            "查询 Autoware 各模块的健康诊断（订阅 /diagnostics），按级别返回 ERROR/WARN 条目，"
            "同时附带当前车辆状态和自动驾驶联锁问题。只读，用于回答“为什么不能启动自动驾驶”“哪个模块在报错”。"
        ),
        annotations=READ,
    )
    async def get_diagnostics(level: Literal["error", "warn", "all"] = "warn", limit: int = 15) -> dict:
        async def body():
            limit_n = max(1, min(int(limit or 15), 40))
            raw = ros.get(DIAG_TOPIC[0], max(stale, 0.0) * 3)  # 诊断约 1Hz，放宽到 3 个周期
            snap = snapshot()
            issues = autonomous_issues(snap)
            if raw is None:
                return {
                    "ok": True,
                    "available": False,
                    "reason": (
                        f"没有收到 /diagnostics 消息（超过 {stale * 3:.0f} 秒未更新）；"
                        "工控机可能未启用 diagnostic_aggregator，或该话题没有被发布"
                    ),
                    "issues": issues,
                    "vehicle_state": public_snap(snap),
                }
            counts = {"error": 0, "warn": 0, "stale": 0, "ok": 0}
            wanted = {"error": (2,), "warn": (1, 2, 3), "all": (0, 1, 2, 3)}.get(level, (0, 1, 2, 3))
            picked: list[dict] = []
            for item in raw.get("status") or []:
                lv = int(item.get("level", 0))
                if lv == 2:
                    counts["error"] += 1
                elif lv == 1:
                    counts["warn"] += 1
                elif lv == 3:
                    counts["stale"] += 1
                else:
                    counts["ok"] += 1
                if lv not in wanted:
                    continue
                picked.append({
                    "level": DIAG_LEVEL_NAME.get(lv, str(lv)),
                    "name": item.get("name", ""),
                    "message": item.get("message", ""),
                    "hardware_id": item.get("hardware_id", ""),
                    "values": {v.get("key"): v.get("value") for v in (item.get("values") or []) if v.get("key")},
                    "_rank": DIAG_LEVEL_RANK.get(lv, 9),
                })
            picked.sort(key=lambda it: (it["_rank"], it["name"]))
            items = [{k: v for k, v in it.items() if k != "_rank"} for it in picked[:limit_n]]
            return {
                "ok": True,
                "available": True,
                "level_filter": level,
                "counts": counts,
                "matched": len(picked),
                "truncated": len(picked) > limit_n,
                "items": items,
                "issues": issues,
                "vehicle_state": public_snap(snap),
            }
        return await run_tool("get_diagnostics", {"level": level, "limit": limit}, body)

    # ── 写工具 ──
    @mcp.tool(description="请求工控机根据 GNSS 自动完成定位初始化。自动驾驶中不可用。", annotations=WRITE)
    async def init_localization() -> dict:
        async def body():
            snap = snapshot()
            require_fresh(snap, "operation_mode")
            require(not snap["is_autonomous"], "自动驾驶中不能重新定位，请先停车", code="ALREADY_AUTONOMOUS")
            rate_limit()
            res = await ros.call_service(*SRV_INIT_LOC, {}, timeout=max(srv_timeout, 45.0))  # 点云匹配较慢
            require(res["success"], f"定位初始化失败：{res['message']}", code="SERVICE_FAILED")
            return {"ok": True, "message": "定位初始化完成"}
        return await run_tool("init_localization", {}, body)

    @mcp.tool(description="把车辆定位起点设置为站点表中的指定站点，消息格式与上位机“设起点”一致。仅在车辆非自动驾驶且停稳时使用。", annotations=DANGER)
    async def set_start_station(station_id: StationId) -> dict:  # type: ignore[valid-type]
        args = {"station_id": station_id}

        async def body():
            st = stations.get(station_id)
            require(st is not None, f"未知站点 {station_id}", code="UNKNOWN_STATION")
            snap = snapshot()
            require_fresh(snap, "operation_mode")
            require(not snap["is_autonomous"], "自动驾驶中不能重新设置起点，请先停车", code="ALREADY_AUTONOMOUS")
            require(float(snap.get("speed_kmh", 0.0)) < 0.5, "车辆尚未停稳，不能重新设置定位起点", code="VEHICLE_MOVING")
            rate_limit()
            res = await ros.call_service(*SRV_INIT_LOC, make_initialization_args(st["x"], st["y"], st["yaw"]), timeout=max(srv_timeout, 45.0))
            require(res["success"], f"设置起点失败：{res['message']}", code="SERVICE_FAILED")
            ready = await wait_for(lambda s: s.get("_loc_state") == 3, timeout=max(15.0, auto_ready_wait))
            require(ready, "定位服务已返回，但没有等到“已初始化”状态", code="LOC_NOT_READY")
            return {"ok": True, "station": st["name"], "message": f"已把定位起点设置为“{st['name']}”"}

        return await run_tool("set_start_station", args, body)

    @mcp.tool(description="只设置/预览指定站点的新路线，不启动自动驾驶。仅当用户明确要求“只规划、不发车”时使用。", annotations=WRITE)
    async def set_destination(station_id: StationId, ctx: Context) -> dict:  # type: ignore[valid-type]
        args = {"station_id": station_id}

        async def body():
            st = stations.get(station_id)
            require(st is not None, f"未知站点 {station_id}", code="UNKNOWN_STATION")
            snap = snapshot()
            require_fresh(snap, "operation_mode", "localization")
            require(snap["_loc_state"] == 3, f"定位状态为“{snap['localization']}”，请先完成定位初始化", code="LOC_NOT_READY")
            if snap["is_autonomous"]:
                await confirm(ctx, f"车辆正在自动驾驶，确认把目的地改为「{st['name']}」吗？车辆会按新路线行驶。")
            rate_limit()
            await ros.publish(*GOAL_TOPIC, make_pose(st["x"], st["y"], st["yaw"]))
            planned = await wait_for(lambda s: s.get("_route_state") == 2, timeout=5.0)
            return {"ok": True, "station": st["name"],
                    "route": "路线已生成" if planned else "已下发目的地，5 秒内未确认路线生成，请查看界面"}
        return await run_tool("set_destination", args, body)

    @mcp.tool(
        description=(
            "派车到指定站点的完整工作流：必要时退出自动驾驶，清除旧路线，检查并尝试修复定位，"
            "下发新终点，等待路线生成，诊断自动驾驶条件；条件满足后弹窗请求操作员确认，确认后切入自动驾驶并核验结果。"
        ),
        annotations=DANGER,
    )
    async def dispatch_to_station(station_id: StationId, ctx: Context, start_station_id: Optional[StationId] = None) -> dict:  # type: ignore[valid-type]
        args = {"station_id": station_id, "start_station_id": start_station_id}
        # 放在 body 外层：被拒绝时 run_tool 才能把“已完成到哪一步”一并返回。
        steps: list[str] = []

        async def body():
            st = stations.get(station_id)
            require(st is not None, f"未知站点 {station_id}", code="UNKNOWN_STATION")
            start_station = stations.get(start_station_id) if start_station_id else None
            require(start_station_id is None or start_station is not None, f"未知起点 {start_station_id}", code="UNKNOWN_STATION")

            snap = snapshot()
            require_fresh(snap, "operation_mode", "localization", "route")

            # 先统一回到 STOP，才能安全清理旧路线；手动/远程模式也不直接跨模式发车。
            if snap.get("_mode") != 1:
                previous_mode = snap.get("operation_mode", "未知模式")
                await pace_command()
                stopped = await ros.call_service(*SRV_TO_STOP, {}, timeout=srv_timeout)
                require(stopped["success"], f"无法从“{previous_mode}”切换到停车模式：{stopped['message']}", code="MODE_SWITCH_FAILED")
                changed = await wait_for(lambda s: s.get("_mode") == 1, timeout=auto_confirm_wait)
                require(changed, "已发送停车指令，但没有确认车辆进入停车模式；请检查运行模式", code="MODE_SWITCH_FAILED")
                steps.append(f"已从“{previous_mode}”切换到停车模式")

            # 用户明确给出起点时，使用与上位机“设起点”完全相同的带协方差定位服务请求。
            if start_station is not None:
                await pace_command()
                initialized = await ros.call_service(
                    *SRV_INIT_LOC,
                    make_initialization_args(start_station["x"], start_station["y"], start_station["yaw"]),
                    timeout=max(srv_timeout, 45.0),
                )
                require(initialized["success"], f"设置起点“{start_station['name']}”失败：{initialized['message']}", code="SERVICE_FAILED")
                ready = await wait_for(lambda s: s.get("_loc_state") == 3, timeout=max(15.0, auto_ready_wait))
                require(ready, "设置起点服务已返回，但没有等到“已初始化”状态", code="LOC_NOT_READY")
                steps.append(f"已把定位起点设置为“{start_station['name']}”")

            # 未指定起点且定位未就绪时，允许工作流自行按 GNSS 初始化一次。
            snap = snapshot()
            if snap.get("_loc_state") != 3:
                await pace_command()
                initialized = await ros.call_service(*SRV_INIT_LOC, {}, timeout=max(srv_timeout, 45.0))
                require(initialized["success"], f"自动定位初始化失败：{initialized['message']}", code="SERVICE_FAILED")
                ready = await wait_for(lambda s: s.get("_loc_state") == 3, timeout=max(15.0, auto_ready_wait))
                require(ready, "定位初始化服务已返回，但没有等到“已初始化”状态", code="LOC_NOT_READY")
                steps.append("已自动完成定位初始化")

            async def clear_old_route() -> None:
                current = snapshot()
                if current.get("_route_state") == 1:
                    return
                await pace_command()
                cleared = await ros.call_service(*SRV_CLEAR_ROUTE, {}, timeout=srv_timeout)
                require(cleared["success"], f"清除旧路线失败：{cleared['message']}", code="ROUTE_CLEAR_FAILED")
                cleared_state = await wait_for(lambda s: s.get("_route_state") == 1, timeout=5.0)
                require(cleared_state, "清除路线服务已返回，但路线状态没有变为“无路线”", code="ROUTE_CLEAR_FAILED")
                steps.append("已清除原有路线")

            async def publish_and_wait_route() -> bool:
                await pace_command()
                await ros.publish(*GOAL_TOPIC, make_pose(st["x"], st["y"], st["yaw"]))
                return await wait_for(lambda s: s.get("_route_state") == 2, timeout=route_wait)

            await clear_old_route()
            planned = await publish_and_wait_route()
            if not planned:
                # 路由状态卡住时做一次受控恢复：清理后重新下发同一个目标。
                steps.append("首次等待路线超时，正在自动重建路线")
                await clear_old_route()
                planned = await publish_and_wait_route()
            require(planned, f"已两次下发终点“{st['name']}”，但都没有等到路线生成；请检查 mission_planner/地图和路线状态", code="PLAN_TIMEOUT")
            steps.append(f"已生成前往“{st['name']}”的新路线")

            # 给 Autoware 一段时间完成规划和控制条件收敛。
            ready = await wait_for(lambda s: not autonomous_issues(s), timeout=auto_ready_wait)
            if not ready:
                snap = snapshot()
                # 若规划期间定位或路线丢失，各允许自动恢复一次。
                if snap.get("_loc_state") != 3:
                    await pace_command()
                    initialized = await ros.call_service(*SRV_INIT_LOC, {}, timeout=max(srv_timeout, 45.0))
                    require(initialized["success"], f"规划后定位丢失，自动恢复失败：{initialized['message']}", code="SERVICE_FAILED")
                    loc_ready = await wait_for(lambda s: s.get("_loc_state") == 3, timeout=max(15.0, auto_ready_wait))
                    require(loc_ready, "重新定位后没有等到“已初始化”状态", code="LOC_NOT_READY")
                    steps.append("已恢复定位")
                if snapshot().get("_route_state") != 2:
                    await clear_old_route()
                    require(await publish_and_wait_route(), "路线丢失且自动重建失败", code="PLAN_TIMEOUT")
                    steps.append("已恢复路线")
                ready = await wait_for(lambda s: not autonomous_issues(s), timeout=auto_ready_wait)

            snap = snapshot()
            issues = autonomous_issues(snap)
            require(ready and not issues, "自动排查后仍不能启动自动驾驶：" + "；".join(issues),
                    code="AUTONOMY_NOT_READY", issues=issues)

            pos = snap.get("position_map", {})
            await confirm(
                ctx,
                f"前往“{st['name']}”的路线已就绪。确认启动自动驾驶？当前位置 "
                f"x={pos.get('x')}, y={pos.get('y')}。请确认车辆周围安全且急停可用。",
            )

            # 操作员确认可能耗时，执行前再次做完整联锁检查。
            snap = snapshot()
            issues = autonomous_issues(snap)
            require(not issues, "确认期间车辆状态发生变化：" + "；".join(issues),
                    code="STATE_CHANGED", issues=issues)

            await pace_command()
            switched = await ros.call_service(*SRV_TO_AUTO, {}, timeout=srv_timeout)
            mode_confirmed = switched["success"] and await wait_for(
                lambda s: s.get("is_autonomous") is True,
                timeout=auto_confirm_wait,
            )
            if not mode_confirmed:
                # 服务偶发成功但状态未切换时，只在联锁仍满足的情况下重试一次。
                snap = snapshot()
                issues = autonomous_issues(snap)
                if not issues:
                    await pace_command()
                    switched = await ros.call_service(*SRV_TO_AUTO, {}, timeout=srv_timeout)
                    mode_confirmed = switched["success"] and await wait_for(
                        lambda s: s.get("is_autonomous") is True,
                        timeout=auto_confirm_wait,
                    )
                    if mode_confirmed:
                        steps.append("首次切换未确认，第二次切换成功")
            if not mode_confirmed:
                snap = snapshot()
                issues = autonomous_issues(snap)
                detail = "；".join(issues) if issues else (switched.get("message") or "运行模式未变为自动驾驶")
                raise ToolRefused(f"路线已保留，但自动驾驶切换失败：{detail}",
                                  code="MODE_SWITCH_FAILED", issues=issues)

            steps.append("已确认进入自动驾驶")
            return {
                "ok": True,
                "station": st["name"],
                "autonomous": True,
                "steps": steps,
                "message": f"车辆已进入自动驾驶，正在前往“{st['name']}”",
            }

        return await run_tool("dispatch_to_station", args, body, steps=steps)

    @mcp.tool(description="清除当前路线。自动驾驶中不可用。", annotations=WRITE)
    async def clear_route() -> dict:
        async def body():
            snap = snapshot()
            require_fresh(snap, "operation_mode")
            require(not snap["is_autonomous"], "自动驾驶中不能取消路线，请先停车", code="ALREADY_AUTONOMOUS")
            rate_limit()
            res = await ros.call_service(*SRV_CLEAR_ROUTE, {}, timeout=srv_timeout)
            require(res["success"], f"取消路线失败：{res['message']}", code="SERVICE_FAILED")
            return {"ok": True}
        return await run_tool("clear_route", {}, body)

    @mcp.tool(description="启动自动驾驶，车辆将沿当前路线行驶。必须已定位、已设路线，且需要操作员人工确认。", annotations=DANGER)
    async def start_autonomous(ctx: Context) -> dict:
        async def body():
            snap = snapshot()
            require_fresh(snap, "operation_mode", "localization", "route")
            require(not snap["is_autonomous"], "车辆已经在自动驾驶中", code="ALREADY_AUTONOMOUS")
            require(not snap["in_transition"], "车辆正在切换模式，请稍候", code="IN_TRANSITION")
            require(snap["_loc_state"] == 3, f"定位状态为“{snap['localization']}”，请先完成定位初始化", code="LOC_NOT_READY")
            require(snap["_route_state"] == 2, f"路线状态为“{snap['route']}”，请先设置目的地", code="ROUTE_MISSING")
            require(snap["autonomous_available"], "Autoware 报告当前不满足自动驾驶条件（定位不可信、车辆未停稳或存在紧急停车等）", code="AUTONOMY_NOT_READY")
            if cfg["chassis_feedback_topic"]:
                require("_brake_blocking" in snap, "没有收到底盘反馈，无法确认底盘状态", code="STATE_STALE")
                require(not snap["_brake_blocking"], f"底盘存在制动请求：{', '.join(snap['_brake_blocking'])}", code="CHASSIS_BRAKE")
            pos = snap.get("position_map", {})
            await confirm(ctx, f"确认启动自动驾驶？当前位置 x={pos.get('x')}, y={pos.get('y')}，"
                               f"最近站点「{snap.get('nearest_station', '未知')}」。请确认车辆周围安全。")
            # 确认可能耗时很久：执行前重新检查关键条件
            snap = snapshot()
            require_fresh(snap, "operation_mode")
            require(snap["autonomous_available"] and not snap["is_autonomous"], "确认期间车辆状态已变化，请重新发起", code="STATE_CHANGED")
            rate_limit()
            res = await ros.call_service(*SRV_TO_AUTO, {}, timeout=srv_timeout)
            require(res["success"], f"启动失败：{res['message']}", code="SERVICE_FAILED")
            return {"ok": True, "message": "已切换到自动驾驶"}
        return await run_tool("start_autonomous", {}, body)

    @mcp.tool(description="停车：退出自动驾驶，车辆平稳减速停住。任何时候都可调用，无需确认。注意这不是物理急停。", annotations=WRITE)
    async def stop_vehicle() -> dict:
        async def body():  # 安全方向的指令：不做前置条件、不限频、不确认
            res = await ros.call_service(*SRV_TO_STOP, {}, timeout=srv_timeout)
            require(res["success"], f"停车指令失败：{res['message']}，请立即使用界面停车按钮或物理急停", code="SERVICE_FAILED")
            return {"ok": True, "message": "已发送停车指令"}
        return await run_tool("stop_vehicle", {}, body)

    return mcp


def main() -> None:
    ap = argparse.ArgumentParser(description="无人车业务 MCP server")
    ap.add_argument("--config", default="config.json")
    ap.add_argument("--transport", default="stdio", choices=["stdio", "streamable-http"])
    a = ap.parse_args()
    build_server(load_config(a.config)).run(transport=a.transport)


if __name__ == "__main__":
    main()
