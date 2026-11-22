"""模拟 rosbridge：回放 Autoware 状态话题、应答 AD API 服务，用于离线测试 server.py。"""
import asyncio, json, sys
import websockets

state = {"mode": 1, "loc": 3, "route": 1, "auto_avail": True}
log = []

# 模拟 diagnostic_aggregator 的输出：一个 ERROR、一个 WARN、一个 OK。
DIAG = {
    "status": [
        {"level": 2, "name": "planning/mission_planning", "message": "no route available",
         "hardware_id": "planner", "values": [{"key": "module", "value": "mission_planner"}]},
        {"level": 1, "name": "localization/ndt_scan_matcher", "message": "matching score low",
         "hardware_id": "localization", "values": [{"key": "score", "value": "1.2"}]},
        {"level": 0, "name": "system/vehicle_info", "message": "", "hardware_id": "", "values": []},
    ]
}

async def handler(ws):
    subs = set()
    async def pump():
        while True:
            msgs = {
                "/api/operation_mode/state": {"mode": state["mode"], "is_autonomous_mode_available": state["auto_avail"], "is_in_transition": False},
                "/api/localization/initialization_state": {"state": state["loc"]},
                "/api/routing/state": {"state": state["route"]},
                "/localization/kinematic_state": {"pose": {"pose": {"position": {"x": 11.0, "y": 4.0}}}, "twist": {"twist": {"linear": {"x": 0.0}}}},
                "/diagnostics": DIAG,
            }
            for t, m in msgs.items():
                if t in subs:
                    await ws.send(json.dumps({"op": "publish", "topic": t, "msg": m}))
            await asyncio.sleep(0.2)
    task = asyncio.create_task(pump())
    try:
        async for raw in ws:
            m = json.loads(raw)
            log.append(m)
            print("RECV", json.dumps(m, ensure_ascii=False), file=sys.stderr, flush=True)
            if m["op"] == "subscribe":
                subs.add(m["topic"])
            elif m["op"] == "publish" and m["topic"] == "/planning/mission_planning/goal":
                state["route"] = 2
            elif m["op"] == "call_service":
                s = m["service"]
                if s.endswith("change_to_autonomous"): state["mode"] = 2
                if s.endswith("change_to_stop"): state["mode"] = 1
                if s.endswith("clear_route"): state["route"] = 1
                await ws.send(json.dumps({"op": "service_response", "id": m["id"], "result": True,
                                          "values": {"status": {"success": True, "code": 0, "message": ""}}}))
    finally:
        task.cancel()

async def main(port):
    async with websockets.serve(handler, "127.0.0.1", port):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main(int(sys.argv[1]) if len(sys.argv) > 1 else 9090))
