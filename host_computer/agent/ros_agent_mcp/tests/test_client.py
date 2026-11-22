"""端到端测试：启动 server.py（stdio），连到 fake_rosbridge，依次调用工具，校验白名单/确认/前置条件。"""
import asyncio, json, os, sys, tempfile
from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client
import mcp.types as types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ANSWER = {"value": True}  # 模拟操作员在确认弹窗上的选择

async def on_elicit(ctx, params):
    print(f"  [确认弹窗] {params.message} -> {'同意' if ANSWER['value'] else '拒绝'}")
    return types.ElicitResult(action="accept", content={"confirm": ANSWER["value"]})

def text(res):
    return json.loads(res.content[0].text)

async def main(port):
    cfg = json.load(open(os.path.join(ROOT, "config.example.json"), encoding="utf-8"))
    cfg["rosbridge_url"] = f"ws://127.0.0.1:{port}"
    # fake_rosbridge 不模拟实车 CCU 反馈；仅在此离线测试中关闭底盘反馈前置检查。
    cfg["chassis_feedback_topic"] = None
    cfg["min_command_interval_seconds"] = 0
    tmp = tempfile.mkdtemp()
    cfg["audit_log"] = os.path.join(tmp, "audit.jsonl")
    cfg_path = os.path.join(tmp, "config.json")
    json.dump(cfg, open(cfg_path, "w", encoding="utf-8"), ensure_ascii=False)

    params = StdioServerParameters(command=sys.executable, args=[os.path.join(ROOT, "server.py"), "--config", cfg_path])
    async with stdio_client(params) as (r, w):
        async with ClientSession(r, w, elicitation_callback=on_elicit) as s:
            await s.initialize()
            tools = (await s.list_tools()).tools
            print("工具列表：", [t.name for t in tools])
            assert {t.name for t in tools} == {"get_vehicle_status", "list_stations", "get_diagnostics", "init_localization",
                                                "set_start_station", "set_destination", "dispatch_to_station", "clear_route",
                                                "start_autonomous", "stop_vehicle"}
            sd = next(t for t in tools if t.name == "set_destination")
            enum = (getattr(sd, "input_schema", None) or sd.inputSchema)["properties"]["station_id"].get("enum")
            print("set_destination 站点枚举：", enum)
            assert enum == ["gate3", "charger", "dock_a"]

            print("状态：", text(await s.call_tool("get_vehicle_status", {})))

            dg = text(await s.call_tool("get_diagnostics", {"level": "error"}))
            print("诊断（仅 ERROR）：", json.dumps(dg, ensure_ascii=False)[:400])
            assert dg["available"] and dg["counts"]["error"] == 1 and dg["counts"]["warn"] == 1
            assert [i["level"] for i in dg["items"]] == ["ERROR"]
            assert dg["items"][0]["values"]["module"] == "mission_planner"

            r1 = await s.call_tool("start_autonomous", {})
            print("未设路线就启动：", text(r1)); assert text(r1).get("refused")
            # 结构化拒绝：必须带稳定错误码、处置提示和现场快照，模型据此才能分析原因
            assert text(r1)["code"] == "ROUTE_MISSING"
            assert text(r1)["hint"] and text(r1)["vehicle_state"]["route"] == "无路线"
            # 拒绝结果必须带逐条列出的 check_items，供模型分点分行输出
            assert isinstance(text(r1)["check_items"], list) and text(r1)["check_items"]

            r2 = await s.call_tool("set_destination", {"station_id": "not_a_station"})
            print("非法站点：isError =", getattr(r2, "is_error", None) or getattr(r2, "isError", None)); assert getattr(r2, "is_error", None) or getattr(r2, "isError", None)

            r3 = await s.call_tool("set_destination", {"station_id": "gate3"})
            print("设目的地：", text(r3)); assert text(r3)["ok"]

            ANSWER["value"] = False
            r4 = await s.call_tool("start_autonomous", {})
            print("操作员拒绝：", text(r4)); assert text(r4).get("refused")
            assert text(r4)["code"] == "CONFIRM_DECLINED"

            ANSWER["value"] = True
            r5 = await s.call_tool("start_autonomous", {})
            print("操作员同意：", text(r5)); assert text(r5)["ok"]
            await asyncio.sleep(0.5)

            r6 = await s.call_tool("clear_route", {})
            print("自动驾驶中取消路线：", text(r6)); assert text(r6).get("refused")
            assert text(r6)["code"] == "ALREADY_AUTONOMOUS"

            r7 = await s.call_tool("stop_vehicle", {})
            print("停车：", text(r7)); assert text(r7)["ok"]
            await asyncio.sleep(0.5)

            ANSWER["value"] = True
            r8 = await s.call_tool("dispatch_to_station", {"start_station_id": "gate3", "station_id": "charger"})
            print("完整派车工作流：", text(r8))
            assert text(r8)["ok"] and text(r8)["autonomous"]

            r9 = await s.call_tool("stop_vehicle", {})
            print("派车后停车：", text(r9)); assert text(r9)["ok"]

    print("\n审计日志：")
    for line in open(cfg["audit_log"], encoding="utf-8"):
        rec = json.loads(line); print(" ", rec["tool"], rec["decision"], rec["args"])
    print("\n全部通过")

if __name__ == "__main__":
    asyncio.run(main(int(sys.argv[1]) if len(sys.argv) > 1 else 19090))
