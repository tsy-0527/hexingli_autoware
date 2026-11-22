import { useCallback, useEffect, useRef, useState } from "react";

const TOOL_LABELS = {
  get_vehicle_status: "查询车辆状态",
  list_stations: "查询可用站点",
  get_diagnostics: "查询模块诊断",
  init_localization: "初始化定位",
  set_start_station: "设置站点起点",
  set_destination: "设置目的地",
  dispatch_to_station: "执行智能派车",
  clear_route: "取消路线",
  start_autonomous: "启动自动驾驶",
  stop_vehicle: "停车",
};

// 常驻在主界面最右侧的一栏（方案 A）：不再是悬浮按键 + 弹出面板。
// 收起后只保留一条竖条，点击即可展开；需要人工确认时会自动展开。
const CSS = `
.va-dock{height:100%;min-width:0;display:flex;flex-direction:column;overflow:hidden;background:rgba(5,10,20,.92);color:#e5edf8;font-family:system-ui,-apple-system,"Microsoft YaHei",sans-serif}
.va-head{flex:0 0 auto;height:40px;padding:0 10px 0 12px;display:flex;align-items:center;gap:8px;border-bottom:1px solid var(--line,rgba(255,255,255,.08));background:linear-gradient(90deg,rgba(63,212,255,.08),transparent)}
.va-dot{width:8px;height:8px;border-radius:50%;background:#5b6b84;flex:0 0 auto}.va-dot.ready{background:#34f5a4;box-shadow:0 0 8px #34f5a4}.va-dot.thinking,.va-dot.connecting{background:#ffb547;animation:va-pulse 1.1s infinite}.va-dot.error{background:#ff5b6b}
.va-title{flex:1;min-width:0}.va-title b{display:block;font-size:13px;letter-spacing:.5px}.va-title small{display:block;margin-top:2px;color:#8293aa;font-size:10px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.va-fold{width:26px;height:26px;border-radius:7px;border:1px solid rgba(255,255,255,.12);background:rgba(255,255,255,.04);color:#9fb2c8;cursor:pointer;display:grid;place-items:center}.va-fold:hover{color:#fff;border-color:rgba(63,212,255,.5)}
.va-state{flex:0 0 auto;display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:8px 10px;border-bottom:1px solid rgba(255,255,255,.06)}
.va-cell{padding:6px 8px;border-radius:8px;background:rgba(255,255,255,.035);border:1px solid rgba(255,255,255,.06);font-size:10px;color:#7f93ab;line-height:1.35}.va-cell b{display:block;margin-top:2px;font-size:12px;color:#dce8f7}.va-cell b.ok{color:#34f5a4}.va-cell b.warn{color:#ffc766}.va-cell b.bad{color:#ff8e9a}.va-cell b.sim{color:#9fe9ff}
.va-alert{flex:0 0 auto;padding:7px 11px;border-bottom:1px solid rgba(255,181,71,.25);background:rgba(255,181,71,.08);color:#ffd89a;font-size:11px;line-height:1.45}.va-alert.error{border-color:rgba(255,91,107,.3);background:rgba(255,91,107,.08);color:#ffc0c7}
.va-messages{flex:1;min-height:0;overflow:auto;padding:11px;display:flex;flex-direction:column;gap:9px;scrollbar-width:thin;scrollbar-color:#27415c transparent}.va-msg{max-width:92%;padding:8px 10px;border-radius:11px;font-size:12px;line-height:1.55;white-space:pre-wrap;overflow-wrap:anywhere}.va-msg.user{align-self:flex-end;background:linear-gradient(135deg,#1679a4,#15557f);color:#fff;border-bottom-right-radius:4px}.va-msg.assistant{align-self:flex-start;background:#111d2d;border:1px solid rgba(255,255,255,.07);color:#dce8f7;border-bottom-left-radius:4px}.va-msg.event{align-self:center;max-width:98%;padding:4px 10px;border:1px solid rgba(63,212,255,.15);background:rgba(63,212,255,.05);color:#8aa0b8;font-size:10px;border-radius:999px}
.va-empty{margin:auto;text-align:center;color:#60758e;font-size:12px;line-height:1.8;padding:0 8px}.va-empty b{display:block;color:#9fe9ff;font-size:14px;margin-bottom:6px}.va-empty span{display:block;color:#41566e;font-size:10px;margin-top:8px}
.va-quick{flex:0 0 auto;display:flex;flex-wrap:wrap;gap:5px;padding:7px 10px 0}.va-chip{height:24px;padding:0 9px;border-radius:999px;border:1px solid rgba(63,212,255,.28);background:rgba(63,212,255,.06);color:#a8e9ff;font-size:11px;cursor:pointer;white-space:nowrap;max-width:100%;overflow:hidden;text-overflow:ellipsis}.va-chip:hover:not(:disabled){border-color:#73e4ff;color:#fff}.va-chip.stop{border-color:rgba(255,91,107,.5);background:rgba(255,91,107,.1);color:#ffb3bb}.va-chip:disabled{opacity:.4;cursor:not-allowed}
.va-compose{flex:0 0 auto;padding:8px 10px 10px;border-top:1px solid rgba(255,255,255,.08);margin-top:7px;background:rgba(3,8,17,.7)}.va-compose form{display:flex;gap:6px}.va-input{flex:1;min-width:0;height:36px;padding:0 10px;border:1px solid rgba(255,255,255,.12);border-radius:9px;background:#081221;color:#e5edf8;outline:none;font:12px system-ui}.va-input:focus{border-color:rgba(63,212,255,.6);box-shadow:0 0 0 2px rgba(63,212,255,.08)}.va-send{width:54px;border:1px solid rgba(63,212,255,.5);border-radius:9px;background:rgba(63,212,255,.12);color:#87e8ff;font-size:12px;font-weight:700;cursor:pointer}.va-send:disabled,.va-input:disabled{opacity:.45;cursor:not-allowed}.va-foot{display:flex;justify-content:space-between;align-items:center;margin-top:6px;color:#40536a;font-size:9px}.va-cancel{border:0;background:transparent;color:#ff9aa4;font-size:10px;cursor:pointer}
.va-strip{height:100%;width:100%;display:flex;flex-direction:column;align-items:center;gap:12px;padding-top:12px;border:0;background:rgba(5,10,20,.92);color:#9fb2c8;cursor:pointer}.va-strip:hover{color:#fff;background:rgba(63,212,255,.07)}.va-strip span{writing-mode:vertical-rl;letter-spacing:4px;font-size:12px}
.va-confirm{flex:0 0 auto;margin:8px 10px 0;padding:11px 12px;border:1px solid rgba(255,181,71,.75);border-radius:12px;background:linear-gradient(145deg,rgba(40,30,14,.95),rgba(20,16,12,.95));box-shadow:0 0 22px rgba(255,181,71,.14);animation:va-fade-in .16s ease-out}.va-confirm b{display:flex;align-items:center;gap:7px;color:#ffe0a8;font-size:13px;margin-bottom:6px}.va-confirm b i{width:18px;height:18px;border-radius:50%;display:grid;place-items:center;background:rgba(255,181,71,.18);color:#ffc766;font-style:normal;font-size:12px}.va-confirm p{margin:0 0 6px;color:#f4ead9;font-size:12px;line-height:1.6;white-space:pre-wrap}.va-confirm small{display:block;color:#aa987c;font-size:10px;margin-bottom:9px}.va-confirm-actions{display:flex;gap:8px}.va-confirm button{flex:1;height:34px;border-radius:9px;font-size:12px;cursor:pointer}.va-deny{border:1px solid rgba(255,255,255,.16);background:rgba(255,255,255,.05);color:#c1ccda}.va-deny:focus{outline:2px solid #8293aa;outline-offset:2px}.va-accept{border:1px solid #ffbd59;background:linear-gradient(135deg,#f29a25,#d97813);color:#fff;font-weight:700;box-shadow:0 6px 18px rgba(217,120,19,.25)}
@keyframes va-pulse{50%{opacity:.35}}@keyframes va-fade-in{from{opacity:0}to{opacity:1}}
`;

function messageText(error) {
  return error?.message || String(error || "未知错误");
}

export default function AgentPanel({
  teleopActive,
  vehicleConnected,
  vehicleChannel,
  driveMode = "real",
  isAutonomous = false,
  autoAvailable = false,
  autoTransitioning = false,
  stationsVersion = 0,
  collapsed = false,
  onCollapsedChange,
  onStop,
  onStartStation,
  headerControl,
}) {
  const api = typeof window !== "undefined" ? window.vehicleAgent : null;
  const [input, setInput] = useState("");
  const [messages, setMessages] = useState([]);
  const [busy, setBusy] = useState(false);
  const [status, setStatus] = useState({ state: api ? "idle" : "error", message: api ? "Agent 待命" : "请使用 Electron 版本" });
  const [confirmation, setConfirmation] = useState(null);
  const [confirmLeft, setConfirmLeft] = useState(0);
  const [stations, setStations] = useState([]);
  const endRef = useRef(null);
  const onCollapsedChangeRef = useRef(onCollapsedChange);
  onCollapsedChangeRef.current = onCollapsedChange;
  const onStartStationRef = useRef(onStartStation);
  onStartStationRef.current = onStartStation;

  useEffect(() => {
    if (!api) return undefined;
    api.getStatus().then((next) => next && setStatus(next)).catch((error) => (
      setStatus({ state: "error", message: messageText(error) })
    ));
    return api.onEvent((event) => {
      if (!event) return;
      if (event.type === "status") setStatus(event);
      if (event.type === "start-station" && event.station) onStartStationRef.current?.(event.station);
      if (event.type === "tool-start") {
        setMessages((list) => [...list, { role: "event", text: `正在${event.label || TOOL_LABELS[event.name] || event.name}…` }]);
      }
      if (event.type === "tool-end") {
        const refused = Boolean(event.result?.refused);
        setMessages((list) => [...list, { role: "event", text: refused ? `${event.label}被拒绝` : `${event.label}已返回结果` }]);
      }
      if (event.type === "confirmation") {
        setConfirmation(event);
        onCollapsedChangeRef.current?.(false);
        if (typeof window !== "undefined") window.focus();
      }
      if (event.type === "confirmation-expired") {
        setConfirmation((current) => (current?.id === event.id ? null : current));
        setMessages((list) => [...list, { role: "event", text: "人工确认已超时，操作已取消" }]);
      }
    });
  }, [api]);

  useEffect(() => {
    if (!api) return;
    api.setContext({ teleopActive, vehicleConnected, vehicleChannel, driveMode }).catch((error) => (
      setStatus({ state: "error", message: messageText(error) })
    ));
  }, [api, teleopActive, vehicleConnected, vehicleChannel, driveMode]);

  // 快捷“去某站”按钮：站点表增删改后（stationsVersion 变化）重新读取
  useEffect(() => {
    if (!api?.getStations) return;
    api.getStations()
      .then((result) => setStations(Array.isArray(result?.stations) ? result.stations : []))
      .catch(() => setStations([]));
  }, [api, stationsVersion]);

  useEffect(() => {
    endRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages, confirmation]);

  // 人工确认倒计时（主进程 60 秒未确认会自动取消）
  useEffect(() => {
    if (!confirmation) return undefined;
    const deadline = Date.now() + (Number(confirmation.timeoutMs) || 60000);
    const tick = () => setConfirmLeft(Math.max(0, Math.ceil((deadline - Date.now()) / 1000)));
    tick();
    const id = setInterval(tick, 500);
    return () => clearInterval(id);
  }, [confirmation]);

  const sendText = useCallback(async (raw) => {
    const text = String(raw || "").trim();
    if (!api || !text || busy) return;
    setBusy(true);
    setMessages((list) => [...list, { role: "user", text }]);
    try {
      const result = await api.sendMessage(text);
      setMessages((list) => [...list, { role: "assistant", text: result?.text || "操作已完成。" }]);
    } catch (error) {
      setMessages((list) => [...list, { role: "assistant", text: `无法完成：${messageText(error)}` }]);
    } finally {
      setBusy(false);
    }
  }, [api, busy]);

  const send = (event) => {
    event.preventDefault();
    const text = input.trim();
    if (!text) return;
    setInput("");
    sendText(text);
  };

  const answerConfirmation = async (accepted) => {
    const current = confirmation;
    if (!current || !api) return;
    setConfirmation(null);
    await api.respondConfirmation(current.id, accepted);
    setMessages((list) => [...list, { role: "event", text: accepted ? "操作员已确认，正在重新检查车辆状态" : "操作员已取消该操作" }]);
  };

  const cancel = async () => {
    await api?.cancel();
    setBusy(false);
    setConfirmation(null);
  };

  const statusMessage = !api
    ? "仅 Electron 版本可用"
    : !status.hasApiKey && status.state !== "ready" && status.state !== "thinking"
      ? "未检测到 DEEPSEEK_API_KEY"
      : status.message;

  const autoCell = !vehicleConnected
    ? { text: "未连接", tone: "bad" }
    : isAutonomous
      ? { text: "自动驾驶中", tone: "ok" }
      : autoTransitioning
        ? { text: "切换中…", tone: "warn" }
        : autoAvailable
          ? { text: "可启动", tone: "ok" }
          : { text: "暂不可用", tone: "warn" };
  const inputLocked = !api || busy || Boolean(confirmation);

  // 人工确认只在智能调度栏内显示（不再全屏遮罩），点按钮即可
  const confirmCard = confirmation && (
    <div className="va-confirm" role="alertdialog" aria-labelledby="va-confirm-title" aria-describedby="va-confirm-message">
      <b id="va-confirm-title"><i>!</i>{confirmation.kind === "autonomous" ? "确认启动自动驾驶" : "需要操作员确认"}</b>
      <p id="va-confirm-message">{confirmation.message}</p>
      <small>请确认车辆周围安全且急停可用 · {confirmLeft} 秒后自动取消</small>
      <div className="va-confirm-actions">
        <button autoFocus type="button" className="va-deny" onClick={() => answerConfirmation(false)}>取消</button>
        <button type="button" className="va-accept" onClick={() => answerConfirmation(true)}>
          {confirmation.kind === "autonomous" ? "确认启动" : "确认执行"}
        </button>
      </div>
    </div>
  );

  if (collapsed) {
    return (
      <>
        <style>{CSS}</style>
        <button type="button" className="va-strip" onClick={() => onCollapsedChange?.(false)} title={`展开智能调度 · ${statusMessage}`}>
          <i className={`va-dot ${status.state}`} />
          <span>智能调度</span>
        </button>
      </>
    );
  }

  return (
    <>
      <style>{CSS}</style>
      <section className="va-dock" aria-label="车辆智能调度助手">
        <div className="va-head">
          <i className={`va-dot ${status.state}`} />
          <div className="va-title" title={`${statusMessage}${status.model ? ` · ${status.model}` : ""}`}>
            <b>智能调度</b>
          </div>
          {headerControl}
          <button type="button" className="va-fold" onClick={() => onCollapsedChange?.(true)} title="收起到右侧竖条">
            <svg viewBox="0 0 24 24" width="14" height="14" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M9 6l6 6-6 6" /></svg>
          </button>
        </div>

        <div className="va-state">
          <div className="va-cell">运行模式<b className={driveMode === "sim" ? "sim" : ""}>{driveMode === "sim" ? "仿真" : "实车"}</b></div>
          <div className="va-cell">Autoware 自动驾驶<b className={autoCell.tone}>{autoCell.text}</b></div>
        </div>

        {teleopActive && <div className="va-alert">底盘手动遥控已激活：Agent 只允许查询，所有车辆写操作都会被拒绝。</div>}
        {!vehicleConnected && <div className="va-alert error">车辆未连接。查询和操作可能失败，请先检查 rosbridge 通道。</div>}

        <div className="va-messages">
          {!messages.length && (
            <div className="va-empty">
              <b>用自然语言调度车辆</b>
              例如“去 8 号楼”“从发车点去停车位”“查询车辆状态”
              <span>Agent 不控制速度、方向、档位，也不能替代物理急停</span>
            </div>
          )}
          {messages.map((message, index) => (
            <div className={`va-msg ${message.role}`} key={`${message.role}-${index}`}>{message.text}</div>
          ))}
          <div ref={endRef} />
        </div>

        {confirmCard}

        <div className="va-quick">
          <button type="button" className="va-chip" disabled={inputLocked} onClick={() => sendText("查询车辆状态")}>车辆状态</button>
          {stations.slice(0, 8).map((station) => (
            <button type="button" className="va-chip" key={station.id} disabled={inputLocked} title={`派车前往“${station.name}”（启动前会弹窗确认）`} onClick={() => sendText(`去${station.name}`)}>
              去{station.name}
            </button>
          ))}
          <button type="button" className="va-chip stop" disabled={!vehicleConnected || !onStop} title="直接调用 Autoware 停车服务（不经过大模型，不能替代物理急停）" onClick={() => { if (busy) cancel(); onStop?.(); }}>停车</button>
        </div>

        <div className="va-compose">
          <form onSubmit={send}>
            <input
              className="va-input"
              value={input}
              onChange={(event) => setInput(event.target.value)}
              placeholder="输入调度指令…"
              maxLength={2000}
              disabled={inputLocked}
            />
            <button className="va-send" type="submit" disabled={inputLocked || !input.trim()}>发送</button>
          </form>
          <div className="va-foot">
            <span>所有车辆操作均写入 MCP 审计日志</span>
            {busy && <button type="button" className="va-cancel" onClick={cancel}>取消本轮</button>}
          </div>
        </div>
      </section>
    </>
  );
}
