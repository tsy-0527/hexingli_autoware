import { useState } from "react";

const box = { background: "rgba(10,14,26,.96)", border: "1px solid #22304a", borderRadius: 8, padding: "8px 10px", marginBottom: 8 };
const h = { color: "#7cc4f0", fontSize: 11, fontWeight: 700, marginBottom: 4, letterSpacing: .5 };
const row = { display: "flex", justifyContent: "space-between", gap: 8, fontFamily: "monospace", fontSize: 11, lineHeight: 1.7 };
const ok = { color: "#66eebb" };
const bad = { color: "#ff8888" };
const dim = { color: "#8899aa" };

function Line({ k, v, style }) {
  return (
    <div style={row}>
      <span style={dim}>{k}</span>
      <span style={style}>{v}</span>
    </div>
  );
}

const fmtAge = (ms) => (ms == null ? "未收到" : ms < 2000 ? "刚刚" : (ms / 1000).toFixed(1) + "s 前");

// 一键诊断面板：只读信息汇总，用来回答“AD API 起没起 / 键盘指令有没有人收 / 定位在抖吗”。
export default function DiagnosticsPanel({ open, onClose, diag, onRun, running }) {
  const [copied, setCopied] = useState(false);
  if (!open) return null;

  const services = diag?.services || [];
  const modeServices = services.filter((s) => s.includes("operation_mode"));
  const locServices = services.filter((s) => s.includes("localization"));
  const subscribers = diag?.cmdVelSubscribers || [];
  const jitter = diag?.jitter || {};
  // 三种证据任一成立即认为 AD API 在线：最近有 /api/... 回包、有模式状态话题、
  // 或 rosapi 的服务列表里出现了 /api/... 服务（模式状态是变化时才推送，单靠它会误报）。
  const rosapiHasAdapi = services.some((s) => s.includes("/api/operation_mode/") || s.includes("/api/localization/"));
  const apiAliveMs = diag?.adapi?.apiAliveAgeMs;
  const adapiOnline = (apiAliveMs != null && apiAliveMs < 120000) || (diag?.adapi?.modeStateAgeMs != null && diag.adapi.modeStateAgeMs < 120000) || rosapiHasAdapi;
  const shaky = (jitter.stillDxStdMm || 0) > 5 || (jitter.stillDyStdMm || 0) > 5;

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(JSON.stringify(diag, null, 2));
      setCopied(true);
      setTimeout(() => setCopied(false), 1500);
    } catch {
      setCopied(false);
    }
  };

  return (
    <div style={{ position: "absolute", right: 12, bottom: 12, width: 400, maxHeight: "72vh", overflowY: "auto", zIndex: 40, background: "rgba(8,11,20,.98)", border: "1px solid #2a6fa8", borderRadius: 10, padding: 10, color: "#cdd6e4", fontSize: 12, boxShadow: "0 8px 30px rgba(0,0,0,.5)" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 8 }}>
        <b style={{ color: "#7cc4f0" }}>自检 / 诊断</b>
        <span style={{ ...dim, fontSize: 10 }}>{diag?.at ? new Date(diag.at).toLocaleTimeString() : ""}</span>
        <button onClick={onRun} disabled={running} style={{ marginLeft: "auto", padding: "3px 10px", borderRadius: 5, border: "1px solid #2a6fa8", background: running ? "rgba(180,140,0,.25)" : "rgba(18,40,66,.92)", color: running ? "#ffcc33" : "#7cc4f0", fontSize: 11, cursor: running ? "progress" : "pointer" }}>
          {running ? "检测中…" : "重新检测"}
        </button>
        <button onClick={copy} style={{ padding: "3px 8px", borderRadius: 5, border: "1px solid #34445a", background: "rgba(20,28,43,.92)", color: "#9fb3c9", fontSize: 11, cursor: "pointer" }}>{copied ? "已复制" : "复制"}</button>
        <button onClick={onClose} style={{ padding: "3px 8px", borderRadius: 5, border: "1px solid #34445a", background: "rgba(40,20,20,.92)", color: "#ff9999", fontSize: 11, cursor: "pointer" }}>关闭</button>
      </div>

      {!diag ? (
        <div style={{ ...dim, padding: "10px 0" }}>点“重新检测”开始，或先确认已连上 rosbridge。</div>
      ) : (
        <>
          <div style={box}>
            <div style={h}>链路 / 话题</div>
            <Line k="WebSocket" v={diag.link.connected ? "已连接" : "断开"} style={diag.link.connected ? ok : bad} />
            <Line k="地址" v={diag.link.url || "-"} />
            <Line k="定位数据年龄" v={diag.link.dataAgeSec == null ? "等待数据" : diag.link.dataAgeSec + "s"} style={diag.link.dataAgeSec != null && diag.link.dataAgeSec > 5 ? bad : ok} />
            {diag.link.topics.map((t) => (
              <Line key={t.topic} k={t.topic.replace(/^\/planning\/scenario_planning\//, ".../")} v={t.hz ? t.hz + " Hz · " + fmtAge(t.ageMs) : "未收到"} style={t.hz ? ok : dim} />
            ))}
          </div>

          <div style={box}>
            <div style={h}>Autoware AD API</div>
            <Line k="AD API 在线" v={adapiOnline ? "是" : "未检测到"} style={adapiOnline ? ok : bad} />
            <Line k="最近 /api 服务回包" v={fmtAge(diag.adapi.apiAliveAgeMs)} />
            <Line k="模式状态话题（仅变化时推送）" v={fmtAge(diag.adapi.modeStateAgeMs)} />
            <Line k="可用性话题 availability" v={fmtAge(diag.adapi.availabilityAgeMs)} />
            {diag.adapi.availability && (
              <Line
                k="模式可用性 stop/AUTO/local/remote"
                v={["stop", "autonomous", "local", "remote"].map((f) => (diag.adapi.availability[f] ? "可用" : "不可用")).join(" / ")}
                style={diag.adapi.availability.autonomous ? ok : bad}
              />
            )}
            <Line k="定位初始化状态" v={diag.adapi.localizationState ? JSON.stringify(diag.adapi.localizationState) : "未收到"} />
            <Line k="当前模式服务名" v={(diag.modeService && diag.modeService.manual) || "-"} />
            {diag.adapi.availability && !diag.adapi.availability.autonomous && (
              <div style={{ ...dim, marginTop: 4, lineHeight: 1.6 }}>
                AUTO 不可用：Autoware 判定切换条件不满足（最常见是车辆不在规划轨迹上，其次是未停稳、定位未初始化、MRM 生效）。工控机执行 ros2 topic echo /control/autoware_operation_mode_transition_manager/debug_info 可看到具体哪一项为 false。
              </div>
            )}
            {!adapiOnline && (
              <div style={{ ...bad, marginTop: 4, lineHeight: 1.6 }}>
                ⚠ 未检测到 AD API：Autoware（仿真或实车）没启动，或 rosbridge 与它不在同一 ROS 环境（DOMAIN_ID / RMW / 网卡）。
              </div>
            )}
            {adapiOnline && diag.adapi.modeStateAgeMs == null && (
              <div style={{ ...dim, marginTop: 4, lineHeight: 1.6 }}>
                模式状态话题暂未推送属正常（该话题只在模式变化时发布）；点一次模式按钮即可让状态刷新。
              </div>
            )}
          </div>

          <div style={box}>
            <div style={h}>服务列表（rosapi）</div>
            {diag.rosapi === "missing" ? (
              <div style={{ ...bad, lineHeight: 1.6 }}>未拿到服务列表：rosapi 未启动。可执行 <code>ros2 run rosapi rosapi_node</code> 后再检测。</div>
            ) : diag.rosapi === "pending" ? (
              <div style={dim}>查询中…</div>
            ) : (
              <>
                <Line k="operation_mode 服务数" v={String(modeServices.length)} style={modeServices.length ? ok : bad} />
                {modeServices.slice(0, 6).map((s) => <Line key={s} k="·" v={s} />)}
                <Line k="localization 服务数" v={String(locServices.length)} />
                {locServices.slice(0, 4).map((s) => <Line key={s} k="·" v={s} />)}
                <Line
                  k="初始化定位服务（实车 GNSS 模式用）"
                  v={services.some((s) => s.includes("/api/localization/initialize")) ? "存在" : "不存在"}
                  style={services.some((s) => s.includes("/api/localization/initialize")) ? ok : dim}
                />
                <div style={{ ...dim, marginTop: 4, lineHeight: 1.6 }}>户外实车用组合导航(GNSS-INS)时必须点“初始化定位”才能起步；仿真(planning_simulator)下定位由仿真直接给出，报错可忽略，不点也不影响。</div>
              </>
            )}
          </div>

          <div style={box}>
            <div style={h}>/cmd_vel 订阅者（谁在收键盘指令）</div>
            {subscribers.length ? subscribers.map((n) => <Line key={n} k="·" v={n} style={ok} />) : (
              <div style={{ ...bad, lineHeight: 1.6 }}>没有订阅者：键盘发出的 /cmd_vel 没人接收，车不会动。需要底盘遥控链路，或把网页接到 /control/command/control_cmd。</div>
            )}
          </div>

          <div style={box}>
            <div style={h}>定位抖动（相邻帧位移）</div>
            <Line k="样本 / 静止样本" v={(jitter.samples ?? 0) + " / " + (jitter.stillSamples ?? 0)} />
            <Line k="静止时 σ(x) / σ(y)" v={(jitter.stillDxStdMm ?? "-") + " / " + (jitter.stillDyStdMm ?? "-") + " mm"} style={shaky ? bad : ok} />
            <Line k="静止时 yaw 角速度 σ" v={(jitter.stillYawRateStd ?? "-") + " rad/s"} />
            <div style={{ ...dim, marginTop: 4, lineHeight: 1.6 }}>
              {jitter.stillSamples > 4
                ? shaky
                  ? "定位静止噪声偏大，跟随视角会放大成整幅地图抖动；可查 NDT/GNSS 融合参数或换顶视图观察。"
                  : "静止噪声正常，画面抖动更像渲染帧率或跟随视角引起。"
                : "静止样本不足：让小车停一会儿再检测。"}
            </div>
          </div>
        </>
      )}
    </div>
  );
}
