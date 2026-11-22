import { useState, useEffect, useMemo, useCallback } from "react";

/* ══════════════════════════════════════════════════════════════════════
 * 巡检运营报表（独立页面，与监控界面 App.jsx 互不依赖）
 *
 * 数据来源：loadReport(range)
 *   - 目前 REPORT_API_URL = null → 使用模拟数据（页面顶部标注“示例数据”）
 *   - 工控机“运营记录”服务做好后，把 REPORT_API_URL 改成接口地址即可，
 *     接口返回格式与 makeMockReport() 的返回值一致，页面无需改动。
 * ══════════════════════════════════════════════════════════════════════ */
const REPORT_API_URL = null; // 例如 "http://10.245.188.140:8600/api/report"

// ── 巡检线路与点位配置（按现场修改） ──
const ROUTE_LENGTH_KM = 1.18; // 一圈巡检路线长度
const CHECKPOINTS = [
  { id: "cp1", name: "东门岗亭" },
  { id: "cp2", name: "2 栋配电房" },
  { id: "cp3", name: "消防通道 A" },
  { id: "cp4", name: "3 栋仓库门口" },
  { id: "cp5", name: "4 栋北侧停车场" },
  { id: "cp6", name: "充电区" },
];

// ── 价值换算（报表中可直接修改，数值会记住） ──
const DEFAULT_VALUE_PARAMS = { wage: 35, factor: 1.0 }; // 人工时薪（元/时）、1 小时自动巡检折合人工小时

// ── 电量估算模型（没有电量采集时，按运行功耗推算） ──
const DEFAULT_BATTERY_PARAMS = {
  capacityKwh: 2.4, // 电池可用容量（48 V × 50 Ah ≈ 2.4 kWh）
  idleW: 150,       // 静止时功耗：工控机 + 激光雷达 + 相机等
  driveW: 230,      // 行驶时额外功耗
};
const BATTERY_UNCERTAINTY = 0.15; // 估算误差按已耗电量的 ±15% 显示

const RANGES = [
  { id: "today", label: "今日", days: 1 },
  { id: "7d", label: "近 7 天", days: 7 },
  { id: "30d", label: "近 30 天", days: 30 },
];

const EVENT_TYPES = {
  anomaly: { label: "点位异常", tone: "critical", icon: "!" },
  mrm: { label: "紧急停车", tone: "critical", icon: "■" },
  takeover: { label: "人工接管", tone: "warning", icon: "✋" },
  obstacle: { label: "避障停车", tone: "info", icon: "◆" },
  missed: { label: "点位漏检", tone: "serious", icon: "○" },
};

/* ───────────── 数据读取 ───────────── */
async function loadReport(now) {
  if (REPORT_API_URL) {
    const res = await fetch(`${REPORT_API_URL}?days=60`);
    if (!res.ok) throw new Error(`报表接口返回 ${res.status}`);
    return { ...(await res.json()), mock: false };
  }
  return makeMockReport(now);
}

// 可复现的伪随机数（同一天打开看到的示例数据一致）
function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const dayKey = (d) => `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, "0")}-${String(d.getDate()).padStart(2, "0")}`;
const ROUND_HOURS = [1, 3, 8, 10, 13, 15, 17, 19, 21, 23]; // 示例排班：含夜间巡检，每天整点出发

function makeMockReport(now) {
  const days = [];
  const events = [];
  const cpStats = Object.fromEntries(CHECKPOINTS.map((c) => [c.id, { done: 0, planned: 0, anomalies: 0, missed: 0, last: null }]));
  const heat = []; // 每天 24 小时的运行分钟数
  const hourlyToday = Array.from({ length: 24 }, () => 0);
  const nowHour = now.getHours() + now.getMinutes() / 60;

  for (let back = 59; back >= 0; back--) {
    const d = new Date(now.getFullYear(), now.getMonth(), now.getDate() - back);
    const r = rng(d.getFullYear() * 1000 + d.getMonth() * 40 + d.getDate());
    const isToday = back === 0;
    const hours = ROUND_HOURS.filter(() => r() > 0.08).filter((h) => !isToday || h + 0.3 <= nowHour);
    const heatRow = Array.from({ length: 24 }, () => 0);
    let km = 0;
    let autoKm = 0;
    let takeovers = 0;
    let mrm = 0;
    let obstacle = 0;
    let checkins = 0;

    hours.forEach((h) => {
      const roundKm = ROUTE_LENGTH_KM * (0.97 + r() * 0.06);
      const autoRate = r() > 0.12 ? 1 : 0.8 + r() * 0.15;
      km += roundKm;
      autoKm += roundKm * autoRate;
      const minutes = (roundKm / 4.3) * 60 + CHECKPOINTS.length * 0.5;
      heatRow[h] += Math.min(60, minutes);
      if (isToday) hourlyToday[h] += roundKm;
      const t0 = new Date(d.getFullYear(), d.getMonth(), d.getDate(), h, Math.floor(r() * 6));

      CHECKPOINTS.forEach((cp, i) => {
        const st = cpStats[cp.id];
        const at = new Date(t0.getTime() + (i + 1) * (minutes / CHECKPOINTS.length) * 60000);
        if (back < 30) st.planned++;
        if (r() < 0.012) {
          if (back < 30) st.missed++;
          events.push({ at, type: "missed", place: cp.name, detail: "未在规定时间到达点位" });
          return;
        }
        checkins++;
        if (back < 30) {
          st.done++;
          st.last = at;
        }
        const anomalyP = cp.id === "cp3" ? 0.05 : 0.008;
        if (r() < anomalyP) {
          if (back < 30) st.anomalies++;
          events.push({
            at, type: "anomaly", place: cp.name,
            detail: cp.id === "cp3" ? "通道内检测到占用物，已拍照留存" : "点位状态与基准不一致，已拍照留存",
          });
        }
      });
      if (autoRate < 1) {
        takeovers++;
        events.push({ at: new Date(t0.getTime() + r() * 14 * 60000), type: "takeover", place: CHECKPOINTS[Math.floor(r() * 5)].name + "附近", detail: "操作员接管后手动通过" });
      }
      const obs = Math.floor(r() * 3);
      obstacle += obs;
      for (let k = 0; k < obs; k++) {
        if (r() < 0.35) events.push({ at: new Date(t0.getTime() + r() * 14 * 60000), type: "obstacle", place: CHECKPOINTS[Math.floor(r() * 6)].name + "附近", detail: `前方障碍物，停车等待 ${Math.round(4 + r() * 20)} 秒后继续` });
      }
      if (r() < 0.03) {
        mrm++;
        events.push({ at: new Date(t0.getTime() + r() * 14 * 60000), type: "mrm", place: "风华路路口", detail: "定位置信度下降触发紧急停车，38 秒后恢复" });
      }
    });

    const autoHours = (autoKm / 4.3) + (hours.length * CHECKPOINTS.length * 0.5) / 60;
    days.push({
      date: dayKey(d),
      km, autoKm, autoHours,
      rounds: hours.length,
      checkins, plannedCheckins: hours.length * CHECKPOINTS.length,
      takeovers, mrm, obstacle,
      onlineRate: 0.975 + r() * 0.024,
    });
    heat.push({ date: dayKey(d), minutes: heatRow });
  }

  // 电量估算用的今日运行时间线（每 5 分钟一个样本：是否在行驶）
  const timeline = [];
  for (let t = 0; t <= Math.min(nowHour, 23.95); t += 5 / 60) {
    const moving = ROUND_HOURS.some((h) => hourlyToday[h] > 0 && t >= h && t < h + 0.3);
    timeline.push({ hour: t, moving });
  }
  // 示例：车辆在两轮巡检之间回到充电区停靠超过 30 分钟，推断为充电
  const chargeWindows = [
    { from: 4.5, to: 6.2, note: "停靠充电区 102 分钟" },
    { from: 11.3, to: 12.6, note: "停靠充电区 78 分钟" },
    { from: 17.5, to: 18.7, note: "停靠充电区 72 分钟" },
  ]
    .filter((w) => w.from < nowHour)
    .map((w) => ({ ...w, to: Math.min(w.to, nowHour) }));

  events.sort((a, b) => b.at - a.at);
  return { days, heat, hourlyToday, events, checkpoints: cpStats, battery: { timeline, chargeWindows }, mock: true, generatedAt: now };
}

/* ───────────── 计算工具 ───────────── */
const sum = (arr, k) => arr.reduce((s, x) => s + (k ? x[k] : x), 0);
const fmt1 = (v) => (Number.isFinite(v) ? v.toFixed(1) : "--");
const fmtInt = (v) => (Number.isFinite(v) ? Math.round(v).toLocaleString("zh-CN") : "--");
const pad2 = (n) => String(n).padStart(2, "0");
const fmtTime = (d) => `${pad2(d.getMonth() + 1)}-${pad2(d.getDate())} ${pad2(d.getHours())}:${pad2(d.getMinutes())}`;
const fmtHour = (h) => `${pad2(Math.floor(h))}:${pad2(Math.round((h % 1) * 60) % 60)}`;

function summarize(days) {
  const km = sum(days, "km");
  const autoKm = sum(days, "autoKm");
  const plannedCheckins = sum(days, "plannedCheckins");
  return {
    km, autoKm,
    autoHours: sum(days, "autoHours"),
    rounds: sum(days, "rounds"),
    checkins: sum(days, "checkins"),
    plannedCheckins,
    takeovers: sum(days, "takeovers"),
    mrm: sum(days, "mrm"),
    obstacle: sum(days, "obstacle"),
    autoRate: km > 0 ? autoKm / km : null,
    onlineRate: days.length ? sum(days, "onlineRate") / days.length : null,
  };
}

function delta(cur, prev) {
  if (!Number.isFinite(cur) || !Number.isFinite(prev) || prev === 0) return null;
  return (cur - prev) / prev;
}

function niceMax(v) {
  if (!(v > 0)) return 1;
  const p = Math.pow(10, Math.floor(Math.log10(v)));
  const n = v / p;
  const step = n <= 1 ? 1 : n <= 2 ? 2 : n <= 2.5 ? 2.5 : n <= 5 ? 5 : 10;
  return step * p;
}

// 电量估算：按运行时间线积分功耗；推断的充电窗口内视为充满
function estimateBattery(timeline, chargeWindows, p) {
  const out = [];
  let used = 0; // 自上次充满以来的耗电（Wh）
  const cap = p.capacityKwh * 1000;
  let totalWh = 0;
  timeline.forEach((s, i) => {
    const dtH = i === 0 ? 0 : s.hour - timeline[i - 1].hour;
    const charging = chargeWindows.some((w) => s.hour >= w.from && s.hour < w.to);
    if (charging) used = 0;
    else {
      const wh = (p.idleW + (s.moving ? p.driveW : 0)) * dtH;
      used += wh;
      totalWh += wh;
    }
    const soc = Math.max(0, 100 - (used / cap) * 100);
    const err = (used / cap) * 100 * BATTERY_UNCERTAINTY;
    out.push({ hour: s.hour, soc, lo: Math.max(0, soc - err), hi: Math.min(100, soc + err), charging });
  });
  const last = out[out.length - 1];
  const avgW = p.idleW + p.driveW * 0.35; // 按巡检排班约 35% 时间在行驶
  const remainH = last ? ((last.soc / 100) * cap) / avgW : null;
  return { series: out, last, totalKwh: totalWh / 1000, remainH };
}

/* ───────────── 样式 ───────────── */
const CSS = `
.or-root{--bg:#060a14;--panel:#0b1322;--panel-2:#0f1a2d;--line:rgba(96,165,250,.14);--line-strong:rgba(63,212,255,.35);--grid:rgba(120,160,220,.12);
  --text:#e6f1ff;--sub:#93a7c3;--dim:#5b6b84;--accent:#3fd4ff;--accent-soft:rgba(63,212,255,.12);
  --good:#0ca30c;--warning:#fab219;--serious:#ec835a;--critical:#d03b3b;--info:#6f8fb5;
  color-scheme:dark;min-height:100vh;background:radial-gradient(1200px 600px at 15% -10%,#0d1a33 0%,var(--bg) 60%);color:var(--text);
  font-family:"PingFang SC","Microsoft YaHei","Noto Sans SC",system-ui,sans-serif;font-size:14px;}
.or-root *{box-sizing:border-box;}
.or-num{font-family:"DIN Alternate","Bahnschrift","Roboto Mono","SF Mono",ui-monospace,monospace;}
.or-tab{font-variant-numeric:tabular-nums;}
.or-head{position:sticky;top:0;z-index:20;display:flex;align-items:center;gap:14px;flex-wrap:wrap;padding:10px 20px;background:rgba(8,14,28,.92);border-bottom:1px solid var(--line);backdrop-filter:blur(8px);}
.or-brand{display:flex;flex-direction:column;line-height:1.2;}
.or-brand b{font-size:16px;letter-spacing:1px;}
.or-brand span{font-size:11px;color:var(--sub);letter-spacing:1px;}
.or-seg{display:flex;padding:3px;border-radius:9px;background:rgba(255,255,255,.04);border:1px solid var(--line);}
.or-seg button{border:0;background:transparent;color:var(--sub);font:inherit;font-size:13px;padding:5px 14px;border-radius:6px;cursor:pointer;}
.or-seg button.on{background:var(--accent-soft);color:#fff;box-shadow:inset 0 0 0 1px var(--line-strong);}
.or-seg button:focus-visible,.or-btn:focus-visible,.or-chipbtn:focus-visible{outline:2px solid var(--accent);outline-offset:2px;}
.or-mock{padding:3px 10px;border-radius:999px;font-size:12px;color:#ffe0ad;border:1px solid rgba(250,178,25,.4);background:rgba(250,178,25,.08);}
.or-head-right{margin-left:auto;display:flex;align-items:center;gap:10px;}
.or-btn{display:inline-flex;align-items:center;gap:6px;padding:6px 14px;border-radius:8px;border:1px solid var(--line);background:rgba(255,255,255,.04);color:var(--sub);font:inherit;font-size:13px;cursor:pointer;text-decoration:none;}
.or-btn:hover{color:#fff;border-color:var(--line-strong);}
.or-updated{font-size:12px;color:var(--dim);}
.or-main{max-width:1440px;margin:0 auto;padding:18px 20px 40px;display:flex;flex-direction:column;gap:16px;}
.or-lead{font-size:15px;color:var(--sub);line-height:1.6;}
.or-lead b{color:var(--text);font-weight:600;}
.or-kpis{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;}
.or-kpi{padding:14px 16px;border-radius:12px;background:var(--panel);border:1px solid var(--line);}
.or-kpi-label{font-size:12px;color:var(--sub);}
.or-kpi-val{margin-top:6px;font-size:30px;font-weight:700;line-height:1.1;}
.or-kpi-val small{font-size:13px;color:var(--sub);font-weight:500;margin-left:4px;}
.or-kpi-delta{margin-top:6px;font-size:12px;color:var(--dim);}
.or-kpi-delta .up{color:#7be08a;} .or-kpi-delta .down{color:#ff9aa4;}
.or-value{display:flex;align-items:center;gap:18px;flex-wrap:wrap;padding:14px 18px;border-radius:12px;background:linear-gradient(90deg,rgba(63,212,255,.10),rgba(52,245,164,.05));border:1px solid var(--line-strong);}
.or-value-main{font-size:18px;}
.or-value-main b{font-size:26px;margin:0 4px;}
.or-value-params{margin-left:auto;display:flex;gap:12px;flex-wrap:wrap;font-size:12px;color:var(--sub);}
.or-field{display:flex;align-items:center;gap:6px;}
.or-field input{width:70px;padding:4px 8px;border-radius:6px;border:1px solid var(--line);background:rgba(255,255,255,.04);color:var(--text);font:inherit;font-size:13px;}
.or-field input:focus{outline:none;border-color:var(--line-strong);}
.or-grid{display:grid;grid-template-columns:minmax(0,1.6fr) minmax(0,1fr);gap:16px;}
.or-panel{padding:14px 16px 16px;border-radius:12px;background:var(--panel);border:1px solid var(--line);min-width:0;}
.or-panel h2{margin:0;font-size:14px;font-weight:600;display:flex;align-items:center;gap:8px;}
.or-panel h2 small{font-size:12px;color:var(--dim);font-weight:400;}
.or-panel-head{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:10px;}
.or-panel-head .or-spacer{margin-left:auto;}
.or-svg{display:block;width:100%;height:auto;overflow:visible;}
.or-axis{fill:var(--dim);font-size:11px;}
.or-tip{position:fixed;z-index:50;pointer-events:none;padding:8px 10px;border-radius:8px;background:rgba(6,10,20,.95);border:1px solid var(--line-strong);font-size:12px;line-height:1.6;color:var(--text);box-shadow:0 8px 24px rgba(0,0,0,.4);max-width:260px;}
.or-tip b{font-weight:600;}
.or-tip .muted{color:var(--sub);}
.or-bat-stats{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-bottom:8px;}
.or-bat-stats div{padding:8px 10px;border-radius:8px;background:var(--panel-2);}
.or-bat-stats span{display:block;font-size:11px;color:var(--sub);}
.or-bat-stats b{font-size:18px;}
.or-note{font-size:12px;color:var(--dim);line-height:1.6;}
.or-params{display:flex;gap:10px;flex-wrap:wrap;margin-top:8px;font-size:12px;color:var(--sub);}
.or-table-wrap{overflow-x:auto;}
.or-table{width:100%;border-collapse:collapse;font-size:13px;}
.or-table th{text-align:left;font-weight:500;color:var(--dim);font-size:12px;padding:6px 8px;border-bottom:1px solid var(--line);white-space:nowrap;}
.or-table td{padding:8px;border-bottom:1px solid rgba(96,165,250,.07);white-space:nowrap;}
.or-table td.num{text-align:right;}
.or-table th.num{text-align:right;}
.or-pill{display:inline-flex;align-items:center;gap:5px;padding:2px 8px;border-radius:999px;font-size:12px;border:1px solid;}
.or-pill i{font-style:normal;font-size:10px;width:14px;height:14px;display:inline-flex;align-items:center;justify-content:center;border-radius:50%;color:#061018;}
.or-pill.good{color:#9fe8a6;border-color:rgba(12,163,12,.45);} .or-pill.good i{background:var(--good);}
.or-pill.warning{color:#ffe0ad;border-color:rgba(250,178,25,.45);} .or-pill.warning i{background:var(--warning);}
.or-pill.serious{color:#ffc9b3;border-color:rgba(236,131,90,.45);} .or-pill.serious i{background:var(--serious);}
.or-pill.critical{color:#ffb8b8;border-color:rgba(208,59,59,.55);} .or-pill.critical i{background:var(--critical);color:#fff;}
.or-pill.info{color:#c6d6ea;border-color:rgba(111,143,181,.45);} .or-pill.info i{background:var(--info);}
.or-bar-track{position:relative;height:6px;border-radius:3px;background:rgba(63,212,255,.12);min-width:80px;}
.or-bar-fill{position:absolute;left:0;top:0;bottom:0;border-radius:3px;background:var(--accent);}
.or-safety{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;}
.or-safety div{padding:12px;border-radius:10px;background:var(--panel-2);}
.or-safety span{display:block;font-size:12px;color:var(--sub);}
.or-safety b{display:block;margin-top:4px;font-size:22px;}
.or-safety em{font-style:normal;font-size:11px;color:var(--dim);}
.or-chips{display:flex;gap:6px;flex-wrap:wrap;}
.or-chipbtn{padding:3px 10px;border-radius:999px;border:1px solid var(--line);background:transparent;color:var(--sub);font:inherit;font-size:12px;cursor:pointer;}
.or-chipbtn.on{color:#fff;border-color:var(--line-strong);background:var(--accent-soft);}
.or-legend{display:flex;align-items:center;gap:6px;font-size:11px;color:var(--dim);}
.or-legend-bar{width:120px;height:8px;border-radius:4px;}
.or-defs{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:6px 24px;font-size:12px;color:var(--sub);line-height:1.6;}
.or-defs b{color:var(--text);font-weight:500;}
.or-empty{padding:24px;text-align:center;color:var(--dim);}
@media (max-width:1000px){.or-kpis{grid-template-columns:repeat(2,minmax(0,1fr));}.or-grid{grid-template-columns:minmax(0,1fr);}.or-defs{grid-template-columns:minmax(0,1fr);}}
@media (max-width:520px){.or-kpis{grid-template-columns:minmax(0,1fr);}.or-head{padding:10px 16px;}.or-main{padding:14px 16px 32px;}}
@media (prefers-reduced-motion:reduce){.or-root *{transition:none!important;}}
`;

/* ───────────── 图表组件 ───────────── */
function Tooltip({ tip }) {
  if (!tip) return null;
  const x = Math.min(tip.x + 14, window.innerWidth - 270);
  return (
    <div className="or-tip" style={{ left: x, top: tip.y + 14 }}>
      {tip.lines.map((l, i) => (
        <div key={i} className={i === 0 ? "" : "muted"}>{i === 0 ? <b>{l}</b> : l}</div>
      ))}
    </div>
  );
}

// 单系列柱状图：柱宽不超过 24px，圆角只在柱顶
function BarChart({ data, unit, onTip }) {
  const W = 760;
  const H = 230;
  const m = { l: 40, r: 8, t: 12, b: 26 };
  const iw = W - m.l - m.r;
  const ih = H - m.t - m.b;
  const max = niceMax(Math.max(...data.map((d) => d.value), 0));
  const ticks = [0, 0.25, 0.5, 0.75, 1].map((f) => f * max);
  const band = iw / data.length;
  const bw = Math.min(24, band * 0.62);
  const labelEvery = Math.ceil(data.length / 12);
  return (
    <svg className="or-svg" viewBox={`0 0 ${W} ${H}`} role="img" aria-label={`里程柱状图，单位 ${unit}`}>
      {ticks.map((t) => {
        const y = m.t + ih - (t / max) * ih;
        return (
          <g key={t}>
            <line x1={m.l} x2={W - m.r} y1={y} y2={y} stroke="var(--grid)" strokeWidth="1" />
            <text className="or-axis or-tab" x={m.l - 6} y={y + 4} textAnchor="end">{Number.isInteger(t) ? t : t.toFixed(1)}</text>
          </g>
        );
      })}
      {data.map((d, i) => {
        const h = max > 0 ? (d.value / max) * ih : 0;
        const x = m.l + band * i + (band - bw) / 2;
        const y = m.t + ih - h;
        const r = Math.min(4, h);
        const path = h > 0
          ? `M${x},${m.t + ih} L${x},${y + r} Q${x},${y} ${x + r},${y} L${x + bw - r},${y} Q${x + bw},${y} ${x + bw},${y + r} L${x + bw},${m.t + ih} Z`
          : "";
        return (
          <g key={d.label}
            onMouseMove={(e) => onTip({ x: e.clientX, y: e.clientY, lines: d.tip })}
            onMouseLeave={() => onTip(null)}>
            <rect x={m.l + band * i} y={m.t} width={band} height={ih} fill="transparent" />
            {path && <path d={path} fill={d.highlight ? "var(--accent)" : "rgba(63,212,255,.55)"} />}
            {i % labelEvery === 0 && (
              <text className="or-axis or-tab" x={m.l + band * i + band / 2} y={H - 8} textAnchor="middle">{d.label}</text>
            )}
          </g>
        );
      })}
      <line x1={m.l} x2={W - m.r} y1={m.t + ih} y2={m.t + ih} stroke="var(--line)" strokeWidth="1" />
    </svg>
  );
}

// 电量估算曲线：线 + 误差带 + 充电窗口 + 悬停十字线
function BatteryChart({ est, chargeWindows, onTip }) {
  const W = 520;
  const H = 200;
  const m = { l: 36, r: 12, t: 10, b: 24 };
  const iw = W - m.l - m.r;
  const ih = H - m.t - m.b;
  const s = est.series;
  const [hover, setHover] = useState(null);
  if (s.length < 2) return <div className="or-empty">今日数据不足，稍后查看</div>;
  const h0 = 0;
  const h1 = Math.max(s[s.length - 1].hour, h0 + 1);
  const X = (h) => m.l + ((h - h0) / (h1 - h0)) * iw;
  const Y = (v) => m.t + ih - (v / 100) * ih;
  const line = s.map((p, i) => `${i ? "L" : "M"}${X(p.hour).toFixed(1)},${Y(p.soc).toFixed(1)}`).join("");
  const band = s.map((p, i) => `${i ? "L" : "M"}${X(p.hour).toFixed(1)},${Y(p.hi).toFixed(1)}`).join("")
    + s.slice().reverse().map((p) => `L${X(p.hour).toFixed(1)},${Y(p.lo).toFixed(1)}`).join("") + "Z";
  const last = s[s.length - 1];
  const xTicks = [];
  const tickStep = h1 - h0 > 12 ? 4 : h1 - h0 > 6 ? 2 : 1;
  for (let h = Math.ceil(h0); h <= h1; h += tickStep) xTicks.push(h);
  const onMove = (e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const hx = h0 + (((e.clientX - rect.left) / rect.width) * W - m.l) / iw * (h1 - h0);
    let best = s[0];
    s.forEach((p) => { if (Math.abs(p.hour - hx) < Math.abs(best.hour - hx)) best = p; });
    setHover(best);
    onTip({ x: e.clientX, y: e.clientY, lines: [
      `${fmtHour(best.hour)}`,
      `估算电量 ${best.soc.toFixed(0)}%（${best.lo.toFixed(0)}–${best.hi.toFixed(0)}%）`,
      best.charging ? "推断为充电中" : "",
    ].filter(Boolean) });
  };
  return (
    <svg className="or-svg" viewBox={`0 0 ${W} ${H}`} role="img" aria-label="今日电量估算曲线"
      onMouseMove={onMove} onMouseLeave={() => { setHover(null); onTip(null); }}>
      {[0, 25, 50, 75, 100].map((v) => (
        <g key={v}>
          <line x1={m.l} x2={W - m.r} y1={Y(v)} y2={Y(v)} stroke="var(--grid)" strokeWidth="1" />
          <text className="or-axis or-tab" x={m.l - 6} y={Y(v) + 4} textAnchor="end">{v}%</text>
        </g>
      ))}
      <rect x={m.l} y={Y(20)} width={iw} height={Y(0) - Y(20)} fill="rgba(208,59,59,.08)" />
      <text className="or-axis" x={W - m.r - 4} y={Y(20) - 4} textAnchor="end">低电量 20%</text>
      {chargeWindows.map((w) => (
        <g key={w.from}>
          <rect x={X(w.from)} y={m.t} width={Math.max(2, X(w.to) - X(w.from))} height={ih} fill="rgba(12,163,12,.10)" />
          <text className="or-axis" x={X(w.from) + 4} y={m.t + 12}>充电</text>
        </g>
      ))}
      <path d={band} fill="rgba(63,212,255,.12)" />
      <path d={line} fill="none" stroke="var(--accent)" strokeWidth="2" strokeLinejoin="round" strokeLinecap="round" />
      <circle cx={X(last.hour)} cy={Y(last.soc)} r="4.5" fill="var(--accent)" stroke="var(--panel)" strokeWidth="2" />
      <text className="or-tab" x={X(last.hour) - 8} y={Y(last.soc) - 10} textAnchor="end" fill="var(--text)" fontSize="12">{last.soc.toFixed(0)}%</text>
      {hover && <line x1={X(hover.hour)} x2={X(hover.hour)} y1={m.t} y2={m.t + ih} stroke="var(--sub)" strokeWidth="1" />}
      {xTicks.map((h) => (
        <text key={h} className="or-axis or-tab" x={X(h)} y={H - 6} textAnchor="middle">{pad2(h)}:00</text>
      ))}
    </svg>
  );
}

// 单色顺序色阶：深 → 亮（运行分钟数越多越亮）
function heatColor(f) {
  if (f <= 0) return "rgba(120,160,220,.06)";
  const a = [16, 42, 68];
  const b = [63, 212, 255];
  const t = 0.18 + 0.82 * f;
  return `rgb(${a.map((v, i) => Math.round(v + (b[i] - v) * t)).join(",")})`;
}

function Heatmap({ rows, onTip }) {
  const W = 1360;
  const cell = 20;
  const labelW = 84;
  const H = rows.length * (cell + 3) + 22;
  const cw = (W - labelW) / 24;
  return (
    <svg className="or-svg" viewBox={`0 0 ${W} ${H}`} style={{ maxHeight: 230 }} preserveAspectRatio="xMinYMin meet" role="img" aria-label="近 7 天运行时段分布">
      {rows.map((row, ri) => (
        <g key={row.date}>
          <text className="or-axis or-tab" x={labelW - 8} y={ri * (cell + 3) + cell / 2 + 4} textAnchor="end">{row.label}</text>
          {row.minutes.map((mins, h) => (
            <rect key={h}
              x={labelW + h * cw + 1} y={ri * (cell + 3)} width={cw - 2} height={cell} rx="3"
              fill={heatColor(mins / 60)}
              onMouseMove={(e) => onTip({ x: e.clientX, y: e.clientY, lines: [`${row.label} ${pad2(h)}:00–${pad2(h + 1)}:00`, mins > 0 ? `运行 ${Math.round(mins)} 分钟` : "未运行"] })}
              onMouseLeave={() => onTip(null)} />
          ))}
        </g>
      ))}
      {Array.from({ length: 24 }, (_, h) => h).filter((h) => h % 2 === 0).map((h) => (
        <text key={h} className="or-axis or-tab" x={labelW + h * cw + cw / 2} y={H - 4} textAnchor="middle">{pad2(h)}</text>
      ))}
    </svg>
  );
}

function Pill({ tone, icon, children }) {
  return <span className={`or-pill ${tone}`}><i aria-hidden="true">{icon}</i>{children}</span>;
}

function Delta({ value, goodWhenUp = true, label }) {
  if (value == null) return <div className="or-kpi-delta">{label}：暂无对比</div>;
  const up = value >= 0;
  const good = up === goodWhenUp;
  return (
    <div className="or-kpi-delta">
      较{label} <span className={good ? "up" : "down"}>{up ? "↑" : "↓"} {Math.abs(value * 100).toFixed(0)}%</span>
    </div>
  );
}

function loadParams(key, fallback) {
  try {
    const v = JSON.parse(localStorage.getItem(key) || "null");
    return v && typeof v === "object" ? { ...fallback, ...v } : fallback;
  } catch {
    return fallback;
  }
}
function saveParams(key, v) {
  try { localStorage.setItem(key, JSON.stringify(v)); } catch { /* 忽略 */ }
}

/* ───────────── 页面 ───────────── */
export default function OpsReport({ onBack }) {
  const [rangeId, setRangeId] = useState(() => {
    try { return localStorage.getItem("or_range") || "7d"; } catch { return "7d"; }
  });
  const [report, setReport] = useState(null);
  const [error, setError] = useState(null);
  const [tip, setTip] = useState(null);
  const [valueParams, setValueParams] = useState(() => loadParams("or_value", DEFAULT_VALUE_PARAMS));
  const [batParams, setBatParams] = useState(() => loadParams("or_battery", DEFAULT_BATTERY_PARAMS));
  const [eventFilter, setEventFilter] = useState("all");

  const refresh = useCallback(() => {
    setError(null);
    loadReport(new Date()).then(setReport).catch((e) => setError(e.message || String(e)));
  }, []);
  useEffect(() => { refresh(); }, [refresh]);
  useEffect(() => { try { localStorage.setItem("or_range", rangeId); } catch { /* 忽略 */ } }, [rangeId]);
  useEffect(() => { saveParams("or_value", valueParams); }, [valueParams]);
  useEffect(() => { saveParams("or_battery", batParams); }, [batParams]);
  useEffect(() => { document.title = "巡检运营报表"; }, []);

  const range = RANGES.find((r) => r.id === rangeId) || RANGES[1];

  const view = useMemo(() => {
    if (!report) return null;
    const all = report.days;
    const cur = all.slice(-range.days);
    const prev = all.slice(-range.days * 2, -range.days);
    const s = summarize(cur);
    const p = summarize(prev);
    const fromDate = new Date(cur[0].date + "T00:00:00");
    const events = report.events.filter((e) => e.at >= fromDate);
    return { cur, s, p, events };
  }, [report, range.days]);

  const battery = useMemo(
    () => (report ? estimateBattery(report.battery.timeline, report.battery.chargeWindows, batParams) : null),
    [report, batParams]
  );

  const goBack = () => {
    if (onBack) onBack();
    else window.location.hash = "#/";
  };

  if (error) {
    return (
      <div className="or-root"><style>{CSS}</style>
        <div className="or-main"><div className="or-panel or-empty">报表数据读取失败：{error} <button className="or-btn" onClick={refresh}>重试</button></div></div>
      </div>
    );
  }
  if (!view) {
    return <div className="or-root"><style>{CSS}</style><div className="or-main"><div className="or-empty">正在读取运营数据…</div></div></div>;
  }

  const { cur, s, p, events } = view;
  const prevLabel = range.id === "today" ? "昨日" : "上一周期";
  const savedHours = s.autoHours * valueParams.factor;
  const savedMoney = savedHours * valueParams.wage;
  const checkRate = s.plannedCheckins ? s.checkins / s.plannedCheckins : null;
  const anomalies = events.filter((e) => e.type === "anomaly").length;

  // 里程图：今日按小时，其余按天
  const barData = range.id === "today"
    ? report.hourlyToday.map((km, h) => ({
      label: `${pad2(h)}`, value: km, highlight: h === new Date().getHours(),
      tip: [`${pad2(h)}:00–${pad2(h + 1)}:00`, `巡检里程 ${km.toFixed(2)} km`],
    }))
    : cur.map((d, i) => ({
      label: d.date.slice(5), value: d.km, highlight: i === cur.length - 1,
      tip: [d.date, `巡检里程 ${d.km.toFixed(2)} km`, `自动巡检 ${d.autoHours.toFixed(1)} 小时 · ${d.rounds} 轮`, `点位打卡 ${d.checkins}/${d.plannedCheckins}`],
    }));

  const heatRows = report.heat.slice(-7).map((h) => {
    const d = new Date(h.date + "T00:00:00");
    const wk = "日一二三四五六"[d.getDay()];
    return { ...h, label: `${h.date.slice(5)} 周${wk}` };
  });

  const cpRows = CHECKPOINTS.map((c) => {
    const st = report.checkpoints[c.id];
    const recent = events.filter((e) => e.place === c.name && (e.type === "anomaly" || e.type === "missed"));
    const status = recent.some((e) => e.type === "anomaly") ? "anomaly" : recent.some((e) => e.type === "missed") ? "missed" : "ok";
    return { ...c, ...st, status, rate: st.planned ? st.done / st.planned : null };
  });

  const shownEvents = events.filter((e) => eventFilter === "all" || e.type === eventFilter).slice(0, 40);

  return (
    <div className="or-root">
      <style>{CSS}</style>

      <header className="or-head">
        <div className="or-brand">
          <b>巡检运营报表</b>
          <span>南京核星力 · 孵化园 1 号巡检车</span>
        </div>
        <div className="or-seg" role="tablist" aria-label="统计周期">
          {RANGES.map((r) => (
            <button key={r.id} id={`or-range-${r.id}`} role="tab" aria-selected={rangeId === r.id}
              className={rangeId === r.id ? "on" : ""} onClick={() => setRangeId(r.id)}>{r.label}</button>
          ))}
        </div>
        {report.mock && <span className="or-mock" title="接入工控机运营记录服务后显示真实数据">示例数据</span>}
        <div className="or-head-right">
          <span className="or-updated or-tab">更新于 {fmtTime(report.generatedAt instanceof Date ? report.generatedAt : new Date(report.generatedAt))}</span>
          <button className="or-btn" onClick={refresh}>刷新</button>
          <button className="or-btn" onClick={goBack}>← 返回监控</button>
        </div>
      </header>

      <main className="or-main">
        <p className="or-lead">
          {range.label}共完成 <b className="or-num">{s.rounds}</b> 轮巡检，行驶 <b className="or-num">{fmt1(s.km)} km</b>，
          点位打卡 <b className="or-num">{s.checkins}</b> 次，发现 <b className="or-num">{anomalies}</b> 处异常；
          人工接管 <b className="or-num">{s.takeovers}</b> 次，紧急停车 <b className="or-num">{s.mrm}</b> 次。
        </p>

        <section className="or-kpis" aria-label="核心指标">
          <div className="or-kpi">
            <div className="or-kpi-label">巡检里程</div>
            <div className="or-kpi-val or-num">{fmt1(s.km)}<small>km</small></div>
            <Delta value={delta(s.km, p.km)} label={prevLabel} />
          </div>
          <div className="or-kpi">
            <div className="or-kpi-label">自动巡检时长</div>
            <div className="or-kpi-val or-num">{fmt1(s.autoHours)}<small>小时</small></div>
            <Delta value={delta(s.autoHours, p.autoHours)} label={prevLabel} />
          </div>
          <div className="or-kpi">
            <div className="or-kpi-label">点位打卡完成率</div>
            <div className="or-kpi-val or-num">{checkRate != null ? (checkRate * 100).toFixed(1) : "--"}<small>%</small></div>
            <div className="or-kpi-delta or-tab">{s.checkins} / {s.plannedCheckins} 次</div>
          </div>
          <div className="or-kpi">
            <div className="or-kpi-label">自动驾驶率</div>
            <div className="or-kpi-val or-num">{s.autoRate != null ? (s.autoRate * 100).toFixed(1) : "--"}<small>%</small></div>
            <div className="or-kpi-delta">自动行驶 {fmt1(s.autoKm)} km</div>
          </div>
        </section>

        <section className="or-value" aria-label="价值换算">
          <div className="or-value-main">
            替代人工巡逻<b className="or-num">{fmt1(savedHours)}</b>小时 · 估算节省<b className="or-num">¥{fmtInt(savedMoney)}</b>
          </div>
          <div className="or-value-params">
            <label className="or-field" htmlFor="or-wage">人工时薪
              <input id="or-wage" type="number" min="0" step="1" value={valueParams.wage}
                onChange={(e) => setValueParams((v) => ({ ...v, wage: Number(e.target.value) || 0 }))} />元/时
            </label>
            <label className="or-field" htmlFor="or-factor">1 小时自动巡检折合
              <input id="or-factor" type="number" min="0" step="0.1" value={valueParams.factor}
                onChange={(e) => setValueParams((v) => ({ ...v, factor: Number(e.target.value) || 0 }))} />人工小时
            </label>
          </div>
        </section>

        <div className="or-grid">
          <section className="or-panel">
            <div className="or-panel-head">
              <h2>巡检里程 <small>{range.id === "today" ? "按小时 · km" : "按天 · km"}</small></h2>
            </div>
            <BarChart data={barData} unit="km" onTip={setTip} />
          </section>

          <section className="or-panel">
            <div className="or-panel-head">
              <h2>今日电量 <small>估算值</small></h2>
            </div>
            {battery && (
              <>
                <div className="or-bat-stats">
                  <div><span>当前估算</span><b className="or-num">{battery.last ? `${battery.last.soc.toFixed(0)}%` : "--"}</b></div>
                  <div><span>今日耗电</span><b className="or-num">{battery.totalKwh.toFixed(2)} kWh</b></div>
                  <div><span>预计可再运行</span><b className="or-num">{battery.remainH != null ? `${battery.remainH.toFixed(1)} h` : "--"}</b></div>
                </div>
                <BatteryChart est={battery} chargeWindows={report.battery.chargeWindows} onTip={setTip} />
                <div className="or-params">
                  <label className="or-field" htmlFor="or-cap">电池容量
                    <input id="or-cap" type="number" min="0.1" step="0.1" value={batParams.capacityKwh}
                      onChange={(e) => setBatParams((v) => ({ ...v, capacityKwh: Number(e.target.value) || 0.1 }))} />kWh
                  </label>
                  <label className="or-field" htmlFor="or-idle">静止功耗
                    <input id="or-idle" type="number" min="0" step="10" value={batParams.idleW}
                      onChange={(e) => setBatParams((v) => ({ ...v, idleW: Number(e.target.value) || 0 }))} />W
                  </label>
                  <label className="or-field" htmlFor="or-drive">行驶额外功耗
                    <input id="or-drive" type="number" min="0" step="10" value={batParams.driveW}
                      onChange={(e) => setBatParams((v) => ({ ...v, driveW: Number(e.target.value) || 0 }))} />W
                  </label>
                </div>
                <p className="or-note">
                  底盘未上报电量，按“静止功耗 + 行驶功耗 × 行驶时长”推算，阴影为 ±{Math.round(BATTERY_UNCERTAINTY * 100)}% 误差范围。
                  车辆在充电区停靠超过 30 分钟视为充满。
                  {report.battery.chargeWindows.length > 0 && ` 今日推断充电：${report.battery.chargeWindows.map((w) => `${fmtHour(w.from)}（${w.note}）`).join("、")}。`}
                </p>
              </>
            )}
          </section>
        </div>

        <div className="or-grid">
          <section className="or-panel">
            <div className="or-panel-head">
              <h2>巡检点位 <small>近 30 天</small></h2>
            </div>
            <div className="or-table-wrap">
              <table className="or-table">
                <thead>
                  <tr><th>点位</th><th>当前状态</th><th className="num">打卡</th><th>完成率</th><th className="num">异常</th><th>最近打卡</th></tr>
                </thead>
                <tbody>
                  {cpRows.map((c) => (
                    <tr key={c.id}>
                      <td>{c.name}</td>
                      <td>
                        {c.status === "anomaly" ? <Pill tone="critical" icon="!">有异常</Pill>
                          : c.status === "missed" ? <Pill tone="serious" icon="○">有漏检</Pill>
                            : <Pill tone="good" icon="✓">正常</Pill>}
                      </td>
                      <td className="num or-tab">{c.done}/{c.planned}</td>
                      <td>
                        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                          <div className="or-bar-track"><div className="or-bar-fill" style={{ width: `${(c.rate || 0) * 100}%` }} /></div>
                          <span className="or-tab" style={{ fontSize: 12, color: "var(--sub)" }}>{c.rate != null ? `${(c.rate * 100).toFixed(1)}%` : "--"}</span>
                        </div>
                      </td>
                      <td className="num or-tab">{c.anomalies}</td>
                      <td className="or-tab" style={{ color: "var(--sub)" }}>{c.last ? fmtTime(c.last) : "--"}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </section>

          <section className="or-panel">
            <div className="or-panel-head">
              <h2>安全与稳定 <small>{range.label}</small></h2>
            </div>
            <div className="or-safety">
              <div><span>系统在线率</span><b className="or-num">{s.onlineRate != null ? (s.onlineRate * 100).toFixed(1) : "--"}%</b><em>通信、定位、规划均正常的时间占比</em></div>
              <div><span>人工接管</span><b className="or-num">{s.takeovers} 次</b><em>平均每 {s.takeovers ? fmt1(s.autoKm / s.takeovers) : "--"} km 一次</em></div>
              <div><span>紧急停车</span><b className="or-num">{s.mrm} 次</b><em>Autoware 触发的最小风险停车</em></div>
              <div><span>避障停车</span><b className="or-num">{s.obstacle} 次</b><em>遇障碍物主动停车等待</em></div>
            </div>
          </section>
        </div>

        <section className="or-panel">
          <div className="or-panel-head">
            <h2>运行时段分布 <small>近 7 天 · 每格为 1 小时内的运行分钟数</small></h2>
            <div className="or-spacer or-legend">
              0 分钟
              <span className="or-legend-bar" style={{ background: `linear-gradient(90deg, ${heatColor(0.02)}, ${heatColor(1)})` }} />
              60 分钟
            </div>
          </div>
          <Heatmap rows={heatRows} onTip={setTip} />
        </section>

        <section className="or-panel">
          <div className="or-panel-head">
            <h2>事件记录 <small>{range.label} · 共 {events.length} 条</small></h2>
            <div className="or-spacer or-chips" role="group" aria-label="事件类型筛选">
              <button className={`or-chipbtn ${eventFilter === "all" ? "on" : ""}`} onClick={() => setEventFilter("all")}>全部</button>
              {Object.entries(EVENT_TYPES).map(([k, t]) => (
                <button key={k} className={`or-chipbtn ${eventFilter === k ? "on" : ""}`} onClick={() => setEventFilter(k)}>
                  {t.label} {events.filter((e) => e.type === k).length}
                </button>
              ))}
            </div>
          </div>
          {shownEvents.length ? (
            <div className="or-table-wrap">
              <table className="or-table">
                <thead><tr><th>时间</th><th>类型</th><th>地点</th><th>说明</th></tr></thead>
                <tbody>
                  {shownEvents.map((e, i) => {
                    const t = EVENT_TYPES[e.type];
                    return (
                      <tr key={i}>
                        <td className="or-tab" style={{ color: "var(--sub)" }}>{fmtTime(e.at)}</td>
                        <td><Pill tone={t.tone} icon={t.icon}>{t.label}</Pill></td>
                        <td>{e.place}</td>
                        <td style={{ color: "var(--sub)", whiteSpace: "normal" }}>{e.detail}</td>
                      </tr>
                    );
                  })}
                </tbody>
              </table>
            </div>
          ) : (
            <div className="or-empty">这段时间没有此类事件</div>
          )}
        </section>

        <section className="or-panel" aria-label="指标说明">
          <div className="or-panel-head"><h2>指标说明</h2></div>
          <div className="or-defs">
            <div><b>巡检里程</b>：定位位置逐段累加，静止抖动与定位跳变不计入。</div>
            <div><b>自动巡检时长</b>：驾驶模式为“自动驾驶”的累计时间。</div>
            <div><b>点位打卡</b>：车辆到达点位 3 米内并停留，记为打卡一次；完成率 = 实际打卡 ÷ 计划打卡。</div>
            <div><b>自动驾驶率</b>：自动驾驶里程 ÷ 总里程。</div>
            <div><b>人工接管</b>：未到达终点时自动驾驶被切换为人工控制。</div>
            <div><b>紧急停车</b>：Autoware 触发最小风险停车（MRM）的次数。</div>
            <div><b>替代人工 / 节省费用</b>：自动巡检时长 × 折合系数 × 人工时薪，为估算值，参数可在上方修改。</div>
            <div><b>电量</b>：按功耗模型推算的估算值，接入电量采集后替换为实测。</div>
          </div>
        </section>
      </main>

      <Tooltip tip={tip} />
    </div>
  );
}
