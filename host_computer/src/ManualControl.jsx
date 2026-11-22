import { useEffect, useRef, useState } from "react";

// 速度档位：线速度 m/s，角速度 rad/s（网页遥控限速，实车请先低速验证）
const SPEED_LEVELS = [
  { id: "low", label: "低速", vx: 0.15, wz: 0.5 },
  { id: "medium", label: "中速", vx: 0.3, wz: 0.6 },
  { id: "high", label: "高速", vx: 0.6, wz: 0.8 },
];

const CONTROL_KEYS = ["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright", " "];

function isTypingTarget(el) {
  return el && (el.tagName === "INPUT" || el.tagName === "TEXTAREA" || el.isContentEditable);
}

const btnBase = {
  width: 52,
  height: 40,
  borderRadius: 8,
  border: "1px solid #34445a",
  background: "rgba(22,30,48,.95)",
  color: "#9fc0d8",
  fontSize: 13,
  fontWeight: 700,
  cursor: "pointer",
  userSelect: "none",
  touchAction: "none",
};

export default function ManualControlPanel({ publishCmdVel, viewMode, disabled, autoActive = false, modeKnown = false, modeBusy = false, onModeChange }) {
  const [speedLevel, setSpeedLevel] = useState(SPEED_LEVELS[1]);
  const [keyboardEnabled, setKeyboardEnabled] = useState(true);
  const [lastCmd, setLastCmd] = useState({ vx: 0, wz: 0 });

  const keysRef = useRef(new Set());
  const speedRef = useRef(speedLevel);
  speedRef.current = speedLevel;
  const disabledRef = useRef(disabled);
  disabledRef.current = disabled;
  const holdTimerRef = useRef(null);
  const lastSentRef = useRef({ vx: 0, wz: 0 });

  const publish = (vx, wz) => {
    if (disabledRef.current) return;
    setLastCmd({ vx, wz });
    publishCmdVel?.(vx, wz);
  };

  const sendIfChanged = (vx, wz) => {
    const last = lastSentRef.current;
    if (last.vx === vx && last.wz === wz) return;
    lastSentRef.current = { vx, wz };
    publish(vx, wz);
  };

  const computeCmd = (keys) => {
    const fwd = keys.has("w") || keys.has("arrowup");
    const back = keys.has("s") || keys.has("arrowdown");
    const left = keys.has("a") || keys.has("arrowleft");
    const right = keys.has("d") || keys.has("arrowright");
    const sp = speedRef.current;
    return {
      vx: (fwd ? 1 : 0) * sp.vx - (back ? 1 : 0) * sp.vx,
      wz: (left ? 1 : 0) * sp.wz - (right ? 1 : 0) * sp.wz,
    };
  };

  const stopHold = (clearKeys = false) => {
    if (holdTimerRef.current) {
      clearInterval(holdTimerRef.current);
      holdTimerRef.current = null;
    }
    if (clearKeys) keysRef.current.clear();
    publish(0, 0);
  };

  const startHold = (getCmd) => {
    stopHold();
    keysRef.current.clear(); // 用屏幕按钮时忽略键盘，避免两边指令互相覆盖
    const cmd = getCmd();
    publish(cmd.vx, cmd.wz);
    holdTimerRef.current = setInterval(() => {
      const c = getCmd();
      publish(c.vx, c.wz);
    }, 100);
  };

  useEffect(() => {
    const onKeyDown = (e) => {
      if (isTypingTarget(e.target)) return;
      const k = e.key.toLowerCase();
      if (!CONTROL_KEYS.includes(k) || e.repeat) return;
      // 方向键/空格需要阻止默认行为，避免滚动页面或触发聚焦按钮
      if (k === " " || k.startsWith("arrow")) e.preventDefault();
      keysRef.current.add(k);
    };
    const onKeyUp = (e) => {
      const k = e.key.toLowerCase();
      if (k === " ") e.preventDefault();
      keysRef.current.delete(k);
    };
    const onBlur = () => keysRef.current.clear();

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);

    const timer = setInterval(() => {
      if (disabledRef.current) return;
      // 自由视角下 WASD 留给相机漫游，只用按钮遥控
      if (!keyboardEnabled || viewMode === "free") return;
      const cmd = computeCmd(keysRef.current);
      sendIfChanged(cmd.vx, cmd.wz);
    }, 80);

    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
      clearInterval(timer);
      stopHold();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [keyboardEnabled, viewMode]);

  const hold = (getCmd) => ({
    onPointerDown: (e) => {
      e.preventDefault();
      e.currentTarget.setPointerCapture?.(e.pointerId);
      startHold(getCmd);
    },
    onPointerUp: stopHold,
    onPointerLeave: stopHold,
    onPointerCancel: stopHold,
  });

  const dirBtn = (label, title, getCmd) => (
    <button title={title} style={btnBase} {...hold(getCmd)}>{label}</button>
  );

  return (
    <div style={{
      display: "flex",
      alignItems: "center",
      gap: 14,
      padding: "10px 14px",
      borderRadius: 12,
      background: "rgba(10,14,26,.92)",
      border: "1px solid rgba(0,170,255,.25)",
      boxShadow: "0 6px 22px rgba(0,0,0,.4)",
      backdropFilter: "blur(6px)",
      pointerEvents: disabled ? "none" : "auto",
      opacity: disabled ? 0.55 : 1,
    }}>
      <div style={{ display: "flex", flexDirection: "column", gap: 6, alignItems: "center" }}>
        <span style={{ color: "#00aaff", fontSize: 11, fontWeight: 700, letterSpacing: 0.8 }}>🎮 手动遥控 /cmd_vel</span>
        <div style={{ display: "flex", gap: 4 }}>
          {SPEED_LEVELS.map((l) => (
            <button
              key={l.id}
              onClick={() => { setSpeedLevel(l); stopHold(true); }}
              style={{
                padding: "2px 8px",
                borderRadius: 5,
                border: speedLevel.id === l.id ? "1px solid #00ff88" : "1px solid transparent",
                background: speedLevel.id === l.id ? "rgba(0,255,136,.12)" : "transparent",
                color: speedLevel.id === l.id ? "#00ff88" : "#667",
                fontSize: 10,
                cursor: "pointer",
              }}
            >{l.label}</button>
          ))}
        </div>
        <div style={{ display: "flex", gap: 4 }}>
          {[
            { id: "manual", label: "手动" },
            { id: "autonomous", label: "AUTO" },
          ].map((m) => {
            const active = m.id === "autonomous" ? autoActive : modeKnown && !autoActive;
            return (
              <button
                key={m.id}
                title={m.id === "manual" ? "切到手动模式后用键盘/按钮遥控（仿真测试请先切这里）" : "切到自动驾驶模式"}
                onClick={() => onModeChange?.(m.id)}
                // 只在“没有切换能力”时禁用；切换中(modeBusy)仍可点击，
                // 否则 AUTO 请求卡住时手动/AUTO 会双双按不动。
                disabled={!onModeChange}
                style={{
                  padding: "2px 8px",
                  borderRadius: 5,
                  border: active ? "1px solid #00ff88" : "1px solid transparent",
                  background: active ? "rgba(0,255,136,.14)" : "transparent",
                  color: active ? "#00ff88" : "#667",
                  fontSize: 10,
                  cursor: onModeChange && !modeBusy ? "pointer" : "not-allowed",
                }}
              >{m.label}</button>
            );
          })}
        </div>
      </div>

      <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
        <div style={{ display: "grid", gridTemplateColumns: "repeat(3, 52px)", gridTemplateRows: "repeat(3, 40px)", gap: 5 }}>
          <div />
          {dirBtn("▲", "前进（W / ↑）", () => ({ vx: speedRef.current.vx, wz: 0 }))}
          <div />
          {dirBtn("◀", "左转（A / ←）", () => ({ vx: 0, wz: speedRef.current.wz }))}
          <button
            title="停止（空格）"
            onClick={() => stopHold(true)}
            style={{ ...btnBase, background: "rgba(0,120,90,.85)", border: "1px solid #00cc88", color: "#eafff5" }}
          >STOP</button>
          {dirBtn("▶", "右转（D / →）", () => ({ vx: 0, wz: -speedRef.current.wz }))}
          <div />
          {dirBtn("▼", "后退（S / ↓）", () => ({ vx: -speedRef.current.vx, wz: 0 }))}
          <div />
        </div>

        <div style={{ display: "flex", flexDirection: "column", gap: 6 }}>
          <button
            title="急停：立即发布零速"
            onClick={() => stopHold(true)}
            style={{
              width: 58,
              height: 40,
              borderRadius: 8,
              border: "1px solid #ff5555",
              background: "rgba(150,30,30,.9)",
              color: "#ffd7d7",
              fontSize: 12,
              fontWeight: 800,
              cursor: "pointer",
            }}
          >⚠ 急停</button>
          <button
            title="自由视角下 WASD 控制相机，建议用按钮遥控"
            onClick={() => setKeyboardEnabled(!keyboardEnabled)}
            style={{
              padding: "4px 6px",
              borderRadius: 6,
              border: "1px solid #34445a",
              background: keyboardEnabled ? "rgba(0,170,255,.12)" : "rgba(22,30,48,.95)",
              color: keyboardEnabled ? "#55ccff" : "#667",
              fontSize: 10,
              cursor: "pointer",
            }}
          >⌨ 键盘 {keyboardEnabled ? "开" : "关"}</button>
        </div>
      </div>

      <div style={{ display: "flex", flexDirection: "column", gap: 3, minWidth: 96 }}>
        <div style={{ color: "#556", fontSize: 10, letterSpacing: 0.6 }}>当前指令</div>
        <div style={{ color: "#00ff88", fontSize: 13, fontWeight: 700, fontVariantNumeric: "tabular-nums" }}>
          vx {lastCmd.vx.toFixed(2)} m/s
        </div>
        <div style={{ color: "#00aaff", fontSize: 13, fontWeight: 700, fontVariantNumeric: "tabular-nums" }}>
          wz {lastCmd.wz.toFixed(2)} rad/s
        </div>
        <div style={{ color: autoActive ? "#ffaa33" : "#556677", fontSize: 9, lineHeight: 1.4 }}>
          {autoActive
            ? "AUTO 模式：键盘 /cmd_vel 会被忽略，先点“手动”再遥控"
            : viewMode === "free"
              ? "自由视角：WASD 控制相机，请用按钮"
              : "WASD/方向键遥控，空格停止（仿真请先切“手动”）"}
        </div>
      </div>
    </div>
  );
}
