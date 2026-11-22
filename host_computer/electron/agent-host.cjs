const { spawn } = require("node:child_process");
const crypto = require("node:crypto");
const fs = require("node:fs");
const path = require("node:path");

const MCP_PROTOCOL_VERSION = "2025-06-18";
// 完整派车可能包含重新定位、两次规划恢复和 60 秒人工确认，不能沿用短工具超时。
const REQUEST_TIMEOUT_MS = 300_000;
const CONFIRM_TIMEOUT_MS = 60_000;
const MAX_TOOL_ROUNDS = 8;

const ALLOWED_TOOLS = new Set([
  "get_vehicle_status",
  "list_stations",
  "get_diagnostics",
  "init_localization",
  "set_start_station",
  "set_destination",
  "dispatch_to_station",
  "clear_route",
  "start_autonomous",
  "stop_vehicle",
]);

const WRITE_TOOLS = new Set([
  "init_localization",
  "set_start_station",
  "set_destination",
  "dispatch_to_station",
  "clear_route",
  "start_autonomous",
  "stop_vehicle",
]);

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

const SYSTEM_PROMPT = `你是核星力无人车调度助手。必须遵守以下规则：
1. 只能通过提供的工具读取或操作车辆，绝不声称执行了尚未成功返回的操作。
2. 不提供速度、方向、档位、遥控或物理急停控制。用户要求这些操作时，说明需要使用上位机手动控制或物理急停。
3. 用户说“停”“停车”“立即停车”时，优先调用 stop_vehicle；但它只是 Autoware 平稳停车，不能替代物理急停。
4. 设置目的地只能选择 list_stations 返回的站点，不得虚构坐标或站点。
5. 默认把“去某站”“前往某站”“派车到某站”“把终点设为某站”等目的地请求理解为完整发车任务，优先调用 dispatch_to_station。用户说“从A去B”时把 A 填入 start_station_id、B 填入 station_id；工具会按上位机格式设置起点、清除旧路线、设置新终点、诊断并尝试修复、弹出人工确认，然后切入并核验自动驾驶。不要把这一流程拆成多个写工具。
6. 如果用户明确说“只设置终点/只规划路线，不启动自动驾驶”，才使用 set_destination，且不得继续启动。
7. 每轮用户消息最多调用一个改变车辆状态的工具；dispatch_to_station 虽然内部包含受控步骤，仍视为一个工具。
8. 工具返回 refused 时按固定顺序诊断，不要凭感觉猜：①先读返回里的 code、reason、check_items、issues、completed_steps、vehicle_state；②需要模块级信息时调用 get_diagnostics（只读，可按 level=error 只看错误）；③仍无法定位时如实说明还缺什么信息。
9. 诊断结论必须引用证据：错误码、issues 条目或诊断项名称。不得输出“可能是网络问题”这类没有依据的猜测；确实无法判断时直接说“无法确定原因，需要现场检查”。
10. 建议按三类给出：先处置（操作员现在就能做）、再检查（需要到现场或看工控机）、最后才考虑重新发起。如果返回里 completed_steps 非空，必须说明“已经完成到哪一步”，不要让用户误以为什么都没做。
11. 你只能给出建议，不得自行下发修复类写操作；需要重试时，让用户确认后重新发起。
12. 回答使用简洁中文，明确区分“正在排查”“已下发”“已确认进入自动驾驶”。
13. 报错或排查类回答必须逐条分行，用纯文本（界面不渲染 Markdown，不要写 **、#、反引号）。固定结构：
   第一行：一句结论，注明错误码或证据；若 completed_steps 非空，紧跟一行“已完成：...”。
   空一行后写“可检查项：”，下面每行一项，以“- ”开头，写成“- 项目：当前状态或证据”。把工具返回的 check_items / issues 和 get_diagnostics 的 ERROR 条目逐条列出，一项一行，不合并、不遗漏。
   再空一行写“建议：”，下面每行一条，以“- ”开头，按“先处置 → 再检查 → 重新发起”排序。
   禁止把多个检查项用顿号、分号挤在同一行；每项必须单独占一行。`;

class McpStdioClient {
  constructor({ command, args, cwd, onElicitation, onLog }) {
    this.command = command;
    this.args = args;
    this.cwd = cwd;
    this.onElicitation = onElicitation;
    this.onLog = onLog;
    this.child = null;
    this.buffer = "";
    this.nextId = 1;
    this.pending = new Map();
    this.tools = [];
    this.connected = false;
  }

  async connect() {
    if (this.connected) return;
    if (this.child) await this.close();

    this.child = spawn(this.command, this.args, {
      cwd: this.cwd,
      windowsHide: true,
      stdio: ["pipe", "pipe", "pipe"],
      env: { ...process.env, PYTHONIOENCODING: "utf-8", PYTHONUNBUFFERED: "1" },
    });

    this.child.stdout.setEncoding("utf8");
    this.child.stderr.setEncoding("utf8");
    this.child.stdout.on("data", (chunk) => this._consume(chunk));
    this.child.stderr.on("data", (chunk) => this.onLog?.(String(chunk).trim()));
    this.child.on("error", (error) => this._failAll(error));
    this.child.on("exit", (code, signal) => {
      const detail = signal ? `signal=${signal}` : `code=${code}`;
      this._failAll(new Error(`MCP 服务已退出（${detail}）`));
    });

    const initialized = await this.request("initialize", {
      protocolVersion: MCP_PROTOCOL_VERSION,
      capabilities: { elicitation: {} },
      clientInfo: { name: "hexingli-vehicle-console", version: "1.0.0" },
    }, 15_000);

    if (!initialized?.serverInfo) throw new Error("MCP 初始化响应无效");
    this.notify("notifications/initialized", {});
    const listed = await this.request("tools/list", {}, 15_000);
    this.tools = Array.isArray(listed?.tools)
      ? listed.tools.filter((tool) => ALLOWED_TOOLS.has(tool.name))
      : [];
    if (!this.tools.length) throw new Error("MCP 服务没有返回允许的车辆工具");
    this.connected = true;
  }

  _consume(chunk) {
    this.buffer += chunk;
    while (true) {
      const newline = this.buffer.indexOf("\n");
      if (newline < 0) break;
      const line = this.buffer.slice(0, newline).trim();
      this.buffer = this.buffer.slice(newline + 1);
      if (!line) continue;
      try {
        this._handleMessage(JSON.parse(line));
      } catch (error) {
        this.onLog?.(`无法解析 MCP 输出：${error.message}`);
      }
    }
  }

  async _handleMessage(message) {
    if (Object.prototype.hasOwnProperty.call(message, "id") && (message.result !== undefined || message.error)) {
      const pending = this.pending.get(message.id);
      if (!pending) return;
      this.pending.delete(message.id);
      clearTimeout(pending.timer);
      if (message.error) pending.reject(new Error(message.error.message || "MCP 请求失败"));
      else pending.resolve(message.result);
      return;
    }

    if (Object.prototype.hasOwnProperty.call(message, "id") && message.method) {
      if (message.method === "elicitation/create") {
        try {
          const result = await this.onElicitation(message.params || {});
          this._write({ jsonrpc: "2.0", id: message.id, result });
        } catch (error) {
          this._write({
            jsonrpc: "2.0",
            id: message.id,
            error: { code: -32603, message: error.message || "人工确认失败" },
          });
        }
        return;
      }
      this._write({
        jsonrpc: "2.0",
        id: message.id,
        error: { code: -32601, message: `不支持的服务端请求：${message.method}` },
      });
    }
  }

  _write(payload) {
    if (!this.child?.stdin?.writable) throw new Error("MCP 服务未连接");
    this.child.stdin.write(`${JSON.stringify(payload)}\n`);
  }

  request(method, params = {}, timeoutMs = REQUEST_TIMEOUT_MS) {
    const id = this.nextId++;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`MCP 请求超时：${method}`));
      }, timeoutMs);
      this.pending.set(id, { resolve, reject, timer });
      try {
        this._write({ jsonrpc: "2.0", id, method, params });
      } catch (error) {
        clearTimeout(timer);
        this.pending.delete(id);
        reject(error);
      }
    });
  }

  notify(method, params = {}) {
    this._write({ jsonrpc: "2.0", method, params });
  }

  async callTool(name, args) {
    if (!ALLOWED_TOOLS.has(name)) throw new Error(`工具不在白名单中：${name}`);
    return this.request("tools/call", { name, arguments: args || {} });
  }

  _failAll(error) {
    this.connected = false;
    for (const pending of this.pending.values()) {
      clearTimeout(pending.timer);
      pending.reject(error);
    }
    this.pending.clear();
  }

  async close() {
    this.connected = false;
    const child = this.child;
    this.child = null;
    this._failAll(new Error("MCP 连接已关闭"));
    if (!child) return;
    try { child.stdin.end(); } catch { /* 已关闭 */ }
    try { child.kill(); } catch { /* 已退出 */ }
  }
}

class VehicleAgentHost {
  constructor({ app, getMainWindow }) {
    this.app = app;
    this.getMainWindow = getMainWindow;
    this.context = { teleopActive: false, vehicleChannel: "", driveMode: "real" };
    this.mcp = null;
    this.busy = false;
    this.activeAbort = null;
    this.pendingConfirmations = new Map();
    this.history = [];
    this.status = {
      state: "idle",
      message: "Agent 待命",
      model: this._modelName(),
      hasApiKey: Boolean(process.env.DEEPSEEK_API_KEY),
    };
  }

  _modelName() {
    return process.env.DEEPSEEK_MODEL || "deepseek-chat";
  }

  _emit(payload) {
    const win = this.getMainWindow?.();
    if (win && !win.isDestroyed()) win.webContents.send("agent:event", payload);
  }

  _setStatus(state, message) {
    this.status = {
      state,
      message,
      model: this._modelName(),
      hasApiKey: Boolean(process.env.DEEPSEEK_API_KEY),
    };
    this._emit({ type: "status", ...this.status });
  }

  getStatus() {
    return {
      ...this.status,
      driveMode: this.context.driveMode,
      busy: this.busy,
      mcpConnected: Boolean(this.mcp?.connected),
      hasApiKey: Boolean(process.env.DEEPSEEK_API_KEY),
    };
  }

  async setContext(next = {}) {
    const previousChannel = this.context.vehicleChannel;
    const previousMode = this.context.driveMode;
    this.context = {
      ...this.context,
      teleopActive: Boolean(next.teleopActive),
      vehicleConnected: Boolean(next.vehicleConnected),
      vehicleChannel: typeof next.vehicleChannel === "string" ? next.vehicleChannel.trim() : this.context.vehicleChannel,
      // 仿真 / 实车：只影响 MCP 的底盘联锁（仿真没有云乐底盘），其余条件都以 Autoware 为准
      driveMode: next.driveMode === "sim" ? "sim" : next.driveMode === "real" ? "real" : this.context.driveMode,
    };
    const channelChanged = previousChannel && this.context.vehicleChannel && previousChannel !== this.context.vehicleChannel;
    const modeChanged = previousMode !== this.context.driveMode;
    if (channelChanged || modeChanged) {
      this.cancel();
      if (this.mcp) await this.mcp.close();
      this.mcp = null;
      this.history = [];
      this._setStatus("idle", modeChanged
        ? `已切换为${this.context.driveMode === "sim" ? "仿真" : "实车"}模式，Agent 将在下次对话时重连`
        : "车辆通道已改变，Agent 将在下次对话时重连");
    }
    return this.getStatus();
  }

  _paths() {
    const packagedDir = path.join(process.resourcesPath, "ros_agent_mcp");
    const developmentDir = path.resolve(__dirname, "../agent/ros_agent_mcp");
    const agentDir = this.app.isPackaged ? packagedDir : developmentDir;
    const runtimeDir = path.join(this.app.getPath("userData"), "vehicle-agent");
    fs.mkdirSync(runtimeDir, { recursive: true });
    return {
      agentDir,
      runtimeDir,
      serverPy: path.join(agentDir, "server.py"),
      serverExe: path.join(agentDir, "ros-agent-server.exe"),
      templateConfig: path.join(agentDir, "config.example.json"),
      projectConfig: path.join(agentDir, "config.json"),
      userConfig: path.join(runtimeDir, "config.user.json"),
      runtimeConfig: path.join(runtimeDir, "config.json"),
      auditLog: path.join(runtimeDir, "audit.jsonl"),
    };
  }

  _safeRosbridgeUrl(value, fallback) {
    const raw = value || fallback;
    let parsed;
    try { parsed = new URL(raw); } catch { throw new Error(`rosbridge 地址无效：${raw || "未配置"}`); }
    if (parsed.protocol !== "ws:" && parsed.protocol !== "wss:") {
      throw new Error("Agent 只允许连接 ws:// 或 wss:// rosbridge 地址");
    }
    return parsed.toString().replace(/\/$/, "");
  }

  _sourceConfigPath(paths = this._paths()) {
    const configured = process.env.VEHICLE_AGENT_CONFIG;
    if (configured) return configured;
    if (!this.app.isPackaged && fs.existsSync(paths.projectConfig)) return paths.projectConfig;
    if (!fs.existsSync(paths.userConfig)) fs.copyFileSync(paths.templateConfig, paths.userConfig);
    return paths.userConfig;
  }

  getStations() {
    const paths = this._paths();
    const sourceConfig = this._sourceConfigPath(paths);
    const config = JSON.parse(fs.readFileSync(sourceConfig, "utf8"));
    const stations = Array.isArray(config.stations) ? config.stations : [];
    return {
      stations: stations.map(({ id, name, aliases, x, y, yaw }) => ({ id, name, aliases: aliases || [], x, y, yaw })),
    };
  }

  // ── 站点表增删改：统一校验 → 备份 → 写回 → 让 Agent 用新站点表重连 ──
  _publicStation(station) {
    return { id: station.id, name: station.name, aliases: station.aliases || [], x: station.x, y: station.y, yaw: station.yaw };
  }

  _normalizeStationFields(next, { requirePose }) {
    const out = {};
    if (next.name !== undefined) {
      const name = String(next.name || "").trim();
      if (!name) throw new Error("站点名称不能为空");
      if (name.length > 40) throw new Error("站点名称不能超过 40 个字");
      out.name = name;
    }
    if (next.aliases !== undefined) {
      const list = Array.isArray(next.aliases) ? next.aliases : String(next.aliases || "").split(/[,，、\s]+/);
      out.aliases = [...new Set(list.map((item) => String(item || "").trim()).filter(Boolean))].slice(0, 12);
      if (out.aliases.some((alias) => alias.length > 40)) throw new Error("单个别名不能超过 40 个字");
    }
    const hasPose = ["x", "y", "yaw"].some((key) => next[key] !== undefined && next[key] !== null);
    if (hasPose || requirePose) {
      const x = Number(next.x);
      const y = Number(next.y);
      const yaw = Number(next.yaw);
      if (![x, y, yaw].every(Number.isFinite)) throw new Error("站点 x、y、yaw 必须是有限数字");
      if (yaw < -Math.PI || yaw > Math.PI) throw new Error("站点 yaw 必须在 ±π 范围内");
      Object.assign(out, { x, y, yaw });
    }
    return out;
  }

  _checkNameConflicts(stations, selfId, fields) {
    const names = [fields.name, ...(fields.aliases || [])].filter(Boolean);
    for (const other of stations) {
      if (other.id === selfId) continue;
      const taken = [other.name, ...(other.aliases || [])];
      const clash = names.find((name) => taken.includes(name));
      if (clash) throw new Error(`名称或别名“${clash}”已被站点“${other.name}”使用`);
    }
  }

  async _editStations(label, mutate) {
    if (this.busy) throw new Error("Agent 正在执行任务，任务结束后才能修改站点");
    const paths = this._paths();
    const sourceConfig = this._sourceConfigPath(paths);
    const config = JSON.parse(fs.readFileSync(sourceConfig, "utf8"));
    if (!Array.isArray(config.stations)) config.stations = [];
    const result = mutate(config.stations);

    const stamp = new Date().toISOString().replace(/[:.]/g, "-");
    fs.copyFileSync(sourceConfig, `${sourceConfig}.before-station-${label}-${stamp}.bak`);
    fs.writeFileSync(sourceConfig, `${JSON.stringify(config, null, 2)}\n`, "utf8");

    if (this.mcp) await this.mcp.close();
    this.mcp = null;
    this.history = [];
    return result;
  }

  async updateStation(next = {}) {
    const id = String(next.id || "").trim();
    if (!/^[A-Za-z0-9_-]+$/.test(id)) throw new Error("站点 id 无效");
    const fields = this._normalizeStationFields(next, { requirePose: false });
    const station = await this._editStations(id, (stations) => {
      const target = stations.find((item) => item.id === id);
      if (!target) throw new Error(`未找到站点：${id}`);
      this._checkNameConflicts(stations, id, fields);
      Object.assign(target, fields);
      return target;
    });
    this._setStatus("idle", `站点“${station.name}”已更新，Agent 将使用新站点表重连`);
    return { station: this._publicStation(station) };
  }

  async addStation(next = {}) {
    const fields = this._normalizeStationFields(next, { requirePose: true });
    if (!fields.name) throw new Error("请填写站点名称");
    const station = await this._editStations("add", (stations) => {
      this._checkNameConflicts(stations, null, fields);
      let id;
      do { id = `st_${Date.now().toString(36)}${crypto.randomBytes(2).toString("hex")}`; } while (stations.some((item) => item.id === id));
      const created = { id, name: fields.name, aliases: fields.aliases || [], x: fields.x, y: fields.y, yaw: fields.yaw };
      stations.push(created);
      return created;
    });
    this._setStatus("idle", `已新增站点“${station.name}”，Agent 将使用新站点表重连`);
    return { station: this._publicStation(station) };
  }

  async deleteStation(payload = {}) {
    const id = String(payload.id || "").trim();
    if (!/^[A-Za-z0-9_-]+$/.test(id)) throw new Error("站点 id 无效");
    const removed = await this._editStations(`delete-${id}`, (stations) => {
      const index = stations.findIndex((item) => item.id === id);
      if (index < 0) throw new Error(`未找到站点：${id}`);
      if (stations.length <= 1) throw new Error("至少要保留一个站点（Agent 服务需要站点表才能启动）");
      return stations.splice(index, 1)[0];
    });
    this._setStatus("idle", `已删除站点“${removed.name}”，Agent 将使用新站点表重连`);
    return { id, deleted: true };
  }

  _prepareRuntime() {
    const paths = this._paths();
    if (!fs.existsSync(paths.templateConfig)) throw new Error(`缺少 MCP 配置模板：${paths.templateConfig}`);
    const sourceConfig = this._sourceConfigPath(paths);
    if (!fs.existsSync(sourceConfig)) throw new Error(`Agent 配置文件不存在：${sourceConfig}`);
    const config = JSON.parse(fs.readFileSync(sourceConfig, "utf8"));
    if (config.stations_configured !== true) {
      throw new Error(`请先填写真实站点坐标并把 stations_configured 改为 true：${sourceConfig}`);
    }
    config.rosbridge_url = this._safeRosbridgeUrl(this.context.vehicleChannel, config.rosbridge_url);
    // 仿真没有云乐底盘：去掉底盘反馈联锁，是否能进自动驾驶完全以 Autoware 的
    // is_autonomous_mode_available（界面上“启动自动驾驶”常亮）为准。实车保留底盘制动联锁。
    if (this.context.driveMode === "sim") config.chassis_feedback_topic = null;
    config.audit_log = paths.auditLog;
    fs.writeFileSync(paths.runtimeConfig, `${JSON.stringify(config, null, 2)}\n`, "utf8");
    return paths;
  }

  _pythonCommand(paths) {
    if (this.app.isPackaged && fs.existsSync(paths.serverExe)) {
      return { command: paths.serverExe, args: ["--config", paths.runtimeConfig] };
    }
    const configured = process.env.VEHICLE_AGENT_PYTHON;
    const venvPython = path.join(paths.agentDir, ".venv", process.platform === "win32" ? "Scripts/python.exe" : "bin/python");
    const command = configured || (fs.existsSync(venvPython) ? venvPython : "python");
    if (!fs.existsSync(paths.serverPy)) throw new Error(`缺少 MCP 服务：${paths.serverPy}`);
    return { command, args: ["-u", paths.serverPy, "--config", paths.runtimeConfig] };
  }

  async _ensureMcp() {
    if (this.mcp?.connected) return this.mcp;
    const paths = this._prepareRuntime();
    const launch = this._pythonCommand(paths);
    this._setStatus("connecting", "正在连接车辆 Agent 服务…");
    const mcp = new McpStdioClient({
      command: launch.command,
      args: launch.args,
      cwd: paths.agentDir,
      onElicitation: (params) => this._requestConfirmation(params),
      onLog: (message) => {
        if (message) this._emit({ type: "log", message });
      },
    });
    this.mcp = mcp;
    try {
      await mcp.connect();
      this._setStatus("ready", "车辆 Agent 已连接");
      return mcp;
    } catch (error) {
      await mcp.close();
      if (this.mcp === mcp) this.mcp = null;
      this._setStatus("error", error.message);
      throw error;
    }
  }

  _requestConfirmation(params) {
    const id = crypto.randomUUID();
    const message = params.message || "该车辆操作需要操作员确认";
    const kind = /启动自动驾驶|进入自动驾驶/.test(message) ? "autonomous" : "vehicle-operation";
    const win = this.getMainWindow?.();
    if (win && !win.isDestroyed()) {
      if (win.isMinimized()) win.restore();
      if (!win.isVisible()) win.show();
      win.focus();
      win.flashFrame(true);
    }
    return new Promise((resolve) => {
      const timer = setTimeout(() => {
        this.pendingConfirmations.delete(id);
        resolve({ action: "cancel" });
        if (win && !win.isDestroyed()) win.flashFrame(false);
        this._emit({ type: "confirmation-expired", id });
      }, CONFIRM_TIMEOUT_MS);
      this.pendingConfirmations.set(id, { resolve, timer });
      this._emit({ type: "confirmation", id, kind, message, timeoutMs: CONFIRM_TIMEOUT_MS });
    });
  }

  respondConfirmation(id, accepted) {
    const pending = this.pendingConfirmations.get(id);
    if (!pending) return false;
    clearTimeout(pending.timer);
    this.pendingConfirmations.delete(id);
    const win = this.getMainWindow?.();
    if (win && !win.isDestroyed()) win.flashFrame(false);
    pending.resolve(accepted
      ? { action: "accept", content: { confirm: true } }
      : { action: "decline" });
    return true;
  }

  _modelTools(mcp) {
    return mcp.tools.map((tool) => ({
      type: "function",
      function: {
        name: tool.name,
        description: tool.description || TOOL_LABELS[tool.name] || tool.name,
        parameters: tool.inputSchema || { type: "object", properties: {} },
      },
    }));
  }

  async _deepSeekCompletion(messages, tools, signal) {
    const apiKey = process.env.DEEPSEEK_API_KEY;
    if (!apiKey) throw new Error("未配置 DEEPSEEK_API_KEY，请在 Windows 用户环境变量中设置后重启上位机");
    const baseUrl = (process.env.DEEPSEEK_BASE_URL || "https://api.deepseek.com").replace(/\/$/, "");
    const response = await fetch(`${baseUrl}/chat/completions`, {
      method: "POST",
      headers: {
        Authorization: `Bearer ${apiKey}`,
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        model: this._modelName(),
        messages,
        tools,
        tool_choice: "auto",
        stream: false,
        thinking: { type: "disabled" },
      }),
      signal,
    });
    const text = await response.text();
    let body;
    try { body = JSON.parse(text); } catch { body = null; }
    if (!response.ok) {
      const detail = body?.error?.message || text.slice(0, 500) || `HTTP ${response.status}`;
      throw new Error(`DeepSeek API 调用失败：${detail}`);
    }
    const message = body?.choices?.[0]?.message;
    if (!message) throw new Error("DeepSeek API 没有返回有效消息");
    return message;
  }

  // Agent 按站点设了定位起点：通知界面把起点标牌放到该站点。
  // （Agent 直接调用定位初始化服务，不会发布 /initialpose，界面自己收不到新起点。）
  _emitStartStation(name, args, output) {
    let stationId = null;
    if (name === "set_start_station" && output?.ok) stationId = args?.station_id;
    if (name === "dispatch_to_station" && args?.start_station_id) {
      const steps = Array.isArray(output?.steps) ? output.steps : [];
      if (output?.ok || steps.some((step) => String(step).includes("定位起点"))) stationId = args.start_station_id;
    }
    if (!stationId) return;
    try {
      const station = this.getStations().stations.find((item) => item.id === stationId);
      if (station) this._emit({ type: "start-station", station });
    } catch { /* 读取站点表失败时界面仍会按定位初始化状态更新起点 */ }
  }

  _toolOutput(result) {
    if (result?.structuredContent !== undefined) return result.structuredContent;
    if (Array.isArray(result?.content)) {
      const texts = result.content.filter((item) => item?.type === "text").map((item) => item.text);
      if (texts.length === 1) {
        try { return JSON.parse(texts[0]); } catch { return texts[0]; }
      }
      return texts;
    }
    return result;
  }

  async sendMessage(text) {
    const input = String(text || "").trim();
    if (!input) throw new Error("消息不能为空");
    if (input.length > 2000) throw new Error("单条消息不能超过 2000 个字符");
    if (this.busy) throw new Error("Agent 正在处理上一条消息");

    this.busy = true;
    this.activeAbort = new AbortController();
    this._setStatus("thinking", "Agent 正在思考…");
    try {
      const mcp = await this._ensureMcp();
      const tools = this._modelTools(mcp);
      const messages = [
        { role: "system", content: SYSTEM_PROMPT },
        {
          role: "system",
          content: this.context.driveMode === "sim"
            ? "当前上位机处于【仿真】模式：没有真实底盘，不检查底盘反馈；能否进入自动驾驶以 Autoware 的自动驾驶可用状态为准。"
            : "当前上位机处于【实车】模式：除 Autoware 的自动驾驶可用状态外，还要求底盘反馈在线且没有配置的制动请求（驻车状态不算）。",
        },
        ...this.history.slice(-12),
        { role: "user", content: input },
      ];
      let writeUsed = false;

      for (let round = 0; round < MAX_TOOL_ROUNDS; round += 1) {
        const modelMessage = await this._deepSeekCompletion(messages, tools, this.activeAbort.signal);
        const assistantMessage = {
          role: "assistant",
          content: modelMessage.content || "",
        };
        if (Array.isArray(modelMessage.tool_calls) && modelMessage.tool_calls.length) {
          assistantMessage.tool_calls = modelMessage.tool_calls;
        }
        messages.push(assistantMessage);

        const calls = assistantMessage.tool_calls || [];
        if (!calls.length) {
          const answer = assistantMessage.content.trim() || "操作已完成。";
          this.history.push({ role: "user", content: input }, { role: "assistant", content: answer });
          this.history = this.history.slice(-12);
          this._setStatus("ready", "车辆 Agent 已连接");
          return { text: answer };
        }

        for (const call of calls) {
          const name = call?.function?.name;
          let args = {};
          let output;
          try {
            if (!ALLOWED_TOOLS.has(name)) throw new Error(`工具不在允许列表中：${name || "未知"}`);
            args = JSON.parse(call.function.arguments || "{}");
            if (!args || Array.isArray(args) || typeof args !== "object") throw new Error("工具参数必须是 JSON 对象");
            if (WRITE_TOOLS.has(name) && this.context.teleopActive) {
              output = { refused: true, reason: "底盘手动遥控已激活。请先在上位机退出手动控制，再重新发起 Agent 操作。" };
            } else if (WRITE_TOOLS.has(name) && name !== "stop_vehicle" && writeUsed) {
              output = { refused: true, reason: "为确保安全，每轮对话只允许一个改变车辆状态的操作。请确认当前结果后另行发起下一步。" };
            } else {
              if (WRITE_TOOLS.has(name) && name !== "stop_vehicle") writeUsed = true;
              this._emit({ type: "tool-start", name, label: TOOL_LABELS[name] || name, arguments: args });
              const result = await mcp.callTool(name, args);
              output = this._toolOutput(result);
              this._emit({ type: "tool-end", name, label: TOOL_LABELS[name] || name, result: output });
              this._emitStartStation(name, args, output);
            }
          } catch (error) {
            output = { refused: true, reason: error.message };
            this._emit({ type: "tool-end", name, label: TOOL_LABELS[name] || name || "未知工具", result: output });
          }
          messages.push({
            role: "tool",
            tool_call_id: call.id,
            content: JSON.stringify(output),
          });
        }
      }
      throw new Error("Agent 工具调用轮数超过安全上限");
    } catch (error) {
      const message = error.name === "AbortError" ? "本轮 Agent 操作已取消" : error.message;
      this._setStatus("error", message);
      throw new Error(message);
    } finally {
      this.busy = false;
      this.activeAbort = null;
    }
  }

  cancel() {
    this.activeAbort?.abort();
    const win = this.getMainWindow?.();
    if (win && !win.isDestroyed()) win.flashFrame(false);
    for (const [id, pending] of this.pendingConfirmations) {
      clearTimeout(pending.timer);
      pending.resolve({ action: "cancel" });
      this.pendingConfirmations.delete(id);
    }
    return true;
  }

  async shutdown() {
    this.cancel();
    await this.mcp?.close();
    this.mcp = null;
  }
}

module.exports = { VehicleAgentHost };
