# ros_agent_mcp：无人车业务 MCP server

让大模型用自然语言调度无人车，但**只能调用 10 个业务工具**（其中 3 个只读），不能任意发布话题或调用服务。

## 一、原理

### 1. MCP 是什么

MCP（Model Context Protocol）是大模型和外部工具之间的标准接口，用的是 JSON-RPC 协议。它定义了三个角色：

- **MCP 客户端**：负责和大模型对话。可以是 Claude Desktop、Cursor，也可以是你的上位机 Electron 主进程。
- **MCP server**：也就是本程序。它向客户端声明“我有哪些工具、每个工具的参数格式（JSON Schema）”，并负责执行这些工具。
- **大模型**：只看得到工具的名称、说明和参数格式。它决定调用哪个工具、填什么参数，但不会自己执行。

一次调用的过程如下：

```
用户："派车去3号门"
  → 客户端把对话和工具列表发给大模型
  → 大模型返回：调用 set_destination(station_id="gate3")
  → 客户端通过 MCP 把这个调用发给本程序
  → 本程序完成校验、前置检查，必要时请求确认，然后通过 rosbridge 下发给工控机
  → 结果返回给客户端，再交给大模型
  → 大模型回复："已设置目的地 3号门，路线已生成"
```

### 2. 本程序的分层

```
大模型 ⇄ MCP 客户端 ⇄ (stdio) ⇄ server.py ⇄ (WebSocket) ⇄ rosbridge:9090 ⇄ Autoware AD API
```

| 层 | 作用 | 对应代码 |
| --- | --- | --- |
| 工具声明 | 只声明 10 个工具。`station_id` 由站点表生成枚举，模型只能从中选择 | `build_server` 中的 `@mcp.tool` |
| 统一外壳 | 连接 → 执行 → 拒绝原因结构化返回 → 写审计日志 | `run_tool` |
| 前置条件 | 读取缓存的车辆状态（数据超过 3 秒视为过期），判断能否执行 | `snapshot` / `require` |
| 人工确认 | 通过 MCP elicitation 请客户端弹窗确认。客户端不支持或用户未同意时一律拒绝执行 | `confirm` |
| rosbridge 客户端 | 一条 WebSocket 连接：后台订阅状态话题；服务调用按 id 等待返回结果 | `RosBridge` |

话题名、服务名和消息类型全部写死在代码里，与上位机 App.jsx 使用的接口一致，大模型无法接触到。

### 3. 工具与安全规则

| 工具 | 下发内容 | 前置条件 | 确认 |
| --- | --- | --- | --- |
| `get_vehicle_status` | 无（只读缓存） | — | — |
| `list_stations` | 无 | — | — |
| `get_diagnostics` | 无（只读 `/diagnostics` 缓存，可按级别过滤） | — | — |
| `init_localization` | `/api/localization/initialize` | 非自动驾驶 | — |
| `set_start_station` | 带站点位姿和协方差调用 `/api/localization/initialize`，格式与上位机“设起点”一致 | 非自动驾驶、车辆停稳 | — |
| `set_destination` | 发布 `/planning/mission_planning/goal` | 已定位 | 自动驾驶中改目的地需确认 |
| `dispatch_to_station` | 可选设置站点起点 → 退出旧任务 → 定位自检/恢复 → 清旧路线 → 新终点 → 等待规划 → 自动驾驶自检/恢复 → 切换并核验 | 关键状态可用；无法自动排除的底盘制动、紧急状态等仍会拒绝 | **切入自动驾驶前必须确认** |
| `clear_route` | `/api/routing/clear_route` | 非自动驾驶 | — |
| `start_autonomous` | `/api/operation_mode/change_to_autonomous` | 已定位、已有路线、Autoware 报告可启动、无切换中、底盘无配置的制动请求（可选） | **必须确认**，确认后重新检查条件 |
| `stop_vehicle` | `/api/operation_mode/change_to_stop` | 无（安全方向不设门槛） | — |

另外还有以下约束：

- 写操作有最小间隔（默认 1 秒），停车不受这个限制。
- 每次调用都写入 `audit.jsonl`，内容包括时间、工具、参数、执行还是拒绝、返回结果。
- 遥控（速度、档位）和急停**不提供工具**。`stop_vehicle` 是平稳停车，不能替代物理急停。

### 4. 失败原因怎么交给大模型

前置条件不满足时**不是抛错误，而是结构化返回**，让模型能定位“哪一步、为什么”，而不是只看到一句中文：

```json
{
  "refused": true,
  "code": "AUTONOMY_NOT_READY",
  "reason": "自动排查后仍不能启动自动驾驶：定位状态为“初始化中”",
  "issues": ["定位状态为“初始化中”", "Autoware 尚未开放自动驾驶切换"],
  "completed_steps": ["已切到停车模式", "已生成新路线"],
  "hint": "Autoware 未开放自动驾驶切换：按 issues 逐条排查……",
  "vehicle_state": { "operation_mode": "停车(STOP)", "localization": "初始化中", "route": "已设置路线" }
}
```

- `code` 是稳定错误码（`ROUTE_MISSING`、`LOC_NOT_READY`、`CHASSIS_BRAKE`、`PLAN_TIMEOUT`、`CONFIRM_DECLINED` 等），模型据此映射处置动作，不必解析中文措辞。
- `issues` 一次列全不满足的条件，避免模型只看到第一条就下结论。
- `completed_steps` 说明工作流已经做到哪一步（仅派车类工具），防止把“中途失败”误判成“完全没执行”。
- `hint` 由服务端的 `CODE_HINTS` 统一给出，保证每次建议一致、可核对。

诊断顺序：**先看 `refused` 里的结构化字段 → 再看 `get_diagnostics` 的模块级条目 → 仍无法定位时才需要人工去工控机看日志**。日志不进模型上下文，因为整篇日志既超长又充满正常信息，反而会诱发臆测。

## 二、安装与配置

```bash
pip install -r requirements.txt        # 兼容 mcp 1.x 和 2.x，需要 Python 3.10 及以上
cp config.example.json config.json     # 修改 rosbridge_url 和 stations
```

站点坐标使用 Autoware 的 map 坐标系，yaw 单位为弧度。最简单的取点方法是：在上位机地图上设一次终点，然后在工控机上执行 `ros2 topic echo /planning/mission_planning/goal --once`，读出 position 和 orientation（yaw = 2·atan2(z, w)）。

`chassis_feedback_topic`：底盘反馈走 ROS 时填 `/yunle_chassis/feedback/ccu_status`，启动前会检查制动请求；底盘走 CAN 直连网关时填 `null`。

`brake_interlock_fields`：列出哪些底盘字段算“制动请求”，可选值只有 `ccu_parking_status`、`emergency_brake_request_status`、`remote_brake_request_status`、`touch_brake_request_status`、`handle_brake_request_status`。默认排除 `ccu_parking_status`——自动驾驶中途停车时驻车位会自然置位，把它算作制动请求会导致重新派车被 `CHASSIS_BRAKE` / `AUTONOMY_NOT_READY` 拦截。急停、远程、触摸、手柄四项仍会硬拦截。如需恢复原行为，把五个字段全部填入即可。

## 三、运行方式

### A. 用 Claude Desktop 调试

编辑 `claude_desktop_config.json`：

```json
{
  "mcpServers": {
    "ros-agent": {
      "command": "python",
      "args": ["/path/to/ros_agent_mcp/server.py", "--config", "/path/to/ros_agent_mcp/config.json"]
    }
  }
}
```

注意：如果客户端不支持 elicitation，`start_autonomous` 和 `dispatch_to_station` 的自动驾驶切换都会被拒绝。这是有意的设计：没有确认通道，就不执行危险操作。

### B. 嵌入上位机（产品形态）

在 Electron 主进程里用 MCP SDK 实现一个客户端（TypeScript SDK：`@modelcontextprotocol/sdk`），用 stdio 启动 `server.py`：

1. 主进程持有大模型 API 密钥，负责调用模型，并把 `list_tools` 得到的工具交给模型。
2. 注册 elicitation 回调：通过 IPC 让前端弹出确认卡，用户点击后返回 `{action:"accept", content:{confirm:true}}`。
3. 遥控激活时，前端通知主进程，主进程在这段时间内直接拒绝所有写工具。
4. 前端的停车按钮和快捷键继续直接走 rosbridge，不经过 MCP。

## 四、测试

```bash
cd tests
python fake_rosbridge.py 19090 &        # 模拟工控机
python test_client.py 19090             # 端到端测试：枚举、前置条件、拒绝/同意确认、审计
```

## 五、上线前建议

- 在工控机的 rosbridge 上配置 `topics_glob` / `services_glob` 白名单。这样即使有人绕过本程序，也发不出控制量。
- 先在仿真环境里跑通，再上实车。实车首测选封闭场地，并安排人守在急停旁。
- 本程序没有处理“多车”和“任务队列”，这些可以在此基础上继续加工具。
