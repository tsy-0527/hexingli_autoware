# 云乐底盘直连网关（无 ROS）

这个进程是浏览器和 Linux SocketCAN 之间的最薄适配层：

```text
App.jsx --WebSocket--> yunle_can_gateway --> can0/can1 --> 云乐底盘
```

它不链接、不启动也不调用 ROS。当前协议来自仓库中的云乐 2026 DBC 实现：

- `0x121`：SCU 控制，固定 10 ms 周期发送；
- `0x51`：CCU 状态；
- `0x100`：BMS 状态。

## 编译与自检

```bash
cmake -S tools/yunle_can_gateway -B /tmp/yunle-can-gateway-build
cmake --build /tmp/yunle-can-gateway-build -j
/tmp/yunle-can-gateway-build/yunle_can_gateway --self-test
```

## 先架空车轮测试

当前仓库底盘配置是逻辑 CAN2→Linux `can0`，因此默认在 `can0` 发送 `0x121`，同时从 `can0,can1` 接收反馈。如果现场接线相反，把 `--control-interface` 改成 `can1`，不要同时向两路发送控制帧。

2026-10-25 在当前双通道桥上只读实测：`0x51` 同时镜像到 `can0/can1`，`0x100` 出现在 `can1`。网关会对两路同时到达的相同 `0x51` 去重。

```bash
sudo /tmp/yunle-can-gateway-build/yunle_can_gateway \
  --listen 0.0.0.0 \
  --interfaces can0,can1 \
  --control-interface can0 \
  --token '请替换成长随机字符串' \
  --max-speed-kmh 5.0 \
  --max-steering-deg 21.0 \
  --physical-max-steering-deg 21.0
```

`public/config.json` 增加：

```json
{
  "chassisGateway": {
    "enabled": true,
    "url": "ws://工控机IP:8765",
    "token": "与网关相同的字符串"
  }
}
```

启用直连后，前端不再通过 rosbridge 发布/订阅底盘控制、CCU、BMS 话题；定位、规划、地图和 Autoware 服务不受影响。

## 安全约束

- 同一时刻只有一个 WebSocket 客户端能取得控制权；
- 浏览器指令超过 200 ms 未刷新，立即停止发送 `0x121`；
- `0x51` 反馈超过 500 ms 未刷新，立即停止发送 `0x121`；
- 自动驾驶物理按钮未闭合，或急停/遥控/脚刹/手刹请求有效时，只允许零速制动帧；
- 当前车型进入 AUTO 后会持续置位 `CCU_Parking_Status`，与现有 Autoware 桥一致，不把该位单独作为运动联锁；
- 车速不低于 0.2 km/h 时禁止 D/R 直接切换；
- 服务端再做一次 5.0 km/h、21° 硬限幅，不能由浏览器绕过；当前车辆实测机械限位约为左 20.5°、右 20.9°。
- `0x121` 前轮转角是有符号 8 位编码值，物理最大转角映射到原始值 `±120`。当前按实测机械限位 21° 配置：上位机左转 21° 经方向转换后为 `0x88`（-120），右转 21° 为 `0x78`（+120）。
- 所有运动帧显式置位 `Steering_Angle_Speed_Valid`（bit 60）；该位缺失时，当前 VCU 会执行转向但忽略档位和目标速度。
- `0x51` 转向反馈先按 `0.1/bit` 取得执行器编码角，再按 `编码角 / 120 × 21°` 换算成前轮物理转角；其原生符号为右正左负，上位机在直连边界转换为 Autoware 的左正右负。

直连网关与 `chassis_driver` 都会发送 `0x121`。实车启用前必须停掉旧的底盘控制发送者，保证总线上只有一个控制源；Autoware 本体可以继续运行。

