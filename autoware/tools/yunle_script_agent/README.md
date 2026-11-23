# 上位机脚本白名单服务

独立 WebSocket 服务，仅接受固定脚本 ID，不接受命令、参数或路径。服务以普通用户 `tang` 运行，默认只监听 WireGuard 地址 `10.66.0.2:8766`。

白名单中的三个独立启动项：

- `autoware` → `/home/tang/autoware/scripts/web_start_autoware.sh`
- `rosbridge` → `/home/tang/autoware/scripts/web_start_rosbridge.sh`
- `camera` → `/home/tang/autoware/scripts/web_start_camera.sh`

编译和自检：

```bash
cmake -S tools/yunle_script_agent -B /tmp/yunle-script-agent-build
cmake --build /tmp/yunle-script-agent-build -j
/tmp/yunle-script-agent-build/yunle_script_agent --self-test
```

