const { app, BrowserWindow, ipcMain } = require("electron");
const dgram = require("node:dgram");
const http = require("node:http");
const fs = require("node:fs");
const path = require("node:path");
const { VehicleAgentHost } = require("./agent-host.cjs");

let staticServer;
let mainWindow;
let rosUdpSocket;
let agentHost;

const MIME = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".jpeg": "image/jpeg",
  ".ico": "image/x-icon",
  ".glb": "model/gltf-binary",
  ".gltf": "model/gltf+json",
  ".pcd": "application/octet-stream",
  ".bin": "application/octet-stream",
  ".woff2": "font/woff2",
};

function startStaticServer() {
  const root = path.resolve(__dirname, "../dist");

  return new Promise((resolve, reject) => {
    staticServer = http.createServer((req, res) => {
      try {
        const pathname = decodeURIComponent(
          new URL(req.url, "http://127.0.0.1").pathname
        );

        const relativePath =
          pathname === "/" ? "index.html" : pathname.replace(/^\/+/, "");

        const filePath = path.resolve(root, relativePath);

        if (
          filePath !== root &&
          !filePath.startsWith(root + path.sep)
        ) {
          res.writeHead(403);
          res.end("Forbidden");
          return;
        }

        fs.stat(filePath, (error, stat) => {
          if (error || !stat.isFile()) {
            res.writeHead(404);
            res.end("Not Found");
            return;
          }

          const contentType =
            MIME[path.extname(filePath).toLowerCase()] ||
            "application/octet-stream";

          res.writeHead(200, {
            "Content-Type": contentType,
            "Cache-Control": "no-cache",
          });

          fs.createReadStream(filePath).pipe(res);
        });
      } catch (error) {
        res.writeHead(500);
        res.end("Internal Server Error");
      }
    });

    staticServer.once("error", reject);
    staticServer.listen(0, "127.0.0.1", () => {
      const { port } = staticServer.address();
      resolve(`http://127.0.0.1:${port}`);
    });
  });
}

function stopRosUdp() {
  if (!rosUdpSocket) return;
  const socket = rosUdpSocket;
  rosUdpSocket = null;
  try { socket.close(); } catch { /* socket 尚未完成 bind 时也允许安全退出 */ }
}

function startRosUdp(options = {}) {
  const port = Number(options.port);
  const address = String(options.address || "0.0.0.0").trim();
  const allowedSource = String(options.allowedSource || "").trim();

  if (!Number.isInteger(port) || port < 1 || port > 65535) {
    throw new Error("UDP 监听端口必须是 1~65535 的整数");
  }

  stopRosUdp();
  const socket = dgram.createSocket({ type: "udp4", reuseAddr: true });
  rosUdpSocket = socket;

  return new Promise((resolve, reject) => {
    let settled = false;

    const rejectStart = (error) => {
      if (settled) return;
      settled = true;
      if (rosUdpSocket === socket) rosUdpSocket = null;
      try { socket.close(); } catch { /* 忽略 */ }
      reject(error);
    };

    socket.once("error", rejectStart);
    socket.once("listening", () => {
      if (settled) return;
      settled = true;
      socket.removeListener("error", rejectStart);
      const bound = socket.address();
      resolve({ address: bound.address, port: bound.port });
    });

    socket.on("message", (buffer, remote) => {
      if (allowedSource && remote.address !== allowedSource) return;
      if (!mainWindow || mainWindow.isDestroyed()) return;

      mainWindow.webContents.send("ros-udp:message", {
        text: buffer.toString("utf8"),
        bytes: buffer.length,
        address: remote.address,
        port: remote.port,
      });
    });

    socket.on("error", (error) => {
      if (!settled || !mainWindow || mainWindow.isDestroyed()) return;
      mainWindow.webContents.send("ros-udp:error", error.message);
    });

    try {
      socket.bind(port, address);
    } catch (error) {
      rejectStart(error);
    }
  });
}

ipcMain.handle("ros-udp:start", (_event, options) => startRosUdp(options));
ipcMain.handle("ros-udp:stop", () => {
  stopRosUdp();
  return true;
});

ipcMain.handle("agent:status", () => agentHost?.getStatus() || {
  state: "starting",
  message: "Agent 正在启动",
  busy: false,
  mcpConnected: false,
  hasApiKey: Boolean(process.env.DEEPSEEK_API_KEY),
});
ipcMain.handle("agent:set-context", (_event, context) => agentHost?.setContext(context || {}));
ipcMain.handle("agent:message", (_event, payload) => agentHost?.sendMessage(payload?.text));
ipcMain.handle("agent:cancel", () => agentHost?.cancel() || false);
ipcMain.handle("agent:stations", () => agentHost?.getStations() || { stations: [] });
ipcMain.handle("agent:update-station", (_event, payload) => agentHost?.updateStation(payload || {}));
ipcMain.handle("agent:add-station", (_event, payload) => agentHost?.addStation(payload || {}));
ipcMain.handle("agent:delete-station", (_event, payload) => agentHost?.deleteStation(payload || {}));
ipcMain.handle("agent:confirmation-response", (_event, payload) => (
  agentHost?.respondConfirmation(payload?.id, Boolean(payload?.accepted)) || false
));

async function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1600,
    height: 900,
    minWidth: 1200,
    minHeight: 700,
    autoHideMenuBar: true,
    backgroundColor: "#050910",
    webPreferences: {
      preload: path.join(__dirname, "preload.cjs"),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true,
    },
  });

  const simVehicleUrl = String(process.env.VEHICLE_SIM_URL || "").trim();
  const simQuery = simVehicleUrl ? `?ip=${encodeURIComponent(simVehicleUrl)}` : "";

  if (app.isPackaged) {
    const url = await startStaticServer();
    await mainWindow.loadURL(`${url}${simQuery}`);
  } else {
    await mainWindow.loadURL(`http://127.0.0.1:5173/${simQuery}`);
    mainWindow.webContents.openDevTools();
  }

  mainWindow.on("closed", () => {
    mainWindow = null;
    stopRosUdp();
  });
}

app.whenReady().then(async () => {
  agentHost = new VehicleAgentHost({ app, getMainWindow: () => mainWindow });
  await createWindow();
});

app.on("before-quit", () => {
  agentHost?.shutdown().catch(() => {});
});

app.on("window-all-closed", () => {
  stopRosUdp();
  staticServer?.close();
  app.quit();
});

