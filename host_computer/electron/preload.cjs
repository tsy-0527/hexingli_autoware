const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("rosUdp", {
  start: (options) => ipcRenderer.invoke("ros-udp:start", options),
  stop: () => ipcRenderer.invoke("ros-udp:stop"),
  onMessage: (callback) => {
    const handler = (_event, packet) => callback(packet);
    ipcRenderer.on("ros-udp:message", handler);
    return () => ipcRenderer.removeListener("ros-udp:message", handler);
  },
  onError: (callback) => {
    const handler = (_event, message) => callback(message);
    ipcRenderer.on("ros-udp:error", handler);
    return () => ipcRenderer.removeListener("ros-udp:error", handler);
  },
});

contextBridge.exposeInMainWorld("vehicleAgent", {
  getStatus: () => ipcRenderer.invoke("agent:status"),
  setContext: (context) => ipcRenderer.invoke("agent:set-context", context),
  sendMessage: (text) => ipcRenderer.invoke("agent:message", { text }),
  cancel: () => ipcRenderer.invoke("agent:cancel"),
  getStations: () => ipcRenderer.invoke("agent:stations"),
  updateStation: (station) => ipcRenderer.invoke("agent:update-station", station),
  addStation: (station) => ipcRenderer.invoke("agent:add-station", station),
  deleteStation: (id) => ipcRenderer.invoke("agent:delete-station", { id }),
  respondConfirmation: (id, accepted) => (
    ipcRenderer.invoke("agent:confirmation-response", { id, accepted })
  ),
  onEvent: (callback) => {
    const handler = (_event, payload) => callback(payload);
    ipcRenderer.on("agent:event", handler);
    return () => ipcRenderer.removeListener("agent:event", handler);
  },
});
