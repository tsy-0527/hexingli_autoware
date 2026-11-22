import { useEffect, useState } from "react";
import App from "./App.jsx";            // 监控界面（数字孪生）
import OpsReport from "./OpsReport.jsx"; // 运营报表

// 页面路由：按网址 # 后面的路径切换页面
//   http://上位机地址/          → 监控界面（App.jsx）
//   http://上位机地址/#/report  → 运营报表（OpsReport.jsx）
// 两个页面各自独立维护；监控界面里的“运营报表”按钮会在新标签页打开报表，监控不中断。
function currentRoute() {
  return window.location.hash.replace(/^#\/?/, "").split("?")[0] || "monitor";
}

export default function PageRouter() {
  const [route, setRoute] = useState(currentRoute);
  useEffect(() => {
    const onHash = () => setRoute(currentRoute());
    window.addEventListener("hashchange", onHash);
    return () => window.removeEventListener("hashchange", onHash);
  }, []);
  if (route === "report") return <OpsReport />;
  return <App />;
}
