#!/usr/bin/env bash
set -euo pipefail

CONFIG_FILE="/etc/natfrp/config.json"
LOCK_FILE="/etc/natfrp/lock.pid"
SERVICE_BIN="/home/natfrp/natfrp-service"
SERVICE_WORKDIR="/etc/natfrp"
UDP_TUNNEL_ID=29625231
TCP_TUNNEL_ID=29625310

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用 sudo 运行此脚本。" >&2
  exit 1
fi

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "未找到 SakuraFrp 配置：${CONFIG_FILE}" >&2
  exit 1
fi

if [[ ! -x "${SERVICE_BIN}" ]]; then
  echo "未找到 SakuraFrp 服务程序：${SERVICE_BIN}" >&2
  exit 1
fi

if ! command -v jq >/dev/null 2>&1; then
  echo "缺少 jq，无法安全修改 JSON。" >&2
  exit 1
fi

BACKUP_FILE="${CONFIG_FILE}.backup-$(date +%Y%m%d-%H%M%S)"
TEMP_FILE="$(mktemp "${SERVICE_WORKDIR}/config.json.tmp.XXXXXX")"
trap 'rm -f "${TEMP_FILE}"' EXIT

cp -a -- "${CONFIG_FILE}" "${BACKUP_FILE}"

jq \
  --argjson udp_id "${UDP_TUNNEL_ID}" \
  --argjson tcp_id "${TCP_TUNNEL_ID}" \
  '.auto_start_tunnels = (((.auto_start_tunnels // []) + [$udp_id, $tcp_id]) | unique)' \
  "${CONFIG_FILE}" >"${TEMP_FILE}"

jq -e \
  --argjson udp_id "${UDP_TUNNEL_ID}" \
  --argjson tcp_id "${TCP_TUNNEL_ID}" \
  '(.auto_start_tunnels | index($udp_id)) != null and (.auto_start_tunnels | index($tcp_id)) != null' \
  "${TEMP_FILE}" >/dev/null

chown --reference="${CONFIG_FILE}" "${TEMP_FILE}"
chmod --reference="${CONFIG_FILE}" "${TEMP_FILE}"
mv -- "${TEMP_FILE}" "${CONFIG_FILE}"
trap - EXIT

echo "已保留原隧道并追加：UDP ${UDP_TUNNEL_ID}、TCP ${TCP_TUNNEL_ID}"
echo "原配置备份：${BACKUP_FILE}"

if [[ -r "${LOCK_FILE}" ]]; then
  OLD_PID="$(tr -cd '0-9' <"${LOCK_FILE}")"
else
  OLD_PID=""
fi

if [[ -n "${OLD_PID}" ]] && kill -0 "${OLD_PID}" 2>/dev/null; then
  echo "正在重启 SakuraFrp 服务，现有隧道会短暂中断……"
  kill -TERM "${OLD_PID}"
  for _ in {1..50}; do
    if ! kill -0 "${OLD_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.2
  done
  if kill -0 "${OLD_PID}" 2>/dev/null; then
    echo "SakuraFrp 服务未在 10 秒内退出；未强制终止，请手动重启服务。" >&2
    exit 2
  fi
fi

AUTO_PID=""
if [[ -r "${LOCK_FILE}" ]]; then
  AUTO_PID="$(tr -cd '0-9' <"${LOCK_FILE}")"
fi

if [[ -n "${AUTO_PID}" ]] && kill -0 "${AUTO_PID}" 2>/dev/null; then
  echo "SakuraFrp 已被系统自动重启，PID=${AUTO_PID}。"
else
  runuser -u natfrp -- env NATFRP_SERVICE_WD="${SERVICE_WORKDIR}" "${SERVICE_BIN}" --daemon
  sleep 3
fi

if [[ -r "${LOCK_FILE}" ]]; then
  NEW_PID="$(tr -cd '0-9' <"${LOCK_FILE}")"
else
  NEW_PID=""
fi

if [[ -z "${NEW_PID}" ]] || ! kill -0 "${NEW_PID}" 2>/dev/null; then
  echo "配置已更新，但 SakuraFrp 服务未成功启动。请检查 ${SERVICE_WORKDIR}/Logs/。" >&2
  exit 3
fi

echo "SakuraFrp 服务已启动，PID=${NEW_PID}。请等待约 10 秒后测试公网端口。"
