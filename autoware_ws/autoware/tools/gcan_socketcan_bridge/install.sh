#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run this installer as root: sudo $0 [runtime-user] [single|dual|usbcan2]" >&2
  exit 1
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
runtime_user=${1:-${SUDO_USER:-tang}}
install_mode=${2:-single}

if [[ "$install_mode" != "single" && "$install_mode" != "dual" && "$install_mode" != "usbcan2" ]]; then
  echo "Usage: sudo $0 [runtime-user] [single|dual|usbcan2]" >&2
  exit 1
fi

if ! id "$runtime_user" >/dev/null 2>&1; then
  echo "Runtime user does not exist: $runtime_user" >&2
  exit 1
fi
if [[ ! -x "$script_dir/gcan_socketcan_bridge" ]]; then
  echo "Bridge is not built; run make first." >&2
  exit 1
fi

getent group plugdev >/dev/null || groupadd --system plugdev
usermod -a -G plugdev "$runtime_user"

install -d -m 0755 /opt/gcan-socketcan-bridge/vendor
install -m 0755 "$script_dir/gcan_socketcan_bridge" /opt/gcan-socketcan-bridge/
install -m 0644 "$script_dir/vendor/libECanVci.so.1" /opt/gcan-socketcan-bridge/vendor/
install -m 0644 "$script_dir/vendor/libusb.so" /opt/gcan-socketcan-bridge/vendor/
ln -sfn libECanVci.so.1 /opt/gcan-socketcan-bridge/vendor/libECanVci.so
ln -sfn libusb.so /opt/gcan-socketcan-bridge/vendor/libusb-0.1.so.4
ln -sfn libusb.so /opt/gcan-socketcan-bridge/vendor/libusb-0.1.so

install -m 0755 "$script_dir/systemd/gcan-can0-setup" /usr/local/sbin/
install -m 0755 "$script_dir/systemd/gcan-can1-setup" /usr/local/sbin/

systemctl disable --now gcan-socketcan-bridge.service 2>/dev/null || true
systemctl disable --now gcan-socketcan-control.service 2>/dev/null || true
systemctl disable --now gcan-usbcan2-bridge.service 2>/dev/null || true
rm -f /etc/systemd/system/gcan-socketcan-bridge.service \
  /etc/systemd/system/gcan-socketcan-control.service \
  /etc/systemd/system/gcan-usbcan2-bridge.service

if [[ "$install_mode" == "usbcan2" ]]; then
  sed "s/@RUNTIME_USER@/$runtime_user/g" \
    "$script_dir/systemd/gcan-usbcan2-bridge.service.in" \
    > /etc/systemd/system/gcan-usbcan2-bridge.service
  chmod 0644 /etc/systemd/system/gcan-usbcan2-bridge.service
else
  sed "s/@RUNTIME_USER@/$runtime_user/g" \
    "$script_dir/systemd/gcan-socketcan-bridge.service.in" \
    > /etc/systemd/system/gcan-socketcan-bridge.service
  chmod 0644 /etc/systemd/system/gcan-socketcan-bridge.service
fi

if [[ "$install_mode" == "dual" ]]; then
  sed "s/@RUNTIME_USER@/$runtime_user/g" \
    "$script_dir/systemd/gcan-socketcan-control.service.in" \
    > /etc/systemd/system/gcan-socketcan-control.service
  chmod 0644 /etc/systemd/system/gcan-socketcan-control.service
fi

cat > /etc/udev/rules.d/50-gcan-usbcan.rules <<'RULE'
SUBSYSTEM=="usb", ATTR{product}=="USBCAN*", GROUP="plugdev", MODE="0660"
RULE

udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=usb
systemctl daemon-reload
if [[ "$install_mode" == "usbcan2" ]]; then
  systemctl enable --now gcan-usbcan2-bridge.service
else
  systemctl enable --now gcan-socketcan-bridge.service
fi
if [[ "$install_mode" == "dual" ]]; then
  systemctl enable --now gcan-socketcan-control.service
fi

if [[ "$install_mode" == "usbcan2" ]]; then
  echo "Installed. Check with: systemctl status gcan-usbcan2-bridge.service"
  echo "USBCAN-II channels: ip -details link show can0; ip -details link show can1"
else
  echo "Installed. Check with: systemctl status gcan-socketcan-bridge.service"
  echo "SocketCAN interface: ip -details link show can0"
fi
if [[ "$install_mode" == "dual" ]]; then
  echo "Control bridge: systemctl status gcan-socketcan-control.service"
  echo "Autonomous-control interface: ip -details link show can1"
fi
