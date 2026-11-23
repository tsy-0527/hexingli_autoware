#!/usr/bin/env bash
set -euo pipefail

gateway_binary=/tmp/yunle-can-gateway-build/yunle_can_gateway
gateway_dir=/home/tang/autoware/tools/yunle_can_gateway

if [[ ! -x "$gateway_binary" ]]; then
  echo "Gateway binary not found: $gateway_binary" >&2
  exit 1
fi

install -m 0755 "$gateway_binary" /usr/local/bin/yunle_can_gateway
install -m 0644 "$gateway_dir/systemd/yunle-can-gateway.service" \
  /etc/systemd/system/yunle-can-gateway.service
install -m 0600 "$gateway_dir/systemd/yunle-can-gateway.default" \
  /etc/default/yunle-can-gateway

# Replace the manually launched temporary instance before binding port 8765.
pkill -TERM -f '^/tmp/yunle-can-gateway-build/yunle_can_gateway( |$)' || true

systemctl daemon-reload
systemctl enable yunle-can-gateway.service
systemctl restart yunle-can-gateway.service
systemctl --no-pager --full status yunle-can-gateway.service
