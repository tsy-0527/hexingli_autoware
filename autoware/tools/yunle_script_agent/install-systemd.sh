#!/usr/bin/env bash
set -euo pipefail

agent_binary=/tmp/yunle-script-agent-build/yunle_script_agent
agent_dir=/home/tang/autoware/tools/yunle_script_agent

if [[ ! -x "$agent_binary" ]]; then
  echo "Script agent binary not found: $agent_binary" >&2
  exit 1
fi

install -m 0755 "$agent_binary" /usr/local/bin/yunle_script_agent
install -m 0644 "$agent_dir/systemd/yunle-script-agent.service" \
  /etc/systemd/system/yunle-script-agent.service
install -m 0600 "$agent_dir/systemd/yunle-script-agent.default" \
  /etc/default/yunle-script-agent

systemctl daemon-reload
systemctl enable yunle-script-agent.service
systemctl restart yunle-script-agent.service
systemctl --no-pager --full status yunle-script-agent.service

