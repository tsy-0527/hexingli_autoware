#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_file="${repo_root}/src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml"

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <backup-directory>" >&2
  echo "Example: $0 backups/cost_aware_integration/20260916_170608" >&2
  exit 2
fi

backup_file="${repo_root}/$1/behavior_planning.launch.xml"
if [[ ! -f "${backup_file}" ]]; then
  echo "Backup file not found: ${backup_file}" >&2
  exit 1
fi

if cmp -s "${backup_file}" "${target_file}"; then
  echo "Already restored: ${target_file}"
  exit 0
fi

cp -p "${backup_file}" "${target_file}"
echo "Restored: ${target_file}"
echo "The install path is a symlink in this workspace; no install copy is required."
echo "Restart the planning launch for the restored remap to take effect."
