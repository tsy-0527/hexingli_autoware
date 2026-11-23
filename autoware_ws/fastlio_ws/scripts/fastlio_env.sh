#!/usr/bin/env bash

# Shared ROS 2 / FAST-LIO environment.
# Source this file from the other scripts; do not run it as the main command.

FASTLIO_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export FASTLIO_WS_DIR

source /opt/ros/humble/setup.bash
source /home/tang/livox_ws/install/setup.bash
source "${FASTLIO_WS_DIR}/install/setup.bash"

# /etc/profile 和 ~/.profile 里海康安装器写入的 MVS 路径仍在（登录 shell 会注入）。
# MVS 自带的旧 libusb-1.0.so.0 缺少 libusb_set_option，会破坏系统 PCL (libpcl_io)，
# 导致 fastlio_mapping 启动即崩溃 (exit 127)。这里强制剔除，保证动态库环境干净。
# 注意：00_open_fastlio_terminals.sh 用 bash -lc 启动终端，~/.bashrc 的剔除逻辑不会生效，
# 所以必须在 fastlio_env.sh 里做这一步。
export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)"

mkdir -p "${FASTLIO_WS_DIR}/Log" "${FASTLIO_WS_DIR}/bags"
