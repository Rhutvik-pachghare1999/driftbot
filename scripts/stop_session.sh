#!/usr/bin/env bash
# stop_session.sh — Stop all DriftBot laptop-side processes and tmux sessions.
#
# Usage:
#   ./scripts/stop_session.sh

set -euo pipefail

SESSION_NAME="driftbot"

echo "[stop_session] Stopping DriftBot session '${SESSION_NAME}'..."

# 1. Kill the main tmux session (this takes down agent, SLAM, recorder, etc.)
if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "[stop_session] Killed tmux session '${SESSION_NAME}'."
else
    echo "[stop_session] No tmux session named '${SESSION_NAME}' was running."
fi

# 2. Kill any stray processes that were launched outside the tmux session.
echo "[stop_session] Cleaning up stray processes..."
pkill -f 'micro_ros_agent' 2>/dev/null || true
pkill -f 'ros2 bag record' 2>/dev/null || true
pkill -f 'driftbot_bringup' 2>/dev/null || true   # scan_assembler, topic_monitor, odometry_node
pkill -f 'async_slam_toolbox_node' 2>/dev/null || true
pkill -f 'teleop_twist_keyboard' 2>/dev/null || true
pkill -f 'static_transform_publisher.*odom.*base_link' 2>/dev/null || true
pkill -f 'static_transform_publisher.*base_link.*base_scan' 2>/dev/null || true
pkill -f 'static_transform_publisher.*base_link.*imu_link' 2>/dev/null || true

echo "[stop_session] Done."
tmux ls 2>/dev/null | grep "^${SESSION_NAME}" || echo "[stop_session] No '${SESSION_NAME}' tmux session remains."
