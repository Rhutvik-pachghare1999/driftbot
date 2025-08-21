#!/usr/bin/env bash
# record_mcap.sh — Record all DriftBot telemetry as an .mcap on the host laptop
#
# Usage (from ros2_ws/):
#   source install/setup.bash
#   ./src/driftbot_bringup/scripts/record_mcap.sh [bag_dir] [duration_seconds] [bag_name]
#
# Defaults:
#   bag_dir = ./bags
#   duration_seconds = 60
#   bag_name = driftbot_run_YYYYMMDD_HHMMSS
#
# Requires:
#   - micro_ros_agent package installed and sourced
#   - ros2 bag with the mcap storage plugin (`rosbag2_storage_mcap`)
#   - The robot is flashed with main firmware and on the same WiFi/UDP network
#
# The agent is started automatically on UDP port 8888 and killed when recording
# finishes (or when the script is interrupted).

set -euo pipefail

BAG_DIR="${1:-bags}"
DURATION="${2:-60}"
BAG_NAME="${3:-driftbot_run_$(date +%Y%m%d_%H%M%S)}"

# Ensure the workspace is sourced.
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "[record_mcap] ROS_DISTRO not set. Please source install/setup.bash first."
    exit 1
fi

mkdir -p "${BAG_DIR}"

# Start the micro-ROS agent in the background.
echo "[record_mcap] Starting micro-ROS agent on UDP port 8888..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
AGENT_PID=$!

# Clean up the agent when the script exits.
cleanup() {
    echo "[record_mcap] Stopping micro-ROS agent (PID ${AGENT_PID})..."
    kill "${AGENT_PID}" 2>/dev/null || true
    wait "${AGENT_PID}" 2>/dev/null || true
}
trap cleanup EXIT

# Give the agent a moment to bind before the robot connects.
sleep 2

echo "[record_mcap] Recording all topics for ${DURATION}s to ${BAG_DIR}/${BAG_NAME}.mcap"
# --duration stops automatically; --max-bag-size can be added if needed.
ros2 bag record \
    --storage mcap \
    --output "${BAG_DIR}/${BAG_NAME}" \
    --duration "${DURATION}" \
    -a

echo "[record_mcap] Done. Bag written to ${BAG_DIR}/${BAG_NAME}."
