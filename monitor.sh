#!/bin/bash
# DriftBot Live Monitor — shows all sensor data + commands in real-time
source /opt/ros/jazzy/setup.bash
source ~/Drift_bot/ros2_ws/install/setup.bash

echo "═══════════════════════════════════════════════════════"
echo "  DriftBot Live Monitor — Ctrl+C to exit"
echo "═══════════════════════════════════════════════════════"
echo ""

# Run multiple echo streams in background, prefix each line
(timeout 300 ros2 topic echo /servo/position --field data 2>/dev/null | while read line; do echo "[SERVO] pos=$line°"; done) &
(timeout 300 ros2 topic echo /encoder/left --field data 2>/dev/null | while read line; do echo "[ENC_L] ticks=$line"; done) &
(timeout 300 ros2 topic echo /encoder/right --field data 2>/dev/null | while read line; do echo "[ENC_R] ticks=$line"; done) &
(timeout 300 ros2 topic echo /cmd_vel 2>/dev/null | grep -A2 "linear:\|angular:" | while read line; do echo "[CMD] $line"; done) &
(timeout 300 ros2 topic echo /odom --field pose.pose.position 2>/dev/null | paste -d' ' - - - - | while read line; do echo "[ODOM] $line"; done) &
(timeout 300 ros2 topic echo /scan --field header.stamp 2>/dev/null | while read line; do echo "[SCAN] published"; done) &

wait
