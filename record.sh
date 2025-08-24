#!/bin/bash
# DriftBot Data Recorder — records all sensor + command data
# Usage: ./record.sh [optional_name]
#
# Saves to: ~/Drift_bot/bags/<timestamp>_<name>/

source /opt/ros/jazzy/setup.bash
source ~/Drift_bot/ros2_ws/install/setup.bash

NAME=${1:-drive_test}
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR=~/Drift_bot/bags/${TIMESTAMP}_${NAME}

mkdir -p ~/Drift_bot/bags

echo "╔══════════════════════════════════════════════╗"
echo "║  DriftBot Rosbag Recorder                   ║"
echo "╠══════════════════════════════════════════════╣"
echo "║  Saving to: bags/${TIMESTAMP}_${NAME}       ║"
echo "║  Press Ctrl+C to stop recording             ║"
echo "╠══════════════════════════════════════════════╣"
echo "║  Recording topics:                          ║"
echo "║    /cmd_vel          (your commands)        ║"
echo "║    /servo/position   (scanning servo)       ║"
echo "║    /encoder/left     (wheel ticks L)        ║"
echo "║    /encoder/right    (wheel ticks R)        ║"
echo "║    /tof/sensor_0     (ToF range 0)          ║"
echo "║    /tof/sensor_1     (ToF range 1)          ║"
echo "║    /tof/sensor_2     (ToF range 2)          ║"
echo "║    /imu/data         (IMU accel+gyro)       ║"
echo "║    /odom             (odometry)             ║"
echo "║    /odometry/filtered (EKF output)          ║"
echo "║    /scan             (360° laser scan)      ║"
echo "║    /tf               (transforms)           ║"
echo "║    /tf_static        (static transforms)    ║"
echo "╚══════════════════════════════════════════════╝"
echo ""
echo "Recording... drive the robot now!"
echo ""

ros2 bag record \
    /cmd_vel \
    /servo/position \
    /encoder/left \
    /encoder/right \
    /tof/sensor_0 \
    /tof/sensor_1 \
    /tof/sensor_2 \
    /imu/data \
    /odom \
    /odometry/filtered \
    /scan \
    /tf \
    /tf_static \
    -o "$BAG_DIR"

echo ""
echo "Recording saved to: $BAG_DIR"
echo "To replay: ros2 bag play $BAG_DIR"
echo "To analyze: ros2 bag info $BAG_DIR"
