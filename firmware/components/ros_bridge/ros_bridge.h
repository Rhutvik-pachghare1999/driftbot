/*
 * ros_bridge.h — micro-ROS Bridge Component
 *
 * Publishes all sensor data as ROS2 topics via micro-ROS WiFi UDP.
 *
 * Topics:
 *   /servo/position     (std_msgs/Float32)   — current servo angle (degrees)
 *   /tof/sensor_0       (sensor_msgs/Range)  — distance + true_angle in header
 *   /tof/sensor_1       (sensor_msgs/Range)  — distance + true_angle in header
 *   /tof/sensor_2       (sensor_msgs/Range)  — distance + true_angle in header
 *   /imu/data           (sensor_msgs/Imu)    — accel + gyro from MPU6050
 */

#ifndef ROS_BRIDGE_H
#define ROS_BRIDGE_H

#include <stdint.h>
#include "imu_driver.h"

// Initialize micro-ROS: WiFi, Agent, node, all publishers.
bool ros_init();

// Is micro-ROS connected?
bool ros_is_connected();

// Publish servo position (degrees).
void ros_publish_servo(float degrees);

// Publish ToF reading with true world angle.
// sensor_id: 0, 1, 2
// distance_mm: raw distance
// status: 0=valid
// true_angle_deg: actual world angle this beam points at
void ros_publish_tof(uint8_t sensor_id, uint16_t distance_mm, uint8_t status, float true_angle_deg);

// Publish IMU data.
void ros_publish_imu(const ImuData& data);

// Publish encoder ticks (cumulative tick counts).
void ros_publish_encoders(int32_t left_ticks, int32_t right_ticks);

// Process micro-ROS executor (handles incoming cmd_vel).
void ros_spin();

#endif // ROS_BRIDGE_H
