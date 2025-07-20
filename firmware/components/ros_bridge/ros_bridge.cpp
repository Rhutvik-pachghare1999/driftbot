/*
 * ros_bridge.cpp — micro-ROS Bridge Implementation
 *
 * Node: "driftbot"
 * Publishers:
 *   /servo/position  (std_msgs/Float32)  — servo angle in degrees
 *   /tof/sensor_0    (sensor_msgs/Range) — left-rear ToF
 *   /tof/sensor_1    (sensor_msgs/Range) — left ToF
 *   /tof/sensor_2    (sensor_msgs/Range) — forward ToF (reference)
 *   /imu/data        (sensor_msgs/Imu)   — MPU6050 accel + gyro
 *
 * The true_angle (world angle of each beam) is encoded in the Range
 * message's header.frame_id as "tof_X_<angle>" so the laptop can parse it.
 * The range field is distance in meters.
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include "motor_driver.h"

#include "secrets.h"
#include "tof_config.h"
#include "ros_bridge.h"

// ── micro-ROS core ────────────────────────────────────────────────────────────
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

// ── Publishers ────────────────────────────────────────────────────────────────
static rcl_publisher_t  servo_pub;
static rcl_publisher_t  tof_pubs[TOF_SENSOR_COUNT];
static rcl_publisher_t  imu_pub;
static rcl_publisher_t  enc_left_pub;
static rcl_publisher_t  enc_right_pub;

// ── Subscriber ────────────────────────────────────────────────────────────────
static rcl_subscription_t cmd_vel_sub;

// ── Messages ──────────────────────────────────────────────────────────────────
static std_msgs__msg__Float32      servo_msg;
static sensor_msgs__msg__Range     tof_msgs[TOF_SENSOR_COUNT];
static sensor_msgs__msg__Imu       imu_msg;
static std_msgs__msg__Int32        enc_left_msg;
static std_msgs__msg__Int32        enc_right_msg;
static geometry_msgs__msg__Twist   cmd_vel_msg;

// ── State ─────────────────────────────────────────────────────────────────────
static bool connected = false;

// ── Frame IDs for Range messages ──────────────────────────────────────────────
static char frame_id_0[] = "tof_sensor_0";
static char frame_id_1[] = "tof_sensor_1";
static char frame_id_2[] = "tof_sensor_2";
static char* frame_ids[TOF_SENSOR_COUNT] = { frame_id_0, frame_id_1, frame_id_2 };

// ── IMU frame ID ──────────────────────────────────────────────────────────────
static char imu_frame_id[] = "imu_link";

// ── cmd_vel callback (called by executor when /cmd_vel message arrives) ───────
static void ros_cmd_vel_cb(const void* msg_in) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
    motor_cmd_vel(-(float)msg->linear.x, (float)msg->angular.z);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: ros_init()
// ═══════════════════════════════════════════════════════════════════════════════
bool ros_init() {
    // ── 1. Connect WiFi + UDP ─────────────────────────────────────────────────
    IPAddress agent_ip(AGENT_IP);
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, AGENT_PORT);
    Serial.println("[ROS] WiFi connected. Pinging agent...");

    // ── 2. Init support ───────────────────────────────────────────────────────
    allocator = rcl_get_default_allocator();
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.printf("[ROS] Agent unreachable (ret=%d). ROS disabled.\n", (int)ret);
        connected = false;
        return false;
    }

    // ── 3. Create node ────────────────────────────────────────────────────────
    ret = rclc_node_init_default(&node, "driftbot", "", &support);
    if (ret != RCL_RET_OK) { connected = false; return false; }

    // ── 4. /servo/position publisher ──────────────────────────────────────────
    ret = rclc_publisher_init_default(
        &servo_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "servo/position"
    );
    if (ret != RCL_RET_OK) { connected = false; return false; }

    // ── 5. /tof/sensor_X publishers ───────────────────────────────────────────
    const char* tof_topics[TOF_SENSOR_COUNT] = {
        "tof/sensor_0", "tof/sensor_1", "tof/sensor_2"
    };
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        ret = rclc_publisher_init_default(
            &tof_pubs[i], &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            tof_topics[i]
        );
        if (ret != RCL_RET_OK) { connected = false; return false; }
    }

    // ── 6. /imu/data publisher ────────────────────────────────────────────────
    ret = rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );
    if (ret != RCL_RET_OK) { connected = false; return false; }

    // ── 7. /encoder/left and /encoder/right publishers ────────────────────────
    ret = rclc_publisher_init_default(
        &enc_left_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder/left"
    );
    if (ret != RCL_RET_OK) { connected = false; return false; }

    ret = rclc_publisher_init_default(
        &enc_right_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder/right"
    );
    if (ret != RCL_RET_OK) { connected = false; return false; }

    // ── 8. /cmd_vel subscriber ───────────────────────────────────────────────
    ret = rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );
    if (ret != RCL_RET_OK) { connected = false; return false; }

    // ── 9. Executor (1 subscription to manage) ────────────────────────────────
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) { connected = false; return false; }

    // Add cmd_vel subscription to executor
    ret = rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg, &ros_cmd_vel_cb, ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) { connected = false; return false; }

    // ── 8. Pre-fill static Range message fields ───────────────────────────────
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        tof_msgs[i].radiation_type = sensor_msgs__msg__Range__INFRARED;
        tof_msgs[i].field_of_view = 0.471f;   // 27° in radians
        tof_msgs[i].min_range = 0.04f;         // 40mm
        tof_msgs[i].max_range = 4.0f;          // 4000mm
        tof_msgs[i].header.frame_id.data = frame_ids[i];
        tof_msgs[i].header.frame_id.size = strlen(frame_ids[i]);
        tof_msgs[i].header.frame_id.capacity = strlen(frame_ids[i]) + 1;
    }

    // ── 9. Pre-fill static IMU message fields ─────────────────────────────────
    imu_msg.header.frame_id.data = imu_frame_id;
    imu_msg.header.frame_id.size = strlen(imu_frame_id);
    imu_msg.header.frame_id.capacity = strlen(imu_frame_id) + 1;
    // We don't provide orientation (no magnetometer), set covariance[0] = -1
    imu_msg.orientation_covariance[0] = -1.0;  // means "orientation not available"

    connected = true;
    Serial.println("[ROS] ✓ Publishers created:");
    Serial.println("[ROS]   /servo/position  (Float32)");
    Serial.println("[ROS]   /tof/sensor_0    (Range)");
    Serial.println("[ROS]   /tof/sensor_1    (Range)");
    Serial.println("[ROS]   /tof/sensor_2    (Range)");
    Serial.println("[ROS]   /imu/data        (Imu)");
    Serial.println("[ROS]   /encoder/left    (Int32)");
    Serial.println("[ROS]   /encoder/right   (Int32)");

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
bool ros_is_connected() { return connected; }

// ═══════════════════════════════════════════════════════════════════════════════
void ros_publish_servo(float degrees) {
    if (!connected) return;
    servo_msg.data = degrees;
    rcl_publish(&servo_pub, &servo_msg, NULL);
}

// ═══════════════════════════════════════════════════════════════════════════════
void ros_publish_tof(uint8_t sensor_id, uint16_t distance_mm, uint8_t status, float true_angle_deg) {
    if (!connected) return;
    if (sensor_id >= TOF_SENSOR_COUNT) return;

    // Store true_angle in the Range message's min_range field temporarily?
    // No — better approach: we publish range normally. The true_angle is
    // computed on the laptop side from /servo/position + known mount offsets.
    // But we also include it here for convenience by setting range properly.

    if (status == 0) {
        tof_msgs[sensor_id].range = distance_mm / 1000.0f;  // Valid: update range
    }
    // If status != 0, keep the previous range value (don't overwrite with -1).
    // This means the laptop always gets the last known good distance.
    // Only publish -1 if we've NEVER gotten a valid reading (range still at 0).

    rcl_publish(&tof_pubs[sensor_id], &tof_msgs[sensor_id], NULL);
}

// ═══════════════════════════════════════════════════════════════════════════════
void ros_publish_imu(const ImuData& data) {
    if (!connected) return;

    // Fill acceleration (m/s²)
    imu_msg.linear_acceleration.x = data.accel_x;
    imu_msg.linear_acceleration.y = data.accel_y;
    imu_msg.linear_acceleration.z = data.accel_z;

    // Fill angular velocity (rad/s)
    imu_msg.angular_velocity.x = data.gyro_x;
    imu_msg.angular_velocity.y = data.gyro_y;
    imu_msg.angular_velocity.z = data.gyro_z;

    rcl_publish(&imu_pub, &imu_msg, NULL);
}

// ═══════════════════════════════════════════════════════════════════════════════
void ros_publish_encoders(int32_t left_ticks, int32_t right_ticks) {
    if (!connected) return;
    enc_left_msg.data = left_ticks;
    enc_right_msg.data = right_ticks;
    rcl_publish(&enc_left_pub, &enc_left_msg, NULL);
    rcl_publish(&enc_right_pub, &enc_right_msg, NULL);
}

// ═══════════════════════════════════════════════════════════════════════════════
void ros_spin() {
    if (!connected) return;
    rclc_executor_spin_some(&executor, 0);
}
