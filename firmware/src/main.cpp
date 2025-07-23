/*
 * main.cpp — DriftBot Firmware (Dual-Core)
 *
 * Core 0 (Arduino loop): Steering servo + Motor + Serial commands
 * Core 1 (FreeRTOS task): Sweep servo + WiFi/ROS + Sensors + Publishing
 *
 * Key insight: Steering servo works alone on Core 0 without WiFi.
 *              Sweep servo inits before WiFi on Core 1, reinits after WiFi if corrupted.
 */

#include <Arduino.h>
#include <Wire.h>
#include "servo_driver.h"
#include "tof_driver.h"
#include "imu_driver.h"
/* Encoder driver kept in tree but disabled — only one encoder is reliable.
#include "encoder_driver.h"
*/
#include "motor_driver.h"
#include "ros_bridge.h"
#include "tof_config.h"

// ── Sensor mount angles ───────────────────────────────────────────────────────
static const float mount_offsets[TOF_SENSOR_COUNT] = {
    TOF_ANGLE_0, TOF_ANGLE_1, TOF_ANGLE_2
};

// ── Serial input buffer ───────────────────────────────────────────────────────
static char cmd_buf[256];
static int  cmd_len = 0;

// ── Publish timing ────────────────────────────────────────────────────────────
#define SCAN_PUBLISH_MS    100    // 10Hz
#define IMU_PUBLISH_MS     100    // 10Hz
static unsigned long last_scan_publish = 0;
static unsigned long last_imu_publish = 0;

// ── FreeRTOS task handle ──────────────────────────────────────────────────────
static TaskHandle_t core1_task_handle = NULL;

// ── ESDS JSON motor command parser ────────────────────────────────────────────
// Accepts: {"cmd_id":N,"motor_left":F,"motor_right":F,"duration_ms":N,...}
// Converts differential (left/right) to cmd_vel (linear/angular).
static bool handle_esds_json(const char* cmd) {
    if (cmd[0] != '{') return false;

    float motor_left = 0.0f, motor_right = 0.0f;
    int cmd_id = -1;
    bool found_left = false, found_right = false;
    const char* p;

    p = strstr(cmd, "\"motor_left\"");
    if (p) { p = strchr(p, ':'); if (p) { motor_left = atof(p + 1); found_left = true; } }

    p = strstr(cmd, "\"motor_right\"");
    if (p) { p = strchr(p, ':'); if (p) { motor_right = atof(p + 1); found_right = true; } }

    p = strstr(cmd, "\"cmd_id\"");
    if (p) { p = strchr(p, ':'); if (p) { cmd_id = atoi(p + 1); } }

    if (!found_left || !found_right) return false;

    if (motor_left > 1.0f) motor_left = 1.0f;
    if (motor_left < -1.0f) motor_left = -1.0f;
    if (motor_right > 1.0f) motor_right = 1.0f;
    if (motor_right < -1.0f) motor_right = -1.0f;

    // Differential → twist: linear = avg speed, angular = difference
    float linear_x = (motor_left + motor_right) / 2.0f * 0.5f;
    float angular_z = (motor_right - motor_left) / 2.0f * 1.0f;

    motor_cmd_vel(linear_x, angular_z);

    Serial.printf("{\"ack\":%d,\"ml\":%.2f,\"mr\":%.2f}\n", cmd_id, motor_left, motor_right);
    return true;
}

// ── Serial dispatch ───────────────────────────────────────────────────────────
static void dispatch_command(const char* cmd) {
    if (handle_esds_json(cmd)) return;
    if (servo_handle_serial(cmd)) return;
    if (tof_handle_serial(cmd)) return;
    if (imu_handle_serial(cmd)) return;
    /* Encoder serial handler disabled with encoder module.
    if (encoder_handle_serial(cmd)) return;
    */
    if (motor_handle_serial(cmd)) return;

    Serial.println("\n── Commands ──────────────────────────────");
    servo_print_help();
    Serial.println("  ────────────────────────");
    tof_print_help();
    Serial.println("  ────────────────────────");
    imu_print_help();
    Serial.println("  ────────────────────────");
    /* encoder_print_help();  // disabled — encoder not in use */
    Serial.println("  ────────────────────────");
    motor_print_help();
    Serial.println("  ────────────────────────");
    Serial.println("  help         Show this");
    Serial.println("──────────────────────────────────────────\n");
}

// ══════════════════════════════════════════════════════════════════════════════
// CORE 1: WiFi + ROS + Sweep Servo + Sensors + Publishing
// ══════════════════════════════════════════════════════════════════════════════
static void core1_task(void* param) {
    (void)param;

    // ── Step 1: Sweep servo FIRST (before WiFi) ───────────────────────────────
    Serial.println("[CORE1] Sweep servo init...");
    servo_init();
    servo_handle_serial("start");
    delay(500);

    // ── Step 2: WiFi + micro-ROS ──────────────────────────────────────────────
    Serial.println("[CORE1] WiFi + micro-ROS...");
    ros_init();
    delay(500);

    // ── Step 3: Reinit sweep servo in case WiFi corrupted it ──────────────────
    Serial.println("[CORE1] Sweep servo reinit...");
    servo_init();
    servo_handle_serial("start");
    delay(200);

    // ── Step 4: Sensors ───────────────────────────────────────────────────────
    // ToF sensors live on I2C bus 0 (Wire, GPIO 8/9). The IMU lives on its own
    // dedicated I2C bus 1 (Wire1, GPIO 7/17) so it cannot be starved by ToF traffic.
    Serial.println("[CORE1] ToF init...");
    tof_init();
    delay(200);

    Serial.println("[CORE1] IMU init...");
    imu_init();
    delay(200);

    // Now that IMU is ready, start the ToF continuous ranging loop and tighten
    // the I2C timeout for runtime.
    Serial.println("[CORE1] ToF start ranging...");
    tof_start();
    delay(100);

    /* Encoders disabled — only one wheel encoder is reliable.
    Serial.println("[CORE1] Encoders init...");
    encoder_init();
    delay(100);

    encoder_reset();
    */
    Serial.println("\n[READY] All systems active.\n");

    // ── Core 1 Loop: sweep servo + publishing + ros_spin ──────────────────────
    for (;;) {
        servo_update();
        tof_update();
        imu_update();

        unsigned long now = millis();

        // Publish servo + ToF at 10Hz
        if (now - last_scan_publish >= SCAN_PUBLISH_MS) {
            last_scan_publish = now;
            float servo_pos = servo_get_position();
            ros_publish_servo(servo_pos);

            for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
                float true_angle = (servo_pos - 96.0f) + mount_offsets[i];
                if (true_angle < 0) true_angle += 360.0f;
                if (true_angle >= 360.0f) true_angle -= 360.0f;
                ros_publish_tof(i, tof_get_distance(i), tof_get_status(i), true_angle);
            }
        }

        // Publish IMU + encoders at 10Hz
        if (now - last_imu_publish >= IMU_PUBLISH_MS) {
            last_imu_publish = now;
            ros_publish_imu(imu_get_data());
            /* Encoder publishing disabled — encoders not in use.
            ros_publish_encoders(encoder_get_left(), encoder_get_right());
            */
        }

        // Process incoming cmd_vel
        ros_spin();

        vTaskDelay(1);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP (Core 0)
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("\n═══ DriftBot Firmware (Dual-Core) ═══\n");

    // ── Core 0: Motor + Steering (isolated from WiFi) ─────────────────────────
    Serial.println("[CORE0] Motor + steering init...");
    motor_init();
    delay(300);

    // ── Launch Core 1 ─────────────────────────────────────────────────────────
    xTaskCreatePinnedToCore(core1_task, "Core1", 8192, NULL, 1, &core1_task_handle, 1);
}

// ═══════════════════════════════════════════════════════════════════════════════
// LOOP (Core 0): Steering + Motor control + Serial
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    motor_update();
    motor_update_steering_hw();

    // Serial commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                dispatch_command(cmd_buf);
                cmd_len = 0;
            }
        } else if (cmd_len < (int)sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = c;
        }
    }
}
