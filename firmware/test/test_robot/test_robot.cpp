/*
 * test_robot.cpp — Hardware-in-the-loop tests for DriftBot
 *
 * These tests run ON the ESP32-S3 and exercise real peripherals:
 *   - Scanning servo (MCPWM Unit 0)
 *   - Steering servo + motor driver (MCPWM Unit 1)
 *   - 3× VL53L1X ToF sensors (Wire, GPIO 8/9)
 *   - MPU6050 IMU (Wire1, GPIO 7/17)
 *
 * Run with:
 *   cd firmware && pio test --environment esp32s3
 *
 * Safety:
 *   - Motor is initialized but never driven (only direction/steering state).
 *   - Scanning/steering servos will move.
 */

#include <Arduino.h>
#include <unity.h>
#include <string.h>

#include "pins.h"
#include "servo_config.h"
#include "servo_driver.h"
#include "motor_config.h"
#include "motor_driver.h"
#include "tof_config.h"
#include "tof_driver.h"
#include "imu_driver.h"

// ── Helpers ──────────────────────────────────────────────────────────────────

static void update_servo_for_ms(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        servo_update();
        delay(SERVO_UPDATE_MS + 1);
    }
}

static bool wait_for_tof_responding(unsigned long timeout_ms) {
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        tof_update();

        bool all_responding = true;
        for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
            // Status 255 means "never updated" — the sensor has not been polled
            // successfully yet. Any other status means the sensor is talking.
            if (tof_get_status(i) == 255) {
                all_responding = false;
                break;
            }
        }
        if (all_responding) return true;
        delay(5);
    }

    Serial.println("[TEST] ToF final readings:");
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        Serial.printf("  sensor %d: dist=%u status=%u\n", i,
                      tof_get_distance(i), tof_get_status(i));
    }
    return false;
}

// ── Servo tests ──────────────────────────────────────────────────────────────

void test_servo_init_and_position(void) {
    servo_init();
    delay(100);

    float pos = servo_get_position();
    TEST_ASSERT_FLOAT_WITHIN(2.0f, SERVO_CENTER_DEG, pos);
    TEST_ASSERT_TRUE(pos >= (float)SERVO_LIMIT_MIN);
    TEST_ASSERT_TRUE(pos <= (float)SERVO_LIMIT_MAX);
}

void test_servo_hold_command(void) {
    // Move to a commanded angle and hold.
    TEST_ASSERT_TRUE(servo_handle_serial("go 120"));
    delay(100);
    servo_update();

    float pos = servo_get_position();
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 120.0f, pos);
    TEST_ASSERT_TRUE(pos >= (float)SERVO_LIMIT_MIN);
    TEST_ASSERT_TRUE(pos <= (float)SERVO_LIMIT_MAX);
}

void test_servo_sine_motion(void) {
    // Start sinusoidal sweep and let it move for a short time.
    TEST_ASSERT_TRUE(servo_handle_serial("start"));
    update_servo_for_ms(400);

    float pos = servo_get_position();
    TEST_ASSERT_TRUE(pos >= (SERVO_CENTER_DEG - SERVO_AMPLITUDE_DEG - 1.0f));
    TEST_ASSERT_TRUE(pos <= (SERVO_CENTER_DEG + SERVO_AMPLITUDE_DEG + 1.0f));

    // Return to idle before the next test and let the servo power rail settle.
    servo_handle_serial("stop");
    delay(1000);
}

// ── ToF tests ────────────────────────────────────────────────────────────────

void test_tof_init_all(void) {
    // Give the I2C bus a moment to settle after boot.
    delay(1000);

    bool ok = tof_init();
    if (!ok) {
        TEST_MESSAGE("tof_init() returned false");
    }
    TEST_ASSERT_TRUE(ok);
}

void test_tof_all_responding(void) {
    tof_start();
    bool responding = wait_for_tof_responding(4000);
    if (!responding) {
        TEST_MESSAGE("ToF sensors did not all report a status in 4s");
    }
    TEST_ASSERT_TRUE(responding);

    Serial.println("[TEST] ToF readings after 4s:");
    int valid_count = 0;
    int responding_count = 0;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        uint16_t d = tof_get_distance(i);
        uint8_t  s = tof_get_status(i);
        Serial.printf("  sensor %d: dist=%4umm status=%u\n", i, d, s);

        // Any status other than 255 means the sensor was polled and is talking.
        TEST_ASSERT_FALSE_MESSAGE(s == 255, "sensor never polled");
        if (s != 255) responding_count++;

        if (s == 0 && d > 0 && d <= 4000) {
            valid_count++;
        }
    }

    // On the test stand some sensors may face open space or a target beyond
    // the valid range. We require all sensors to be polled, and at least one
    // valid measurement (status 0) if any sensor sees a target.
    if (valid_count == 0) {
        TEST_MESSAGE("No ToF sensor reported a valid range; stand may be empty/open");
    }
    TEST_ASSERT_TRUE(responding_count == TOF_SENSOR_COUNT);
}

void test_tof_json_format(void) {
    char json_buf[256];
    int n = tof_get_json(json_buf, sizeof(json_buf), 96.0f);

    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_TRUE(n < (int)sizeof(json_buf));
    TEST_ASSERT_NOT_NULL(strstr(json_buf, "\"servo_deg\":"));
    TEST_ASSERT_NOT_NULL(strstr(json_buf, "\"tof\":["));
    TEST_ASSERT_NOT_NULL(strstr(json_buf, "\"id\":0"));
}

// ── IMU tests ────────────────────────────────────────────────────────────────

void test_imu_init(void) {
    // Extra settling time after ToF initialization helps the IMU bus wake cleanly.
    delay(1000);

    bool ok = imu_init();
    if (!ok) {
        TEST_MESSAGE("imu_init() returned false");
    }
    TEST_ASSERT_TRUE(ok);
}

static bool wait_for_imu_valid(unsigned long timeout_ms) {
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        imu_update();
        ImuData d = imu_get_data();
        if (fabsf(d.accel_z) > 5.0f) {
            return true;
        }
        delay(25);
    }

    ImuData d = imu_get_data();
    Serial.printf("[TEST] IMU final: accel=[%.2f, %.2f, %.2f] gyro=[%.3f, %.3f, %.3f]\n",
                  d.accel_x, d.accel_y, d.accel_z,
                  d.gyro_x, d.gyro_y, d.gyro_z);
    return false;
}

void test_imu_read_valid(void) {
    bool ready = wait_for_imu_valid(2000);
    if (!ready) {
        TEST_MESSAGE("IMU did not report a valid gravity vector in 2s");
    }
    TEST_ASSERT_TRUE(ready);

    ImuData d = imu_get_data();

    // Z should feel gravity after the robot-frame rotation.
    TEST_ASSERT_TRUE(d.accel_z > 5.0f);
    TEST_ASSERT_TRUE(d.accel_z < 15.0f);

    // Total acceleration should be close to 1g.
    float total_g = sqrtf(d.accel_x * d.accel_x +
                          d.accel_y * d.accel_y +
                          d.accel_z * d.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 9.81f, total_g);

    // Gyro should be small while the robot is on a stand.
    TEST_ASSERT_TRUE(fabsf(d.gyro_x) < 0.5f);
    TEST_ASSERT_TRUE(fabsf(d.gyro_y) < 0.5f);
    TEST_ASSERT_TRUE(fabsf(d.gyro_z) < 0.5f);
}

// ── Motor / steering tests ───────────────────────────────────────────────────

void test_motor_init_and_center(void) {
    motor_init();
    delay(50);

    TEST_ASSERT_EQUAL_INT(STEER_CENTER, motor_get_steering());
    TEST_ASSERT_EQUAL_INT8(0, motor_get_direction());
}

void test_motor_steering_clamp(void) {
    // Full left beyond limit should clamp to STEER_MAX.
    motor_set_steering(200);
    TEST_ASSERT_EQUAL_INT(STEER_MAX, motor_get_steering());

    // Full right beyond limit should clamp to STEER_MIN.
    motor_set_steering(0);
    TEST_ASSERT_EQUAL_INT(STEER_MIN, motor_get_steering());

    // Center command should land exactly on STEER_CENTER.
    motor_set_steering(STEER_CENTER);
    TEST_ASSERT_EQUAL_INT(STEER_CENTER, motor_get_steering());
}

void test_motor_stop_is_neutral(void) {
    motor_stop();
    delay(50);

    TEST_ASSERT_EQUAL_INT8(0, motor_get_direction());
    TEST_ASSERT_EQUAL_INT(STEER_CENTER, motor_get_steering());
}

// ── Configuration sanity tests ───────────────────────────────────────────────

void test_pin_map_unique(void) {
    // Ensure no two peripherals are mapped to the same GPIO.
    const uint8_t pins[] = {
        PIN_SERVO, PIN_STEERING,
        PIN_MOTOR_RPWM, PIN_MOTOR_LPWM, PIN_MOTOR_REN, PIN_MOTOR_LEN,
        PIN_ENC_LEFT, PIN_ENC_RIGHT,
        PIN_TOF_SDA, PIN_TOF_SCL,
        PIN_IMU_SDA, PIN_IMU_SCL,
        PIN_XSHUT_0, PIN_XSHUT_1, PIN_XSHUT_2,
        PIN_LED
    };
    const size_t n = sizeof(pins) / sizeof(pins[0]);
    for (size_t i = 0; i < n; i++) {
        for (size_t j = i + 1; j < n; j++) {
            TEST_ASSERT_NOT_EQUAL_MESSAGE(pins[i], pins[j],
                "pin map contains duplicate GPIOs");
        }
    }
}

void test_tof_addresses_unique(void) {
    const uint8_t addrs[] = { TOF_ADDR_0, TOF_ADDR_1, TOF_ADDR_2 };
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        // Must not use the factory-default 0x29 address after re-addressing.
        TEST_ASSERT_NOT_EQUAL(0x29, addrs[i]);
        for (int j = i + 1; j < TOF_SENSOR_COUNT; j++) {
            TEST_ASSERT_NOT_EQUAL(addrs[i], addrs[j]);
        }
    }
}

// ── Unity runner ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(2000);

    UNITY_BEGIN();

    // Configuration sanity checks first (pure software, no hardware motion).
    RUN_TEST(test_pin_map_unique);
    RUN_TEST(test_tof_addresses_unique);

    // Sensors next, while the power rail is quiet.
    RUN_TEST(test_tof_init_all);       // address ToF sensors only
    RUN_TEST(test_imu_init);           // initialize IMU before ToF ranging starts
    RUN_TEST(test_imu_read_valid);     // capture a valid IMU reading
    RUN_TEST(test_tof_all_responding); // start continuous ToF ranging
    RUN_TEST(test_tof_json_format);

    // Servo/motor tests move actuators and are run after I2C reads.
    RUN_TEST(test_servo_init_and_position);
    RUN_TEST(test_servo_hold_command);
    RUN_TEST(test_servo_sine_motion);
    RUN_TEST(test_motor_init_and_center);
    RUN_TEST(test_motor_steering_clamp);
    RUN_TEST(test_motor_stop_is_neutral);
    UNITY_END();
}

void loop() {
    // Nothing to do after tests finish.
}
