/*
 * test_unit_logic.cpp — Pure-math unit tests (run on host, no hardware)
 *
 * Tests the shared math in components/common/driftbot_math.h — the exact
 * expressions the drivers use on the robot:
 *   - servo/steering angle -> pulse mapping + clamping
 *   - angular_z -> steering angle, speed -> PWM mapping (deadband, limits)
 *   - IMU raw -> SI conversion, gyro bias, sensor->robot rotation
 *   - encoder signed delta (direction + uint32 wraparound)
 *   - cmd_vel watchdog timeout
 *
 * Run on the host:
 *   cd firmware && pio test -e native
 *
 * The same suite also compiles for the ESP32-S3 (pio test -e esp32s3),
 * where it runs before the hardware-in-the-loop tests.
 */

#include <unity.h>
#include <math.h>

#include "driftbot_math.h"
#include "servo_config.h"
#include "motor_config.h"

// ── Scanning servo: clamp + pulse mapping ────────────────────────────────────

void test_servo_clamp_deg(void) {
    // Mechanical limits: SERVO_LIMIT_MIN..SERVO_LIMIT_MAX (0..180)
    TEST_ASSERT_EQUAL_FLOAT(0.0f,  db_servo_clamp_deg(-10.0f));
    TEST_ASSERT_EQUAL_FLOAT(180.0f, db_servo_clamp_deg(200.0f));
    TEST_ASSERT_EQUAL_FLOAT(90.0f,  db_servo_clamp_deg(90.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f,   db_servo_clamp_deg(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(180.0f, db_servo_clamp_deg(180.0f));
}

void test_servo_pulse_us(void) {
    // Standard servo map: 500us @ 0deg, 2500us @ 180deg, 1500us @ 90deg
    TEST_ASSERT_EQUAL_FLOAT(500.0f,  db_servo_pulse_us(0));
    TEST_ASSERT_EQUAL_FLOAT(1500.0f, db_servo_pulse_us(90));
    TEST_ASSERT_EQUAL_FLOAT(2500.0f, db_servo_pulse_us(180));
    // Calibrated sweep center 96deg -> 500 + 96*2000/180 = 1566.67us
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1566.6667f, db_servo_pulse_us(96));
    // Sweep extremes 96±60 = [36, 156] stay inside the servo pulse range
    TEST_ASSERT_TRUE(db_servo_pulse_us(36)  > 500.0f && db_servo_pulse_us(36)  < 2500.0f);
    TEST_ASSERT_TRUE(db_servo_pulse_us(156) > 500.0f && db_servo_pulse_us(156) < 2500.0f);
}

void test_servo_sweep_bounds(void) {
    // Calibrated sweep: center 96, amplitude 60 -> [36, 156] deg
    // (3 beams x 120deg arcs = 360deg coverage)
    TEST_ASSERT_EQUAL_FLOAT(96.0f, db_servo_sweep_deg(96.0f, 60.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 156.0f, db_servo_sweep_deg(96.0f, 60.0f, (float)(M_PI / 2)));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 36.0f,  db_servo_sweep_deg(96.0f, 60.0f, (float)(-M_PI / 2)));
    // Full sine sweep must stay inside the mechanical limits at any phase
    for (float phase = 0.0f; phase < 6.2832f; phase += 0.01f) {
        float angle = db_servo_sweep_deg(96.0f, 60.0f, phase);
        TEST_ASSERT_TRUE(angle >= (float)SERVO_LIMIT_MIN && angle <= (float)SERVO_LIMIT_MAX);
    }
}

// ── Steering: clamp, pulse map, angular_z -> angle ────────────────────────────

void test_steer_clamp(void) {
    // Calibrated range: STEER_MIN..STEER_MAX (125..155), center 140
    TEST_ASSERT_EQUAL_INT(STEER_MIN,   db_steer_clamp(0));
    TEST_ASSERT_EQUAL_INT(STEER_MAX,   db_steer_clamp(180));
    TEST_ASSERT_EQUAL_INT(STEER_CENTER, db_steer_clamp(140));
    TEST_ASSERT_EQUAL_INT(140, db_steer_clamp(140));
    TEST_ASSERT_EQUAL_INT(125, db_steer_clamp(125));
    TEST_ASSERT_EQUAL_INT(155, db_steer_clamp(155));
    TEST_ASSERT_EQUAL_INT(125, db_steer_clamp(-5));
    TEST_ASSERT_EQUAL_INT(155, db_steer_clamp(200));
}

void test_steer_pulse_us(void) {
    // 544 + angle*1856/180 integer math.
    // Calibrated center 140deg -> 1987us (matches calibration_data.txt)
    TEST_ASSERT_EQUAL_UINT32(1987, db_steer_pulse_us(STEER_CENTER));
    TEST_ASSERT_EQUAL_UINT32(1832, db_steer_pulse_us(STEER_MIN));   // 544 + 125*1856/180
    TEST_ASSERT_EQUAL_UINT32(2142, db_steer_pulse_us(STEER_MAX));   // 544 + 155*1856/180
}

void test_steer_angle_from_angular_z(void) {
    // angular_z (rad/s) -> steering angle (deg), MAX_TURN_RATE = 1.0 rad/s
    TEST_ASSERT_EQUAL_INT(140, db_steer_angle_from_angular_z(0.0f));    // straight
    TEST_ASSERT_EQUAL_INT(155, db_steer_angle_from_angular_z(1.0f));    // full left
    TEST_ASSERT_EQUAL_INT(125, db_steer_angle_from_angular_z(-1.0f));   // full right
    TEST_ASSERT_EQUAL_INT(155, db_steer_angle_from_angular_z(2.0f));    // clamped
    TEST_ASSERT_EQUAL_INT(125, db_steer_angle_from_angular_z(-2.0f));   // clamped
    TEST_ASSERT_EQUAL_INT(147, db_steer_angle_from_angular_z(0.5f));    // 140 + 7.5 -> 147
    TEST_ASSERT_EQUAL_INT(133, db_steer_angle_from_angular_z(-0.5f));   // 140 - 7.5 -> 133
    // Output never leaves the calibrated range
    for (float wz = -3.0f; wz <= 3.0f; wz += 0.05f) {
        int a = db_steer_angle_from_angular_z(wz);
        TEST_ASSERT_TRUE(a >= STEER_MIN && a <= STEER_MAX);
    }
}

// ── Motor: speed -> PWM mapping ───────────────────────────────────────────────

void test_motor_pwm_mapping(void) {
    bool fwd = false;

    // Deadband: |speed| < 0.05 -> PWM 0 (motor cannot overcome stiction below)
    TEST_ASSERT_EQUAL_INT(0, db_motor_pwm_from_speed(0.0f, &fwd));
    TEST_ASSERT_TRUE(fwd);                       // stopped reads as forward
    TEST_ASSERT_EQUAL_INT(0, db_motor_pwm_from_speed(0.04f, &fwd));
    TEST_ASSERT_EQUAL_INT(0, db_motor_pwm_from_speed(-0.04f, &fwd));

    // Above deadband: PWM = 35 + |speed| * (200-35), clamped to [35, 200]
    TEST_ASSERT_EQUAL_INT(43,  db_motor_pwm_from_speed(0.05f, &fwd));   // 35+8.25 -> 43
    TEST_ASSERT_EQUAL_INT(117, db_motor_pwm_from_speed(0.5f, &fwd));    // 35+82.5 -> 117
    TEST_ASSERT_EQUAL_INT(200, db_motor_pwm_from_speed(1.0f, &fwd));    // capped at MOTOR_MAX
    TEST_ASSERT_EQUAL_INT(200, db_motor_pwm_from_speed(5.0f, &fwd));    // clamped input

    // Direction
    TEST_ASSERT_TRUE(fwd);
    TEST_ASSERT_EQUAL_INT(117, db_motor_pwm_from_speed(-0.5f, &fwd));
    TEST_ASSERT_FALSE(fwd);
    TEST_ASSERT_EQUAL_INT(200, db_motor_pwm_from_speed(-1.0f, &fwd));
    TEST_ASSERT_FALSE(fwd);

    // Reverse PWM equals forward PWM at the same |speed| (H-bridge symmetric)
    TEST_ASSERT_EQUAL_INT(db_motor_pwm_from_speed(0.7f, NULL),
                          db_motor_pwm_from_speed(-0.7f, NULL));
}

// ── Motor: cmd_vel watchdog ───────────────────────────────────────────────────

void test_motor_watchdog(void) {
    // WATCHDOG_TIMEOUT_MS = 1000: expires only when strictly past the deadline
    TEST_ASSERT_FALSE(db_watchdog_expired(0,    0,    1000));   // fresh command
    TEST_ASSERT_FALSE(db_watchdog_expired(999,  0,    1000));   // just before
    TEST_ASSERT_FALSE(db_watchdog_expired(1000, 0,    1000));   // exactly at (strictly >)
    TEST_ASSERT_TRUE (db_watchdog_expired(1001, 0,    1000));   // expired
    TEST_ASSERT_TRUE (db_watchdog_expired(5000, 1000, 1000));   // long expired
}

// ── IMU: raw -> SI conversion ────────────────────────────────────────────────

void test_imu_raw_conversion(void) {
    // ±2g range: raw 16384 = 1g = 9.81 m/s²
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.81f,  db_accel_ms2(16384));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -9.81f, db_accel_ms2(-16384));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,   db_accel_ms2(0));

    // ±500°/s range: raw 655 = 655/65.5 = 10°/s = 0.17453 rad/s
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.17453f, db_gyro_rad_s(655));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.17453f, db_gyro_rad_s(-655));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, db_gyro_rad_s(0));
}

void test_imu_gyro_bias(void) {
    // Calibrated bias (2026-07-16, robot still on flat surface):
    // X=-0.0469, Y=-0.0007, Z=+0.0086 rad/s — applied as value -= bias
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    db_gyro_apply_bias(&gx, &gy, &gz);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0469f,  gx);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0007f,  gy);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, -0.0086f, gz);

    // A raw reading of exactly the bias reads zero after correction
    float gzx = -0.0469f, gzy = -0.0007f, gzz = 0.0086f;
    db_gyro_apply_bias(&gzx, &gzy, &gzz);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0f, gzx);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0f, gzy);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0f, gzz);
}

void test_imu_rotation(void) {
    // Sensor X points UP on the robot (90° rotation about Y):
    //   robot_x = -sensor_z, robot_y = sensor_y, robot_z = sensor_x
    // Gravity on a level robot: sensor reads +1g on X -> robot +1g on Z (up)
    float out[6];
    db_imu_rotate(9.81f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  out[0]);   // robot accel X
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  out[1]);   // robot accel Y
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.81f, out[2]);   // robot accel Z (up)

    // Sensor Y is robot Y (left)
    db_imu_rotate(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, out[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out[2]);

    // Sensor Z maps to -robot X (forward): +Z sensor -> robot reverses
    db_imu_rotate(0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -2.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  out[2]);

    // Gyro rotates with the same matrix: sensor +Z spin -> robot -X spin
    db_imu_rotate(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, out[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  out[4]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  out[5]);

    // Full pipeline sanity: raw sensor reading -> SI -> bias -> robot frame.
    // Sensor at rest reads gravity on +X: accel raw (16384, 0, 0),
    // gyro raw reads the calibrated bias (X=-0.0469 -> raw -3.08, but feed
    // the converted value directly): gyro reads exactly the bias -> 0 after
    // correction, so the robot frame sees pure gravity on Z.
    float ax = db_accel_ms2(16384), ay = db_accel_ms2(0), az = db_accel_ms2(0);
    float gx = DB_GYRO_BIAS_X, gy = DB_GYRO_BIAS_Y, gz = DB_GYRO_BIAS_Z;
    db_gyro_apply_bias(&gx, &gy, &gz);
    db_imu_rotate(ax, ay, az, gx, gy, gz, out);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f,  out[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f,  out[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  9.81f, out[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  out[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  out[4]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  out[5]);
}

// ── Encoder: direction + wraparound ──────────────────────────────────────────

void test_encoder_delta_signed(void) {
    // Forward (dir >= 0): raw deltas accumulate positive
    TEST_ASSERT_EQUAL_INT32(5,  db_encoder_delta_signed(15, 10, 1));
    TEST_ASSERT_EQUAL_INT32(5,  db_encoder_delta_signed(15, 10, 0));   // stopped counts as forward
    TEST_ASSERT_EQUAL_INT32(0,  db_encoder_delta_signed(10, 10, 1));   // no ticks

    // Reverse (dir < 0): deltas accumulate negative
    TEST_ASSERT_EQUAL_INT32(-5, db_encoder_delta_signed(15, 10, -1));
    TEST_ASSERT_EQUAL_INT32(-5, db_encoder_delta_signed(15, 10, -2));

    // uint32 wraparound: prev near UINT32_MAX, cur wrapped past 0.
    // 10 - 4294967290 = 16 (mod 2^32) -> +16 forward, -16 reverse.
    TEST_ASSERT_EQUAL_INT32(16,  db_encoder_delta_signed(10, 4294967290u, 1));
    TEST_ASSERT_EQUAL_INT32(-16, db_encoder_delta_signed(10, 4294967290u, -1));
}

// ── Unity runner (native: standard main) ─────────────────────────────────────

void setUp(void) {}
void tearDown(void) {}

int main(void) {
    UNITY_BEGIN();

    // Servo angle math
    RUN_TEST(test_servo_clamp_deg);
    RUN_TEST(test_servo_pulse_us);
    RUN_TEST(test_servo_sweep_bounds);

    // Steering angle math
    RUN_TEST(test_steer_clamp);
    RUN_TEST(test_steer_pulse_us);
    RUN_TEST(test_steer_angle_from_angular_z);

    // Motor PWM mapping + watchdog
    RUN_TEST(test_motor_pwm_mapping);
    RUN_TEST(test_motor_watchdog);

    // IMU conversion, bias, rotation
    RUN_TEST(test_imu_raw_conversion);
    RUN_TEST(test_imu_gyro_bias);
    RUN_TEST(test_imu_rotation);

    // Encoder direction
    RUN_TEST(test_encoder_delta_signed);

    return UNITY_END();
}
