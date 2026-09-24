/*
 * driftbot_math.h — Pure math helpers shared by drivers and unit tests
 *
 * HARDWARE-FREE: this header includes no Arduino/ESP-IDF headers so the
 * exact math used on the robot can be unit-tested on the host
 * (pio test -e native) and on-target (pio test -e esp32s3).
 *
 * Every function here is the byte-identical expression lifted from the
 * driver that owns it:
 *   - servo_driver.cpp  → scan servo clamp / pulse / sine sweep
 *   - motor_driver.cpp  → steering clamp / pulse, angular_z → steer angle,
 *                          speed → PWM mapping, cmd_vel watchdog
 *   - imu_driver.cpp    → raw → SI conversion, gyro bias, sensor→robot rotation
 *   - encoder_driver.cpp→ signed delta from raw counters + direction
 */

#ifndef DRIFTBOT_MATH_H
#define DRIFTBOT_MATH_H

#include <stdint.h>
#include <math.h>

#include "servo_config.h"   // SERVO_LIMIT_MIN/MAX, SERVO_PULSE_MIN_US/MAX_US
#include "motor_config.h"   // MOTOR_MIN_USEFUL/MAX, STEER_CENTER/MIN/MAX, MAX_TURN_RATE

// ── Generic clamp ─────────────────────────────────────────────────────────────

static inline float db_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ── Scanning servo (servo_driver.cpp) ─────────────────────────────────────────

// Clamp to the mechanical limits (SERVO_LIMIT_MIN..SERVO_LIMIT_MAX).
static inline float db_servo_clamp_deg(float deg) {
    return db_clampf(deg, (float)SERVO_LIMIT_MIN, (float)SERVO_LIMIT_MAX);
}

// Hobby-servo pulse width: SERVO_PULSE_MIN_US at 0deg, SERVO_PULSE_MAX_US at 180deg.
static inline float db_servo_pulse_us(int deg) {
    return (float)SERVO_PULSE_MIN_US +
           (float)deg * (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180.0f;
}

// Sinusoidal sweep position: center + amplitude * sin(phase).
static inline float db_servo_sweep_deg(float center, float amplitude, float phase) {
    return center + amplitude * sinf(phase);
}

// ── Steering (motor_driver.cpp) ────────────────────────────────────────────────

// Clamp to the calibrated steering range (STEER_MIN..STEER_MAX).
static inline int db_steer_clamp(int angle_deg) {
    if (angle_deg < STEER_MIN) return STEER_MIN;
    if (angle_deg > STEER_MAX) return STEER_MAX;
    return angle_deg;
}

// Steering pulse width (same map the servo hardware uses, 544..2400us).
// 140deg center -> 1987us (matches the calibrated center pulse).
static inline uint32_t db_steer_pulse_us(int angle_deg) {
    return 544 + (uint32_t)angle_deg * 1856 / 180;
}

// angular velocity command (rad/s) -> steering servo angle (deg).
// +1.0 rad/s -> full left (STEER_MAX), -1.0 rad/s -> full right (STEER_MIN).
static inline int db_steer_angle_from_angular_z(float angular_z) {
    float steer_norm = angular_z / MAX_TURN_RATE;
    if (steer_norm > 1.0f) steer_norm = 1.0f;
    if (steer_norm < -1.0f) steer_norm = -1.0f;

    if (steer_norm >= 0.0f) {
        return STEER_CENTER + (int)(steer_norm * (STEER_MAX - STEER_CENTER));
    }
    return STEER_CENTER + (int)(steer_norm * (STEER_CENTER - STEER_MIN));
}

// ── DC motor (motor_driver.cpp) ───────────────────────────────────────────────

// Normalized speed (-1..1) -> BTS7960 PWM value (0..255 scale, capped at
// MOTOR_MAX). |speed| < 0.05 is the deadband -> PWM 0 (returns 0 and *forward=true,
// matching apply_motor(0, true)). MOTOR_MIN_USEFUL is the stall floor.
static inline int db_motor_pwm_from_speed(float speed_normalized, bool* forward) {
    if (speed_normalized > 1.0f)  speed_normalized = 1.0f;
    if (speed_normalized < -1.0f) speed_normalized = -1.0f;

    bool fwd = (speed_normalized >= 0.0f);
    float abs_speed = fabsf(speed_normalized);
    if (abs_speed < 0.05f) {
        if (forward) *forward = true;   // stopped is "forward" (both H-bridge low)
        return 0;
    }

    int pwm = (int)(MOTOR_MIN_USEFUL + abs_speed * (MOTOR_MAX - MOTOR_MIN_USEFUL));
    if (forward) *forward = fwd;
    return pwm;
}

// cmd_vel watchdog: true when now is more than timeout_ms past last_cmd.
// Unsigned subtraction is wraparound-safe (same as the driver's millis() math).
static inline bool db_watchdog_expired(unsigned long now, unsigned long last_cmd,
                                       unsigned long timeout_ms) {
    return (now - last_cmd) > timeout_ms;
}

// ── IMU (imu_driver.cpp) ───────────────────────────────────────────────────────

// MPU6050 raw -> SI. ±2g range: raw/16384 = g -> m/s^2.
// ±500deg/s range: raw/65.5 = deg/s -> rad/s.
#define DB_ACCEL_SCALE  (9.81f / 16384.0f)
#define DB_GYRO_SCALE   (1.0f / 65.5f * 0.017453f)

static inline float db_accel_ms2(int16_t raw) { return (float)raw * DB_ACCEL_SCALE; }
static inline float db_gyro_rad_s(int16_t raw) { return (float)raw * DB_GYRO_SCALE; }

// Calibrated gyro bias (rad/s), measured 2026-07-16 with the robot still
// on a flat surface. Applied as value -= bias.
#define DB_GYRO_BIAS_X  (-0.0469f)
#define DB_GYRO_BIAS_Y  (-0.0007f)
#define DB_GYRO_BIAS_Z  (+0.0086f)

static inline void db_gyro_apply_bias(float* gx, float* gy, float* gz) {
    *gx -= DB_GYRO_BIAS_X;
    *gy -= DB_GYRO_BIAS_Y;
    *gz -= DB_GYRO_BIAS_Z;
}

// Sensor frame -> robot frame (REP-103: X=forward, Y=left, Z=up).
// The IMU is mounted with sensor X pointing UP, a 90deg rotation about Y:
//   robot_x = -sensor_z
//   robot_y =  sensor_y
//   robot_z =  sensor_x
// Same rotation applies to accel and gyro. out6 = [ax ay az gx gy gz].
static inline void db_imu_rotate(float ax, float ay, float az,
                                 float gx, float gy, float gz,
                                 float* out6) {
    out6[0] = -az;    // robot accel X
    out6[1] =  ay;    // robot accel Y
    out6[2] =  ax;    // robot accel Z
    out6[3] = -gz;    // robot gyro X
    out6[4] =  gy;    // robot gyro Y
    out6[5] =  gx;    // robot gyro Z
}

// ── Encoder (encoder_driver.cpp) ────────────────────────────────────────────────

// Signed tick delta from two raw ISR counters and the motor direction.
// ISR counters only increment; the sign is applied at read time using the
// motor direction. uint32 subtraction is wraparound-safe.
static inline int32_t db_encoder_delta_signed(uint32_t cur, uint32_t prev, int8_t dir) {
    uint32_t delta = cur - prev;
    return (dir >= 0) ? (int32_t)delta : -(int32_t)delta;
}

#endif // DRIFTBOT_MATH_H
