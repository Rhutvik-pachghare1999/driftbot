/*
 * motor_config.h — Drive & Steering Configuration
 *
 * BTS7960 DC Motor Driver:
 *   - RPWM: forward speed (0-255 PWM duty)
 *   - LPWM: reverse speed (0-255 PWM duty)
 *   - REN/LEN: enable pins (HIGH to enable direction)
 *   - Only one direction active at a time
 *
 * MG90S Steering Servo:
 *   - Metal geared, 180° range
 *   - Center = straight, min/max = full left/right turn
 *
 * PWM VALUES:
 *   MOTOR_MAX = 180      Maximum PWM duty (not 255 — protects motor/gears)
 *   MOTOR_MIN_USEFUL = 35  Below this the motor stalls (not enough torque)
 *
 * STEERING ANGLES (degrees written to servo):
 *   STEER_CENTER = 140   Wheels point straight (not 90° — servo trim offset)
 *   STEER_MIN = 125      Full right turn
 *   STEER_MAX = 155      Full left turn
 *   Range = ±15° from center
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// ── DC Motor (BTS7960) ────────────────────────────────────────────────────────
#define MOTOR_MAX         200    // Max PWM value (calibrated)
#define MOTOR_MIN_USEFUL  35     // Minimum PWM where motor actually turns

// ── Steering Servo (MG90S) ────────────────────────────────────────────────────
#define STEER_CENTER      140    // Straight ahead (calibrated)
#define STEER_MIN         125    // Full right turn (calibrated)
#define STEER_MAX         155    // Full left turn (calibrated)

// ── Speed Limits ──────────────────────────────────────────────────────────────
#define MAX_SPEED_MS      0.5f   // Maximum linear speed (meters/second)
#define MAX_TURN_RATE     1.0f   // Maximum angular velocity (radians/second)

#endif // MOTOR_CONFIG_H
