/*
 * servo_config.h — Servo Tuning Parameters
 *
 * WHAT TO CHANGE:
 *   After you find good values via serial commands, update them here
 *   so they persist across reboots.
 *
 * SINUSOIDAL MOTION PARAMETERS:
 *   The servo sweeps using sin(). The formula is:
 *
 *     angle = CENTER + AMPLITUDE * sin(angle_rad)
 *
 *   Where angle_rad grows by SPEED_RAD_PER_SEC * dt each loop iteration.
 *
 *   CENTER     = midpoint of the sweep (degrees)
 *   AMPLITUDE  = how far from center it swings (degrees)
 *   SPEED      = how fast it oscillates (radians per second)
 *
 *   Example: CENTER=90, AMPLITUDE=45, SPEED=1.0
 *     → sweeps between 45° and 135° in a smooth sine wave
 *     → one full cycle takes 2π/1.0 = 6.28 seconds
 */

#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

// ── Physical Limits (hardware protection) ─────────────────────────────────────
// These are the ABSOLUTE endpoints. The servo will NEVER go outside these.
// Set these to where your mechanism physically stops.
#define SERVO_LIMIT_MIN     0       // degrees — absolute minimum
#define SERVO_LIMIT_MAX     180     // degrees — absolute maximum

// ── PWM Pulse Width ───────────────────────────────────────────────────────────
// Standard hobby servo: 500µs=0°, 2500µs=180°
// If servo jitters at endpoints or doesn't reach full range, adjust these.
#define SERVO_PULSE_MIN_US  500     // microseconds at 0°
#define SERVO_PULSE_MAX_US  2500    // microseconds at 180°

// ── Sinusoidal Motion Defaults ────────────────────────────────────────────────
// These are the startup values. Change at runtime via serial commands.
//
// 360° COVERAGE MATH:
//   3 sensors × 120° apart. Each sensor needs to sweep 120° to fill gaps.
//   Amplitude = 60° means servo goes 90±60 = [30° to 150°].
//   Each sensor sweeps a 120° arc. 3 × 120° = 360° full circle.
#define SERVO_CENTER_DEG    96.0f   // midpoint of sine sweep (calibrated — true forward)
#define SERVO_AMPLITUDE_DEG 60.0f   // half-swing: 60° gives 120° per sensor = 360° total
#define SERVO_SPEED_RAD_S   3.0f    // angular velocity (rad/s) — 2.1s per full cycle

// ── Loop Timing ───────────────────────────────────────────────────────────────
// How often the servo position updates (milliseconds).
// 10ms = 100Hz = smoother motion, smaller steps between positions
#define SERVO_UPDATE_MS     10

#endif // SERVO_CONFIG_H
