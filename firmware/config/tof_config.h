/*
 * tof_config.h — ToF Sensor Array Configuration
 *
 * Three VL53L1X sensors (TOF-400C modules) mounted at fixed angles.
 * Each produces a distance reading. Combined with the servo angle,
 * this creates a 2.5D point set published as JSON over serial.
 *
 * JSON OUTPUT FORMAT (published every TOF_PUBLISH_MS):
 * {
 *   "servo_deg": 90.0,
 *   "tof": [
 *     {"id":0, "angle":-30.0, "dist_mm":1234, "status":0},
 *     {"id":1, "angle":0.0,   "dist_mm":2100, "status":0},
 *     {"id":2, "angle":30.0,  "dist_mm":987,  "status":0}
 *   ]
 * }
 */

#ifndef TOF_CONFIG_H
#define TOF_CONFIG_H

// ── Number of sensors ─────────────────────────────────────────────────────────
#define TOF_SENSOR_COUNT    3

// ── I2C Addresses (assigned at boot via XSHUT sequencing) ─────────────────────
// Default is 0x29 for all. We reassign to unique addresses.
#define TOF_ADDR_0          0x30    // Left sensor
#define TOF_ADDR_1          0x31    // Center sensor
#define TOF_ADDR_2          0x32    // Right sensor

// ── Mounting Angles (degrees offset from reference) ───────────────────────────
// Sensor 2 (GPIO 6) is the REFERENCE: aligned with servo 90° = robot forward.
// Sensors are mounted on a triangular plate, 120° apart.
// true_angle = (servo_angle - 90) + mount_offset
//
// With amplitude=60° sweep: each sensor covers 120° arc.
// 3 × 120° = 360° full coverage.
#define TOF_ANGLE_0         180.0f  // Sensor 0 points backward (calibrated)
#define TOF_ANGLE_1         115.0f  // Sensor 1 points left-rear (calibrated)
#define TOF_ANGLE_2           0.0f  // Sensor 2 = forward (reference)

// ── Distance Mode ─────────────────────────────────────────────────────────────
// 1 = Short  (up to 1.3m, best ambient light immunity)
// 2 = Long   (up to 4.0m, less ambient immunity)
#define TOF_DISTANCE_MODE   2       // Long mode for 4m range

// ── Timing Budget (microseconds) ──────────────────────────────────────────────
// Higher = more accurate but slower.
//   20000µs  = ±25mm accuracy, 50Hz max  ← FASTEST (we use this for fast scanning)
//   50000µs  = ±15mm accuracy, 20Hz max
//  100000µs  = ±10mm accuracy, 10Hz max
#define TOF_TIMING_BUDGET_US  20000  // 20ms — fastest possible, ±25mm noise

// ── Inter-measurement period (milliseconds) ───────────────────────────────────
// Must be >= timing_budget. This is how often the sensor starts a new range.
#define TOF_INTER_MEAS_MS   25      // 25ms — slightly more than 20ms budget

// ── JSON Publish Rate ─────────────────────────────────────────────────────────
// How often to print the JSON object to serial (milliseconds).
#define TOF_PUBLISH_MS      100     // 100ms = 10Hz JSON output

#endif // TOF_CONFIG_H
