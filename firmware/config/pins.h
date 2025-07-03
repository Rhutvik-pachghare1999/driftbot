/*
 * pins.h — Central GPIO Pin Map
 *
 * RULE: Every physical wire connection is defined HERE and NOWHERE ELSE.
 *       When you rewire the robot, you change ONE file.
 */

#ifndef PINS_H
#define PINS_H

// ── Scanning Servo (sensor platform rotation) ─────────────────────────────────
#define PIN_SERVO       18    // GPIO 18 → scanning servo signal (sinusoidal sweep)

// ── Steering Servo (MG90S, front wheel direction) ─────────────────────────────
#define PIN_STEERING    10    // GPIO 10 → steering servo signal

// ── DC Motor Driver (BTS7960 43A H-Bridge) ────────────────────────────────────
#define PIN_MOTOR_RPWM  11    // GPIO 11 → right PWM (forward speed)
#define PIN_MOTOR_LPWM  12    // GPIO 12 → left PWM (reverse speed)
#define PIN_MOTOR_REN   15    // GPIO 15 → right enable (HIGH = forward enabled)
#define PIN_MOTOR_LEN   16    // GPIO 16 → left enable (HIGH = reverse enabled)

// ── Onboard LED ───────────────────────────────────────────────────────────────
#define PIN_LED         48    // ESP32-S3-DevKitC-1 addressable RGB LED

// ── ToF Sensors (VL53L1X) — I2C Bus 0 ────────────────────────────────────────
// The physical I2C wires on this robot are connected to GPIO 8 (SDA) and
// GPIO 9 (SCL). These are general-purpose GPIOs on the ESP32-S3-DevKitC-1-N8
// (the on-module flash/PSRAM uses different pins and does NOT conflict).
#define PIN_TOF_SDA     8     // GPIO 8 → I2C data line (shared by all 3 sensors)
#define PIN_TOF_SCL     9     // GPIO 9 → I2C clock line (shared by all 3 sensors)

// ── IMU (MPU6050) — I2C Bus 1 ─────────────────────────────────────────────────
// The IMU was disappearing when sharing the bus with three VL53L1X sensors.
// It now lives on its own second I2C bus (Wire1) on two free GPIOs.
#define PIN_IMU_SDA     7     // GPIO 7  → IMU I2C data line (J1 pin 7)
#define PIN_IMU_SCL     17    // GPIO 17 → IMU I2C clock line (J1 pin 10)

// ── ToF Sensors — XSHUT pins (one per sensor) ────────────────────────────────
// XSHUT = shutdown. LOW = sensor off. HIGH = sensor on.
// Used at boot to assign unique I2C addresses to each sensor.
#define PIN_XSHUT_0     4     // GPIO 4  → Left sensor XSHUT
#define PIN_XSHUT_1     5     // GPIO 5  → Center sensor XSHUT
#define PIN_XSHUT_2     6     // GPIO 6  → Right sensor XSHUT

// ── Hall Encoders (rear wheels, 10 magnets per wheel) ─────────────────────────
#define PIN_ENC_LEFT    13    // GPIO 13 → Left rear wheel hall sensor
#define PIN_ENC_RIGHT   14    // GPIO 14 → Right rear wheel hall sensor

#endif // PINS_H
