/*
 * imu_driver.cpp — MPU6050 IMU Implementation
 *
 * Uses direct I2C register access (no external library needed).
 * The MPU6050 is simple: write config registers, then read 14 bytes of data.
 *
 * Register map (what matters):
 *   0x6B = PWR_MGMT_1   → write 0x00 to wake up (chip starts in sleep mode)
 *   0x1B = GYRO_CONFIG  → write 0x08 for ±500°/s range
 *   0x1C = ACCEL_CONFIG → write 0x00 for ±2g range
 *   0x3B = ACCEL_XOUT_H → start of 14-byte burst read (accel + temp + gyro)
 *
 * Data format: each axis is 16-bit signed (high byte first).
 *   Accel: raw / 16384.0 = g → × 9.81 = m/s²  (at ±2g range)
 *   Gyro:  raw / 65.5 = °/s → × π/180 = rad/s (at ±500°/s range)
 */

#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "imu_driver.h"
#include "driftbot_math.h"

// ── MPU6050 I2C Address ───────────────────────────────────────────────────────
// Default 0x68 (AD0 pin LOW). If AD0 is HIGH, address becomes 0x69.
#define MPU6050_ADDR_DEFAULT  0x68
#define MPU6050_ADDR_ALT      0x69

// Active address (determined at init)
static uint8_t mpu_addr = MPU6050_ADDR_DEFAULT;

// ── Registers ─────────────────────────────────────────────────────────────────
#define REG_PWR_MGMT_1      0x6B
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_WHO_AM_I        0x75

// ── Conversion factors ────────────────────────────────────────────────────────
// Defined in driftbot_math.h (DB_ACCEL_SCALE / DB_GYRO_SCALE) — shared with
// the host unit tests (pio test -e native).
#define ACCEL_SCALE  DB_ACCEL_SCALE   // ±2g range: raw/16384 = g -> m/s²
#define GYRO_SCALE   DB_GYRO_SCALE    // ±500°/s range: raw/65.5 = °/s -> rad/s

// ── State ─────────────────────────────────────────────────────────────────────
static ImuData current_data = {0};
static bool initialized = false;
static unsigned long last_read = 0;

// ── Update rate ───────────────────────────────────────────────────────────────
#define IMU_UPDATE_MS   20   // 50Hz — avoids flooding shared I2C bus

// Temporary diagnostic counter for I2C errors
#define DIAG_MAX 5
static int diag_count = 0;

// ── Helper: write one byte to a register ──────────────────────────────────────
static bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    Wire1.beginTransmission(mpu_addr);
    Wire1.write(buf, 2);
    uint8_t err = Wire1.endTransmission(true);
    if (err != 0 && diag_count < DIAG_MAX) {
        Serial.printf("[IMU-DEBUG] write_reg(0x%02X)=0x%02X err=%u\n", reg, value, err);
        diag_count++;
    }
    return err == 0;
}

// ── Helper: read one byte from a register ─────────────────────────────────────
static uint8_t read_reg(uint8_t reg) {
    Wire1.beginTransmission(mpu_addr);
    Wire1.write(reg);
    Wire1.endTransmission(true);            // STOP, not repeated-start
    Wire1.requestFrom(mpu_addr, (uint8_t)1);
    if (Wire1.available()) return Wire1.read();
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: imu_init()
// ═══════════════════════════════════════════════════════════════════════════════
bool imu_init() {
    // The IMU lives on a dedicated second I2C bus (Wire1) on GPIO 7/17 with
    // external pull-ups. The GY-521 module's green LED shows that its 3.3 V
    // regulator ramps up slowly: it flickers dim, fades off, then stabilizes.
    // We therefore wait up to 4 s for the chip to answer reliably before
    // configuring it.
    Wire1.setPins(PIN_IMU_SDA, PIN_IMU_SCL);
    Wire1.begin();
    Wire1.setClock(50000);    // 50 kHz — more tolerant of marginal wiring
    Wire1.setTimeOut(200);    // generous per-byte timeout
    delay(100);

    // Wait for the module to finish its power-on ramp.
    bool found = false;
    for (int attempt = 0; attempt < 40; ++attempt) {
        Wire1.beginTransmission(MPU6050_ADDR_DEFAULT);
        if (Wire1.endTransmission(true) != 0) {
            delay(100);
            continue;
        }
        Wire1.beginTransmission(MPU6050_ADDR_DEFAULT);
        Wire1.write(REG_WHO_AM_I);
        if (Wire1.endTransmission(true) != 0) {
            delay(100);
            continue;
        }
        if (Wire1.requestFrom((uint16_t)MPU6050_ADDR_DEFAULT, (uint8_t)1, (uint8_t)1) == 1) {
            uint8_t who = Wire1.read();
            if (who == 0x68 || who == 0x98) {
                mpu_addr = MPU6050_ADDR_DEFAULT;
                found = true;
                break;
            }
        }
        delay(100);
    }

    // Also probe the alternate address in case AD0 is wired high.
    if (!found) {
        for (int attempt = 0; attempt < 10; ++attempt) {
            Wire1.beginTransmission(MPU6050_ADDR_ALT);
            if (Wire1.endTransmission(true) == 0) {
                mpu_addr = MPU6050_ADDR_ALT;
                found = true;
                break;
            }
            delay(100);
        }
    }

    if (!found) {
        Serial.println("[IMU] MPU6050 not found at 0x68 or 0x69 on Wire1 bus.");
        initialized = false;
        return false;
    }

    Serial.printf("[IMU] MPU6050 found at 0x%02X\n", mpu_addr);

    // The chip powers on with SLEEP=1. Some modules need a reset and several
    // wake attempts before the write latches, so retry and read back the
    // register to verify.
    write_reg(REG_PWR_MGMT_1, 0x80);  // H_RESET
    delay(300);

    bool awake = false;
    const uint8_t wake_vals[] = { 0x00, 0x01, 0x02, 0x03, 0x00, 0x01 };
    for (uint8_t wake : wake_vals) {
        write_reg(REG_PWR_MGMT_1, wake);
        delay(120);
        uint8_t pwr1 = read_reg(REG_PWR_MGMT_1);
        if ((pwr1 & 0x40) == 0) {
            awake = true;
            break;
        }
    }

    if (!awake) {
        Serial.println("[IMU] ERROR: sleep bit did not clear — IMU unusable");
        initialized = false;
        return false;
    }

    // Configure ranges.
    write_reg(REG_GYRO_CONFIG, 0x08);   // ±500°/s
    write_reg(REG_ACCEL_CONFIG, 0x00);  // ±2g
    delay(10);

    initialized = true;
    last_read = millis();
    Serial.println("[IMU] MPU6050 initialized (±2g, ±500°/s, 50Hz)");
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: imu_update() — read all 6 axes in one burst
// ═══════════════════════════════════════════════════════════════════════════════
void imu_update() {
    if (!initialized) return;

    unsigned long now = millis();
    if (now - last_read < IMU_UPDATE_MS) return;
    last_read = now;

    // Request 14 bytes starting at ACCEL_XOUT_H. MPU6050 supports a
    // repeated-start burst read (the common Arduino pattern), so keep the bus
    // active between the register-pointer write and the data read.
    static int debug_reads = 0;

    Wire1.beginTransmission(mpu_addr);
    Wire1.write(REG_ACCEL_XOUT_H);
    uint8_t err = Wire1.endTransmission(false);  // repeated-start
    if (err != 0) {
        if (debug_reads < 5) {
            Serial.printf("[IMU-DEBUG] endTransmission err=%u\n", err);
            debug_reads++;
        }
        return;  // Bus busy or NACK — skip
    }

    uint8_t count = Wire1.requestFrom(mpu_addr, (uint8_t)14, (uint8_t)true);
    if (count < 14) {
        if (debug_reads < 5) {
            Serial.printf("[IMU-DEBUG] requestFrom count=%u\n", count);
            debug_reads++;
        }
        // Flush whatever is there
        while (Wire1.available()) Wire1.read();
        return;
    }

    // Read raw 16-bit values (high byte first)
    int16_t ax = (Wire1.read() << 8) | Wire1.read();
    int16_t ay = (Wire1.read() << 8) | Wire1.read();
    int16_t az = (Wire1.read() << 8) | Wire1.read();
    int16_t temp_raw = (Wire1.read() << 8) | Wire1.read();  // Skip temperature
    (void)temp_raw;
    int16_t gx = (Wire1.read() << 8) | Wire1.read();
    int16_t gy = (Wire1.read() << 8) | Wire1.read();
    int16_t gz = (Wire1.read() << 8) | Wire1.read();

    // Convert to SI units (shared math in driftbot_math.h)
    float new_ax = db_accel_ms2(ax);  // m/s²
    float new_ay = db_accel_ms2(ay);
    float new_az = db_accel_ms2(az);
    float new_gx = db_gyro_rad_s(gx);  // rad/s
    float new_gy = db_gyro_rad_s(gy);
    float new_gz = db_gyro_rad_s(gz);

    // Sanity check: only discard if ALL values are exactly zero (I2C failure)
    if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
        if (debug_reads < 5) {
            Serial.println("[IMU-DEBUG] all raw values zero");
            debug_reads++;
        }
        return;
    }

    // ── Apply calibrated gyro bias correction (from calibration tool) ────────
    // These values measured 2026-07-16 with robot still on flat surface
    // (DB_GYRO_BIAS_* in driftbot_math.h: X=-0.0469, Y=-0.0007, Z=+0.0086)
    db_gyro_apply_bias(&new_gx, &new_gy, &new_gz);

    // ── Apply rotation matrix: sensor frame → robot frame (REP-103) ─────────
    // IMU is mounted with sensor X pointing UP.
    // Robot convention (ROS REP-103): X=forward, Y=left, Z=up
    //   robot_x = -sensor_z, robot_y = sensor_y, robot_z = sensor_x
    // (db_imu_rotate in driftbot_math.h; same rotation for accel + gyro)
    float robot[6];
    db_imu_rotate(new_ax, new_ay, new_az, new_gx, new_gy, new_gz, robot);
    current_data.accel_x = robot[0];
    current_data.accel_y = robot[1];
    current_data.accel_z = robot[2];
    current_data.gyro_x  = robot[3];
    current_data.gyro_y  = robot[4];
    current_data.gyro_z  = robot[5];
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: imu_get_data()
// ═══════════════════════════════════════════════════════════════════════════════
ImuData imu_get_data() {
    return current_data;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: imu_handle_serial()
// ═══════════════════════════════════════════════════════════════════════════════
bool imu_handle_serial(const char* cmd) {
    if (strcmp(cmd, "imu status") == 0) {
        Serial.println("┌─── IMU Status ─────────────────────────────────┐");
        Serial.printf( "│  Chip:    MPU6050 @ 0x%02X\n", mpu_addr);
        Serial.printf( "│  Status:  %s\n", initialized ? "OK" : "FAILED");
        Serial.printf( "│  Accel:   [%+.2f, %+.2f, %+.2f] m/s²\n",
                       current_data.accel_x, current_data.accel_y, current_data.accel_z);
        Serial.printf( "│  Gyro:    [%+.3f, %+.3f, %+.3f] rad/s\n",
                       current_data.gyro_x, current_data.gyro_y, current_data.gyro_z);
        Serial.printf( "│  Rate:    %d Hz\n", 1000 / IMU_UPDATE_MS);
        Serial.println("└────────────────────────────────────────────────┘");
        return true;
    }

    if (strcmp(cmd, "imu read") == 0) {
        Serial.printf("[IMU] accel=[%+.2f, %+.2f, %+.2f] gyro=[%+.3f, %+.3f, %+.3f]\n",
                      current_data.accel_x, current_data.accel_y, current_data.accel_z,
                      current_data.gyro_x, current_data.gyro_y, current_data.gyro_z);
        return true;
    }

    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: imu_print_help()
// ═══════════════════════════════════════════════════════════════════════════════
void imu_print_help() {
    Serial.println("  imu status    Show IMU state and readings");
    Serial.println("  imu read      Print current accel + gyro");
}
