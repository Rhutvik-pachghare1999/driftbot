/*
 * DriftBot Calibration Tool
 *
 * A standalone firmware for calibrating each sensor individually.
 * Flash this instead of main firmware, calibrate, then flash main firmware back.
 *
 * DOES NOT modify main firmware files.
 *
 * MENU:
 *   1 = IMU calibration (gyro bias + accel offset)
 *   2 = ToF calibration (offset per sensor)
 *   3 = Encoder calibration (ticks per revolution)
 *   4 = Steering servo calibration (center + endpoints)
 *   5 = Scanning servo calibration (angle verification)
 *   6 = Motor speed calibration (PWM vs actual speed)
 *   h = Show help menu
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <VL53L1X.h>
#include "driver/mcpwm.h"
#include "pins.h"
#include "motor_config.h"
#include "servo_config.h"
#include "tof_config.h"

// ── Globals ───────────────────────────────────────────────────────────────────
static char cmd[64];
static int cmd_len = 0;
static int current_mode = 0;

// ToF sensors
static VL53L1X tof_sensors[3];
static uint8_t tof_xshut[] = {PIN_XSHUT_0, PIN_XSHUT_1, PIN_XSHUT_2};
static uint8_t tof_addr[] = {TOF_ADDR_0, TOF_ADDR_1, TOF_ADDR_2};

// Encoder: Timer-based polling with state machine
// Polls at 500Hz (every 2ms via hardware timer).
// State machine: IDLE → TRIGGERED → IDLE
//   IDLE: pin is HIGH, waiting for LOW
//   TRIGGERED: pin went LOW, count tick, wait until pin goes HIGH again
// This naturally handles oscillation: once we enter TRIGGERED, we stay there
// until the magnet fully passes and pin returns to HIGH. No double-counting.
static volatile uint32_t enc_left = 0;
static volatile uint32_t enc_right = 0;

// States: 0=waiting for LOW (idle), 1=saw LOW, waiting for HIGH (triggered)
static volatile uint8_t state_l = 0;
static volatile uint8_t state_r = 0;

// Hardware timer for polling
static hw_timer_t* enc_timer = NULL;

static void IRAM_ATTR enc_timer_isr() {
    uint8_t l = digitalRead(PIN_ENC_LEFT);
    uint8_t r = digitalRead(PIN_ENC_RIGHT);

    // Left encoder state machine
    if (state_l == 0 && l == 0) {
        // Transition: was waiting, now see LOW → count tick
        enc_left++;
        state_l = 1;  // Now wait for HIGH before counting again
    } else if (state_l == 1 && l == 1) {
        // Magnet passed, pin back HIGH → ready for next
        state_l = 0;
    }

    // Right encoder state machine
    if (state_r == 0 && r == 0) {
        enc_right++;
        state_r = 1;
    } else if (state_r == 1 && r == 1) {
        state_r = 0;
    }
}

// Dummy ISRs (attachInterrupt still called but they do nothing)
static void IRAM_ATTR isr_l() {}
static void IRAM_ATTR isr_r() {}

// IMU
#define MPU_ADDR 0x68
static float gyro_bias[3] = {0, 0, 0};
static float accel_bias[3] = {0, 0, 0};


// ═══════════════════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

static void imu_read_raw(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);
    if (Wire.available() < 14) { *ax=*ay=*az=*gx=*gy=*gz=0; return; }
    int16_t raw_ax = (Wire.read()<<8)|Wire.read();
    int16_t raw_ay = (Wire.read()<<8)|Wire.read();
    int16_t raw_az = (Wire.read()<<8)|Wire.read();
    Wire.read(); Wire.read(); // skip temp
    int16_t raw_gx = (Wire.read()<<8)|Wire.read();
    int16_t raw_gy = (Wire.read()<<8)|Wire.read();
    int16_t raw_gz = (Wire.read()<<8)|Wire.read();
    *ax = raw_ax * 9.81f / 16384.0f;
    *ay = raw_ay * 9.81f / 16384.0f;
    *az = raw_az * 9.81f / 16384.0f;
    *gx = raw_gx / 65.5f * 0.017453f;
    *gy = raw_gy / 65.5f * 0.017453f;
    *gz = raw_gz / 65.5f * 0.017453f;
}

static void steer_servo_write(int angle) {
    uint32_t pulse_us = 544 + (uint32_t)angle * 1856 / 180;
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_us);
}

static void scan_servo_write(int angle) {
    float pulse_us = SERVO_PULSE_MIN_US + (float)angle * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180.0f;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_us / 20000.0f * 100.0f);
}

static bool motor_running = false;

static bool motor_is_running() { return motor_running; }

static void motor_drive(int pwm) {
    // positive = forward, negative = reverse, 0 = stop
    motor_running = (pwm != 0);
    float duty = abs(pwm) / 255.0f * 100.0f;
    if (pwm == 0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    } else if (pwm > 0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, duty);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
// MODE 1: IMU CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════
static void imu_calibrate() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║  IMU CALIBRATION                         ║");
    Serial.println("║  Keep robot PERFECTLY STILL on flat      ║");
    Serial.println("║  surface for 5 seconds...                ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
    Serial.println("Collecting 500 samples...");

    float sum_ax=0, sum_ay=0, sum_az=0;
    float sum_gx=0, sum_gy=0, sum_gz=0;
    float ax, ay, az, gx, gy, gz;

    for (int i = 0; i < 500; i++) {
        imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz);
        sum_ax += ax; sum_ay += ay; sum_az += az;
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        delay(10);
        if (i % 50 == 0) Serial.printf("  %d/500...\n", i);
    }

    gyro_bias[0] = sum_gx / 500.0f;
    gyro_bias[1] = sum_gy / 500.0f;
    gyro_bias[2] = sum_gz / 500.0f;
    accel_bias[0] = sum_ax / 500.0f - 0.0f;   // expected X = 0
    accel_bias[1] = sum_ay / 500.0f - 0.0f;   // expected Y = 0
    accel_bias[2] = sum_az / 500.0f - 9.81f;  // expected Z = 9.81

    Serial.println("\n═══ RESULTS ═══");
    Serial.printf("  Gyro bias:  X=%+.4f  Y=%+.4f  Z=%+.4f rad/s\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    Serial.printf("  Accel bias: X=%+.4f  Y=%+.4f  Z=%+.4f m/s²\n", accel_bias[0], accel_bias[1], accel_bias[2]);
    Serial.println("\n  ► Copy these values to imu_driver.cpp calibration constants.");
    Serial.println("  ► Gyro Z bias is most critical for SLAM heading accuracy.");
    Serial.printf("  ► Without correction: %.1f° drift per minute\n", gyro_bias[2] * 60.0f * 180.0f / 3.14159f);

    Serial.println("\n  Type 'r' to read live corrected values, 'q' to return to menu.");
    current_mode = 10; // sub-mode: live IMU display
}

static void imu_live() {
    float ax, ay, az, gx, gy, gz;
    imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.printf("  RAW  accel=[%+.2f %+.2f %+.2f] gyro=[%+.4f %+.4f %+.4f]\n", ax, ay, az, gx, gy, gz);
    Serial.printf("  CORR accel=[%+.2f %+.2f %+.2f] gyro=[%+.4f %+.4f %+.4f]\n",
        ax - accel_bias[0], ay - accel_bias[1], az - accel_bias[2],
        gx - gyro_bias[0], gy - gyro_bias[1], gz - gyro_bias[2]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 2: TOF CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════
static void tof_calibrate() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║  TOF OFFSET CALIBRATION                  ║");
    Serial.println("║  Place WHITE paper at exactly 140mm      ║");
    Serial.println("║  from ALL 3 sensor faces.                ║");
    Serial.println("║  Press Enter when ready...               ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
    current_mode = 20; // wait for enter
}

static void tof_run_calibration() {
    Serial.println("Reading 50 samples per sensor...");
    for (int s = 0; s < 3; s++) {
        long sum = 0;
        int count = 0;
        for (int i = 0; i < 50; i++) {
            tof_sensors[s].read();
            if (tof_sensors[s].ranging_data.range_status == 0) {
                sum += tof_sensors[s].ranging_data.range_mm;
                count++;
            }
            delay(30);
        }
        if (count > 0) {
            float avg = (float)sum / count;
            float offset = avg - 140.0f;
            Serial.printf("  Sensor %d: avg=%.1fmm, offset=%+.1fmm (subtract this from readings)\n", s, avg, offset);
        } else {
            Serial.printf("  Sensor %d: NO VALID READINGS — check sensor\n", s);
        }
    }
    Serial.println("\n  ► Apply offset in tof_driver.cpp: distance_mm -= offset;");
    Serial.println("  Type 'r' for live readings, 'q' to return.");
    current_mode = 21;
}

static void tof_live() {
    for (int s = 0; s < 3; s++) {
        tof_sensors[s].read();
        Serial.printf("  ToF%d: %4dmm (status=%d)", s,
            tof_sensors[s].ranging_data.range_mm,
            tof_sensors[s].ranging_data.range_status);
    }
    Serial.println();
}


// ═══════════════════════════════════════════════════════════════════════════════
// MODE 3: ENCODER CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════
static void encoder_calibrate() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║  ENCODER CALIBRATION                     ║");
    Serial.println("║                                          ║");
    Serial.println("║  Commands:                               ║");
    Serial.println("║    c     = clear counters                ║");
    Serial.println("║    f PWM = motor forward (e.g. 'f 60')   ║");
    Serial.println("║    r PWM = motor reverse                 ║");
    Serial.println("║    s     = stop motor                    ║");
    Serial.println("║    t     = show tick count               ║");
    Serial.println("║    q     = return to menu                ║");
    Serial.println("║                                          ║");
    Serial.println("║  PROCEDURE:                              ║");
    Serial.println("║  1. Type 'c' to clear                    ║");
    Serial.println("║  2. Rotate ONE wheel exactly 1 turn      ║");
    Serial.println("║  3. Type 't' — should show 10 ticks      ║");
    Serial.println("║  4. Or: type 'f 60', drive 1m measured   ║");
    Serial.println("║     then 's', check ticks. Divide 1m by  ║");
    Serial.println("║     ticks = meters_per_tick              ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
    enc_left = 0; enc_right = 0;
    current_mode = 30;
}

static void encoder_handle(const char* c) {
    int val;
    if (c[0] == 'c') {
        enc_left = 0; enc_right = 0;
        Serial.println("  Cleared.");
    } else if (c[0] == 't') {
        Serial.printf("  L=%u  R=%u ticks\n", (unsigned)enc_left, (unsigned)enc_right);
    } else if (c[0] == 'd') {
        // Raw pin state diagnostic — read pin 20 times over 2 seconds
        Serial.println("  Reading raw pin state (L=GPIO13, R=GPIO14):");
        for (int i = 0; i < 20; i++) {
            int l = digitalRead(PIN_ENC_LEFT);
            int r = digitalRead(PIN_ENC_RIGHT);
            Serial.printf("  [%02d] L=%d  R=%d  %s %s\n", i, l, r,
                l == 0 ? "(MAGNET)" : "(open)",
                r == 0 ? "(MAGNET)" : "(open)");
            delay(100);
        }
        Serial.println("  Done. If pin stays 0 constantly = sensor stuck on magnet.");
        Serial.println("  If flickers 0/1 at rest = noise/bad connection.");
    } else if (c[0] == 'p') {
        // High-speed probe: sample every 1ms for 3 seconds, report any LOW pulses
        Serial.println("  High-speed probe (3s) — rotate wheel SLOWLY now...");
        int low_count_l = 0, low_count_r = 0;
        int transitions_l = 0, transitions_r = 0;
        int prev_l = 1, prev_r = 1;
        unsigned long min_pulse_l = 999999, min_pulse_r = 999999;
        unsigned long pulse_start_l = 0, pulse_start_r = 0;

        for (int i = 0; i < 3000; i++) {
            int l = digitalRead(PIN_ENC_LEFT);
            int r = digitalRead(PIN_ENC_RIGHT);

            if (l == 0) {
                low_count_l++;
                if (prev_l == 1) { transitions_l++; pulse_start_l = micros(); }
            } else {
                if (prev_l == 0 && pulse_start_l > 0) {
                    unsigned long pw = micros() - pulse_start_l;
                    if (pw < min_pulse_l) min_pulse_l = pw;
                }
            }

            if (r == 0) {
                low_count_r++;
                if (prev_r == 1) { transitions_r++; pulse_start_r = micros(); }
            } else {
                if (prev_r == 0 && pulse_start_r > 0) {
                    unsigned long pw = micros() - pulse_start_r;
                    if (pw < min_pulse_r) min_pulse_r = pw;
                }
            }

            prev_l = l; prev_r = r;
            delay(1);
        }
        Serial.printf("  LEFT:  %d low samples, %d transitions, min pulse=%luus\n",
                      low_count_l, transitions_l, min_pulse_l == 999999 ? 0 : min_pulse_l);
        Serial.printf("  RIGHT: %d low samples, %d transitions, min pulse=%luus\n",
                      low_count_r, transitions_r, min_pulse_r == 999999 ? 0 : min_pulse_r);
        Serial.println("  If transitions but very short pulse → sensor too far from magnets.");
    } else if (sscanf(c, "f %d", &val) == 1) {
        motor_drive(val);
        Serial.printf("  Motor FWD pwm=%d\n", val);
    } else if (sscanf(c, "r %d", &val) == 1) {
        motor_drive(-val);
        Serial.printf("  Motor REV pwm=%d\n", val);
    } else if (c[0] == 's') {
        motor_drive(0);
        Serial.println("  Motor stopped.");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 4: STEERING SERVO CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════
static int steer_current = STEER_CENTER;
static int steer_step = 5;

static void steering_calibrate() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║  STEERING SERVO CALIBRATION              ║");
    Serial.println("║                                          ║");
    Serial.println("║  Commands:                               ║");
    Serial.println("║    <number>  = go to angle (e.g. 135)    ║");
    Serial.println("║    +         = increment by step         ║");
    Serial.println("║    -         = decrement by step         ║");
    Serial.println("║    s1/s5/s10 = set step size (1,5,10°)   ║");
    Serial.println("║    p <us>    = set pulse width directly  ║");
    Serial.println("║    d         = dead zone test (sweep)    ║");
    Serial.println("║    w         = sweep full range slowly   ║");
    Serial.println("║    q         = return to menu            ║");
    Serial.println("║                                          ║");
    Serial.println("║  PROCEDURE:                              ║");
    Serial.println("║  1. Type 'd' to measure dead zone        ║");
    Serial.println("║  2. Use +/- with s5 for coarse, s1 fine  ║");
    Serial.println("║  3. Find center (wheels straight)        ║");
    Serial.println("║  4. Find max left / max right limits     ║");
    Serial.println("║  5. Type 'p 1500' for fine pulse tuning  ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
    Serial.printf("  Currently at: %d° (step=5)\n", STEER_CENTER);
    steer_servo_write(STEER_CENTER);
    steer_current = STEER_CENTER;
    current_mode = 40;
}

static void steer_deadzone_test() {
    Serial.println("\n  ── DEAD ZONE TEST ──");
    Serial.println("  Sweeping from 100° to 170° in 1° steps (0.5s each)");
    Serial.println("  Watch when servo FIRST moves and LAST moves.\n");

    for (int angle = 100; angle <= 170; angle++) {
        steer_servo_write(angle);
        Serial.printf("  → %d°", angle);
        if (angle % 10 == 0) Serial.println();
        else Serial.print(" ");
        delay(500);
    }
    Serial.println("\n");
    Serial.println("  Note: angles where servo didn't move = DEAD ZONE");
    Serial.println("  The usable range is between first and last movement.");
    Serial.printf("  Returning to %d°\n", steer_current);
    steer_servo_write(steer_current);
}

static void steer_sweep_test() {
    Serial.println("\n  ── FULL RANGE SWEEP ──");
    Serial.println("  Sweeping 90° → 180° → 90° slowly...\n");
    for (int angle = 90; angle <= 180; angle += 2) {
        steer_servo_write(angle);
        Serial.printf("  %d°\r", angle);
        delay(200);
    }
    for (int angle = 180; angle >= 90; angle -= 2) {
        steer_servo_write(angle);
        Serial.printf("  %d°\r", angle);
        delay(200);
    }
    Serial.printf("\n  Done. Returning to %d°\n", steer_current);
    steer_servo_write(steer_current);
}

static void steer_set_pulse(int pulse_us) {
    if (pulse_us < 400 || pulse_us > 2600) {
        Serial.println("  Pulse must be 400-2600 us");
        return;
    }
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_us);
    // Back-calculate approximate angle
    int approx_angle = (pulse_us - 544) * 180 / 1856;
    steer_current = approx_angle;
    Serial.printf("  Pulse: %dus (≈%d°)\n", pulse_us, approx_angle);
}

static void steering_handle(const char* c) {
    int val;
    if (c[0] == '+') {
        steer_current += steer_step;
        if (steer_current > 180) steer_current = 180;
    } else if (c[0] == '-') {
        steer_current -= steer_step;
        if (steer_current < 0) steer_current = 0;
    } else if (strcmp(c, "s1") == 0) {
        steer_step = 1;
        Serial.println("  Step = 1°");
        return;
    } else if (strcmp(c, "s5") == 0) {
        steer_step = 5;
        Serial.println("  Step = 5°");
        return;
    } else if (strcmp(c, "s10") == 0) {
        steer_step = 10;
        Serial.println("  Step = 10°");
        return;
    } else if (c[0] == 'd') {
        steer_deadzone_test();
        return;
    } else if (c[0] == 'w') {
        steer_sweep_test();
        return;
    } else if (sscanf(c, "p %d", &val) == 1) {
        steer_set_pulse(val);
        return;
    } else {
        val = atoi(c);
        if (val >= 0 && val <= 180) steer_current = val;
        else { Serial.println("  Commands: +/-/s1/s5/s10/d/w/p <us>/<angle>/q"); return; }
    }
    steer_servo_write(steer_current);
    // Show pulse width too
    uint32_t pulse = 544 + (uint32_t)steer_current * 1856 / 180;
    Serial.printf("  Steering: %d° (pulse=%dus, step=%d)\n", steer_current, pulse, steer_step);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 5: SCANNING SERVO CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════
static int scan_current = 90;
static int scan_step = 5;

static void scan_calibrate() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║  SCANNING SERVO CALIBRATION              ║");
    Serial.println("║                                          ║");
    Serial.println("║  Commands:                               ║");
    Serial.println("║    <number>  = go to angle (e.g. 90)     ║");
    Serial.println("║    +/-       = increment/decrement step  ║");
    Serial.println("║    s1/s5/s10 = set step size             ║");
    Serial.println("║    r         = read ToF sensors now      ║");
    Serial.println("║    w         = sweep with live ToF       ║");
    Serial.println("║    p <us>    = pulse width directly      ║");
    Serial.println("║    q         = return to menu            ║");
    Serial.println("║                                          ║");
    Serial.println("║  PROCEDURE:                              ║");
    Serial.println("║  1. Place wall in FRONT of robot ~50cm   ║");
    Serial.println("║  2. Type 90, then 'r' — Sensor 2 should ║");
    Serial.println("║     read ~500mm (it faces forward)       ║");
    Serial.println("║  3. If wrong sensor reads short, the     ║");
    Serial.println("║     mount angles in tof_config.h are off ║");
    Serial.println("║  4. Type 'w' for sweep with live ToF     ║");
    Serial.println("║     to see how readings change with angle║");
    Serial.println("╚══════════════════════════════════════════╝\n");
    scan_servo_write(90);
    scan_current = 90;
    current_mode = 50;
}

static void scan_read_tof() {
    Serial.printf("  Servo: %d° | ", scan_current);
    for (int s = 0; s < 3; s++) {
        tof_sensors[s].read();
        Serial.printf("ToF%d=%4dmm(s%d) ", s,
            tof_sensors[s].ranging_data.range_mm,
            tof_sensors[s].ranging_data.range_status);
    }
    // Show computed true angles for each sensor at this servo position
    Serial.printf("\n         True angles: S0=%.0f° S1=%.0f° S2=%.0f°\n",
        fmod((scan_current - 90.0f) + TOF_ANGLE_0 + 360.0f, 360.0f),
        fmod((scan_current - 90.0f) + TOF_ANGLE_1 + 360.0f, 360.0f),
        fmod((scan_current - 90.0f) + TOF_ANGLE_2 + 360.0f, 360.0f));
}

static void scan_sweep_with_tof() {
    Serial.println("\n  ── SWEEP WITH LIVE TOF ──");
    Serial.println("  Sweeping 30° → 150° (step=10°), reading ToF at each position\n");
    for (int angle = 30; angle <= 150; angle += 10) {
        scan_servo_write(angle);
        scan_current = angle;
        delay(300);  // Let servo settle
        // Read sensors
        for (int s = 0; s < 3; s++) tof_sensors[s].read();
        Serial.printf("  %3d° | S0=%4dmm S1=%4dmm S2=%4dmm | angles: %.0f° %.0f° %.0f°\n",
            angle,
            tof_sensors[0].ranging_data.range_mm,
            tof_sensors[1].ranging_data.range_mm,
            tof_sensors[2].ranging_data.range_mm,
            fmod((angle - 90.0f) + TOF_ANGLE_0 + 360.0f, 360.0f),
            fmod((angle - 90.0f) + TOF_ANGLE_1 + 360.0f, 360.0f),
            fmod((angle - 90.0f) + TOF_ANGLE_2 + 360.0f, 360.0f));
    }
    Serial.println("\n  If wall is in front: Sensor 2's distance should be shortest near 90°");
    Serial.println("  If angles are wrong: adjust TOF_ANGLE_0/1/2 in tof_config.h");
    Serial.printf("  Returning to %d°\n", scan_current);
    scan_servo_write(scan_current);
}

static void scan_handle(const char* c) {
    int val;
    if (c[0] == '+') {
        scan_current += scan_step;
        if (scan_current > 180) scan_current = 180;
    } else if (c[0] == '-') {
        scan_current -= scan_step;
        if (scan_current < 0) scan_current = 0;
    } else if (strcmp(c, "s1") == 0) { scan_step = 1; Serial.println("  Step = 1°"); return; }
    else if (strcmp(c, "s5") == 0) { scan_step = 5; Serial.println("  Step = 5°"); return; }
    else if (strcmp(c, "s10") == 0) { scan_step = 10; Serial.println("  Step = 10°"); return; }
    else if (c[0] == 'r') { scan_read_tof(); return; }
    else if (c[0] == 'w') { scan_sweep_with_tof(); return; }
    else if (sscanf(c, "p %d", &val) == 1) {
        if (val >= 400 && val <= 2600) {
            float pulse_us = val;
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_us / 20000.0f * 100.0f);
            scan_current = (val - SERVO_PULSE_MIN_US) * 180 / (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
            Serial.printf("  Pulse: %dus (≈%d°)\n", val, scan_current);
        }
        return;
    } else {
        val = atoi(c);
        if (val >= 0 && val <= 180) scan_current = val;
        else { Serial.println("  Commands: +/-/s1/s5/s10/r/w/p <us>/<angle>/q"); return; }
    }
    scan_servo_write(scan_current);
    uint32_t pulse = SERVO_PULSE_MIN_US + (uint32_t)scan_current * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180;
    Serial.printf("  Scan servo: %d° (pulse=%dus, step=%d)\n", scan_current, pulse, scan_step);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 6: MOTOR SPEED CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════
static void motor_calibrate() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║  MOTOR SPEED CALIBRATION                 ║");
    Serial.println("║                                          ║");
    Serial.println("║  Commands:                               ║");
    Serial.println("║    f PWM  = forward (0-255)              ║");
    Serial.println("║    r PWM  = reverse (0-255)              ║");
    Serial.println("║    s      = stop                         ║");
    Serial.println("║    q      = return to menu               ║");
    Serial.println("║                                          ║");
    Serial.println("║  PROCEDURE:                              ║");
    Serial.println("║  1. Find minimum PWM where motor turns   ║");
    Serial.println("║     (currently set to 35)                ║");
    Serial.println("║  2. Find max safe PWM (currently 180)    ║");
    Serial.println("║  3. Update MOTOR_MIN_USEFUL, MOTOR_MAX   ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
    current_mode = 60;
}

static void motor_handle(const char* c) {
    int val;
    if (sscanf(c, "f %d", &val) == 1) {
        motor_drive(val);
        Serial.printf("  Motor FWD pwm=%d (%.0f%%)\n", val, val/255.0f*100.0f);
    } else if (sscanf(c, "r %d", &val) == 1) {
        motor_drive(-val);
        Serial.printf("  Motor REV pwm=%d (%.0f%%)\n", val, val/255.0f*100.0f);
    } else if (c[0] == 's') {
        motor_drive(0);
        Serial.println("  Motor stopped.");
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
// MODE 7: SENSOR PLACEMENT VERIFICATION
// ═══════════════════════════════════════════════════════════════════════════════
static void placement_calibrate() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║  SENSOR PLACEMENT VERIFICATION           ║");
    Serial.println("║                                          ║");
    Serial.println("║  Commands:                               ║");
    Serial.println("║    i = IMU orientation test              ║");
    Serial.println("║    t = ToF placement test                ║");
    Serial.println("║    l = Live IMU axes display             ║");
    Serial.println("║    d = ToF distance comparison           ║");
    Serial.println("║    q = return to menu                    ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
    current_mode = 70;
}

static void placement_imu_test() {
    Serial.println("\n  ── IMU ORIENTATION TEST ──");
    Serial.println("  Place robot flat on table, still.");
    Serial.println("  Reading 100 samples...\n");

    // Pause ToF to free I2C bus
    for (int i = 0; i < 3; i++) tof_sensors[i].stopContinuous();
    delay(10);

    float sum_ax=0, sum_ay=0, sum_az=0;
    float ax, ay, az, gx, gy, gz;
    for (int i = 0; i < 100; i++) {
        imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz);
        sum_ax += ax; sum_ay += ay; sum_az += az;
        delay(10);
    }
    float avg_ax = sum_ax/100, avg_ay = sum_ay/100, avg_az = sum_az/100;

    // Restart ToF
    for (int i = 0; i < 3; i++) tof_sensors[i].startContinuous(50);

    Serial.printf("  Raw accel: X=%+.2f  Y=%+.2f  Z=%+.2f m/s²\n", avg_ax, avg_ay, avg_az);
    Serial.println("\n  EXPECTED for correct mounting (X=forward, Z=up):");
    Serial.println("    Flat:     X≈0    Y≈0    Z≈+9.81");
    Serial.println("    Tilted forward:  X>0 (positive)");
    Serial.println("    Tilted left:     Y>0 (positive)");

    Serial.println("\n  YOUR RESULT:");
    // Check which axis has gravity
    float mag = sqrtf(avg_ax*avg_ax + avg_ay*avg_ay + avg_az*avg_az);
    Serial.printf("  Magnitude: %.2f m/s² (should be ~9.81)\n", mag);

    if (fabsf(avg_az) > 8.0f) {
        Serial.println("  ✅ Z-axis has gravity — IMU is FLAT (correct basic orientation)");
        if (avg_az > 0) Serial.println("  ✅ Z is positive — chip faces UP (correct)");
        else Serial.println("  ⚠️  Z is negative — chip faces DOWN (flip it or change rotation matrix)");
    } else if (fabsf(avg_ax) > 8.0f) {
        Serial.println("  ⚠️  X-axis has gravity — IMU is mounted VERTICALLY (X pointing up/down)");
        Serial.println("      Current firmware has rotation matrix for this. Check if it's intentional.");
    } else if (fabsf(avg_ay) > 8.0f) {
        Serial.println("  ❌ Y-axis has gravity — IMU is mounted SIDEWAYS. Fix mounting or rotation matrix.");
    }

    // Check tilt
    float tilt_x = atan2f(avg_ax, avg_az) * 180.0f / 3.14159f;
    float tilt_y = atan2f(avg_ay, avg_az) * 180.0f / 3.14159f;
    Serial.printf("\n  Tilt: pitch=%.1f° roll=%.1f° (should both be <3° on flat surface)\n", tilt_x, tilt_y);
    if (fabsf(tilt_x) > 3.0f || fabsf(tilt_y) > 3.0f) {
        Serial.println("  ⚠️  Robot not level or IMU misaligned. Check mounting.");
    } else {
        Serial.println("  ✅ IMU is level — mounting looks good.");
    }
}

static void placement_imu_live() {
    // Pause ToF ranging to free I2C bus for IMU
    for (int i = 0; i < 3; i++) tof_sensors[i].stopContinuous();
    delay(10);

    float ax, ay, az, gx, gy, gz;
    imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz);

    // Restart ToF
    for (int i = 0; i < 3; i++) tof_sensors[i].startContinuous(50);

    // Apply rotation matrix (same as main firmware)
    float robot_ax = -az;
    float robot_ay = ay;
    float robot_az = ax;
    float robot_gx = -gz;
    float robot_gy = gy;
    float robot_gz = gx;

    Serial.printf("  RAW:   aX=%+.2f aY=%+.2f aZ=%+.2f | gX=%+.3f gY=%+.3f gZ=%+.3f\n",
                  ax, ay, az, gx, gy, gz);
    Serial.printf("  ROBOT: aX=%+.2f aY=%+.2f aZ=%+.2f | gX=%+.3f gY=%+.3f gZ=%+.3f\n",
                  robot_ax, robot_ay, robot_az, robot_gx, robot_gy, robot_gz);
    Serial.println("         (Robot frame: X=forward, Y=left, Z=up)");
    Serial.println("  Tilt robot FORWARD → robot aX should increase");
    Serial.println("  Rotate robot LEFT  → robot gZ should be positive\n");
}

static void placement_tof_test() {
    Serial.println("\n  ── TOF PLACEMENT TEST ──");
    Serial.println("  This checks if sensors are pointing in correct directions.");
    Serial.println("  Place a wall/object on ONE side only, note which sensor reads short.\n");
    Serial.println("  Expected mounting (120° apart on triangular plate):");
    Serial.printf("    Sensor 0 (GPIO %d XSHUT): mount angle = %.0f° (right-rear)\n", PIN_XSHUT_0, TOF_ANGLE_0);
    Serial.printf("    Sensor 1 (GPIO %d XSHUT): mount angle = %.0f° (left-rear)\n", PIN_XSHUT_1, TOF_ANGLE_1);
    Serial.printf("    Sensor 2 (GPIO %d XSHUT): mount angle = %.0f° (forward)\n", PIN_XSHUT_2, TOF_ANGLE_2);
    Serial.println("\n  TEST: Point the robot FORWARD at a wall ~50cm away.");
    Serial.println("  Sensor 2 should read ~500mm. Others should read longer.\n");

    Serial.println("  Reading 10 samples per sensor...");
    for (int s = 0; s < 3; s++) {
        long sum = 0; int count = 0;
        for (int i = 0; i < 10; i++) {
            tof_sensors[s].read();
            if (tof_sensors[s].ranging_data.range_status == 0) {
                sum += tof_sensors[s].ranging_data.range_mm;
                count++;
            }
            delay(60);
        }
        if (count > 0) {
            Serial.printf("    Sensor %d (%.0f°): %dmm avg\n", s, (s==0)?TOF_ANGLE_0:(s==1)?TOF_ANGLE_1:TOF_ANGLE_2, (int)(sum/count));
        } else {
            Serial.printf("    Sensor %d: NO READING\n", s);
        }
    }

    Serial.println("\n  VERIFY:");
    Serial.println("  1. Move object to LEFT of robot  → Sensor 1 should drop");
    Serial.println("  2. Move object to RIGHT of robot → Sensor 0 should drop");
    Serial.println("  3. Move object in FRONT          → Sensor 2 should drop");
    Serial.println("  If wrong sensor responds, swap XSHUT pins or mount angles in tof_config.h");
}

static void placement_tof_live() {
    Serial.print("  ");
    for (int s = 0; s < 3; s++) {
        tof_sensors[s].read();
        float angle = (s==0)?TOF_ANGLE_0:(s==1)?TOF_ANGLE_1:TOF_ANGLE_2;
        Serial.printf("S%d(%.0f°)=%4dmm  ", s, angle, tof_sensors[s].ranging_data.range_mm);
    }
    Serial.println();
}

static void placement_handle(const char* c) {
    if (c[0] == 'i') { placement_imu_test(); }
    else if (c[0] == 'l') { placement_imu_live(); }
    else if (c[0] == 't') { placement_tof_test(); }
    else if (c[0] == 'd') { placement_tof_live(); }
    else { Serial.println("  Commands: i=IMU test, t=ToF test, l=live IMU, d=live ToF distances"); }
}


// ═══════════════════════════════════════════════════════════════════════════════
// MODE 8: ALIGNMENT (Servo + ToF + IMU all pointing same direction)
// ═══════════════════════════════════════════════════════════════════════════════
static void align_calibrate() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║  ALIGNMENT CALIBRATION                   ║");
    Serial.println("║  Aligns servo, ToF, and IMU forward.     ║");
    Serial.println("║                                          ║");
    Serial.println("║  Commands:                               ║");
    Serial.println("║    a = Auto-align (find forward w/ ToF)  ║");
    Serial.println("║    m = Manual (show servo+ToF+IMU)       ║");
    Serial.println("║    i = IMU forward tilt check            ║");
    Serial.println("║    +/- = nudge servo ±1°                 ║");
    Serial.println("║    <num> = set servo angle               ║");
    Serial.println("║    q = return to menu                    ║");
    Serial.println("║                                          ║");
    Serial.println("║  PROCEDURE:                              ║");
    Serial.println("║  1. Place wall in front ~50cm            ║");
    Serial.println("║  2. Type 'a' to find true forward        ║");
    Serial.println("║  3. Type 'i' to verify IMU alignment     ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
    scan_servo_write(90);
    scan_current = 90;
    current_mode = 80;
}

static void align_auto() {
    Serial.println("\n  ── AUTO-ALIGN: Finding true forward ──");
    Serial.println("  Sweeping servo 60°→120°, reading Sensor 2 (forward)...\n");
    int best_angle = 90, min_dist = 9999;
    for (int angle = 60; angle <= 120; angle += 2) {
        scan_servo_write(angle);
        delay(200);
        tof_sensors[2].read();
        int dist = tof_sensors[2].ranging_data.range_mm;
        int status = tof_sensors[2].ranging_data.range_status;
        if (status == 0 && dist > 0 && dist < min_dist) { min_dist = dist; best_angle = angle; }
        Serial.printf("  %3d° → S2=%4dmm%s\n", angle, dist, (dist==min_dist && status==0)?" ◄":"");
    }
    Serial.printf("\n  RESULT: Forward = servo %d° (wall at %dmm)\n", best_angle, min_dist);
    if (best_angle != 90) {
        Serial.printf("  ⚠️  Off by %+d° → set SERVO_CENTER_DEG = %d\n", best_angle-90, 90-(best_angle-90));
    } else {
        Serial.println("  ✅ 90° = forward (aligned)");
    }
    scan_servo_write(best_angle);
    scan_current = best_angle;
}

static void align_imu_check() {
    Serial.println("\n  ── IMU FORWARD CHECK ──");
    Serial.println("  Tilt robot FORWARD (nose down) now. Reading 20 samples...\n");
    for (int i = 0; i < 3; i++) tof_sensors[i].stopContinuous();
    delay(10);
    for (int j = 0; j < 20; j++) {
        float ax, ay, az, gx, gy, gz;
        imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz);
        float robot_ax = -az;
        Serial.printf("  robot aX=%+.2f %s\n", robot_ax, (robot_ax > 1.0f)?"✅ FORWARD":"");
        delay(100);
    }
    for (int i = 0; i < 3; i++) tof_sensors[i].startContinuous(50);
    Serial.println("\n  aX positive when tilting forward = IMU aligned ✅");
    Serial.println("  aX negative = IMU is mounted backwards");
}

static void align_manual() {
    for (int i = 0; i < 3; i++) tof_sensors[i].stopContinuous();
    delay(10);
    float ax, ay, az, gx, gy, gz;
    imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz);
    for (int i = 0; i < 3; i++) tof_sensors[i].startContinuous(50);
    delay(50);
    for (int s = 0; s < 3; s++) tof_sensors[s].read();
    Serial.printf("  Servo=%d° | S0=%dmm S1=%dmm S2=%dmm | IMU aX=%+.2f\n",
        scan_current,
        tof_sensors[0].ranging_data.range_mm,
        tof_sensors[1].ranging_data.range_mm,
        tof_sensors[2].ranging_data.range_mm, -az);
}

static void align_handle(const char* c) {
    int val;
    if (c[0] == 'a') { align_auto(); }
    else if (c[0] == 'm') { align_manual(); }
    else if (c[0] == 'i') { align_imu_check(); }
    else if (c[0] == '+') { scan_current++; scan_servo_write(scan_current); Serial.printf("  %d°\n", scan_current); }
    else if (c[0] == '-') { scan_current--; scan_servo_write(scan_current); Serial.printf("  %d°\n", scan_current); }
    else if ((val = atoi(c)) >= 0 && val <= 180) { scan_current = val; scan_servo_write(scan_current); Serial.printf("  %d°\n", scan_current); }
    else { Serial.println("  Commands: a/m/i/+/-/<angle>/q"); }
}


// ═══════════════════════════════════════════════════════════════════════════════
// MENU
// ═══════════════════════════════════════════════════════════════════════════════
static void show_menu() {
    Serial.println("\n╔══════════════════════════════════════════╗");
    Serial.println("║       DriftBot Calibration Tool          ║");
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.println("║  1 = IMU (gyro bias + accel offset)      ║");
    Serial.println("║  2 = ToF (distance offset per sensor)    ║");
    Serial.println("║  3 = Encoder (ticks per revolution)      ║");
    Serial.println("║  4 = Steering servo (center + limits)    ║");
    Serial.println("║  5 = Scanning servo (angle verify)       ║");
    Serial.println("║  6 = Motor (PWM speed tuning)            ║");
    Serial.println("║  7 = Sensor placement (IMU + ToF verify) ║");
    Serial.println("║  8 = Alignment (servo + ToF + IMU fwd)   ║");
    Serial.println("║  h = Show this menu                      ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("\n═══ DriftBot Calibration Tool ═══\n");

    // I2C for IMU + ToF
    Wire.begin(PIN_TOF_SDA, PIN_TOF_SCL);
    Wire.setClock(400000);

    // Wake MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); Wire.write(0x08); // ±500°/s
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); Wire.write(0x00); // ±2g
    Wire.endTransmission();
    Serial.println("[OK] IMU ready");

    // ToF sensors
    for (int i = 0; i < 3; i++) {
        pinMode(tof_xshut[i], OUTPUT);
        digitalWrite(tof_xshut[i], LOW);
    }
    delay(50);
    for (int i = 0; i < 3; i++) {
        digitalWrite(tof_xshut[i], HIGH);
        delay(150);
        tof_sensors[i].setBus(&Wire);
        tof_sensors[i].setTimeout(500);
        if (tof_sensors[i].init()) {
            tof_sensors[i].setAddress(tof_addr[i]);
            tof_sensors[i].setDistanceMode(VL53L1X::Long);
            tof_sensors[i].setMeasurementTimingBudget(50000);
            tof_sensors[i].startContinuous(50);
            Serial.printf("[OK] ToF %d at 0x%02X\n", i, tof_addr[i]);
        } else {
            Serial.printf("[FAIL] ToF %d\n", i);
        }
    }

    // Encoders (timer-polled, not interrupt-driven)
    pinMode(PIN_ENC_LEFT, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
    // Start hardware timer at 500Hz (2ms period) for encoder polling
    enc_timer = timerBegin(0, 80, true);  // timer 0, prescaler 80 (1MHz), count up
    timerAttachInterrupt(enc_timer, enc_timer_isr, true);  // edge triggered
    timerAlarmWrite(enc_timer, 2000, true);  // 2000us = 2ms = 500Hz, auto-reload
    timerAlarmEnable(enc_timer);
    Serial.println("[OK] Encoders (timer-polled 500Hz)");

    // Motor enable
    pinMode(PIN_MOTOR_REN, OUTPUT);
    pinMode(PIN_MOTOR_LEN, OUTPUT);
    digitalWrite(PIN_MOTOR_REN, HIGH);
    digitalWrite(PIN_MOTOR_LEN, HIGH);

    // MCPWM: Motor FIRST (Unit 1, Timer 1)
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, PIN_MOTOR_RPWM);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, PIN_MOTOR_LPWM);
    mcpwm_config_t motor_cfg = {.frequency=1000, .cmpr_a=0, .cmpr_b=0, .duty_mode=MCPWM_DUTY_MODE_0, .counter_mode=MCPWM_UP_COUNTER};
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &motor_cfg);
    motor_drive(0);

    // MCPWM: Steering LAST (Unit 1, Timer 0)
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PIN_STEERING);
    mcpwm_config_t steer_cfg = {.frequency=50, .cmpr_a=0, .cmpr_b=0, .duty_mode=MCPWM_DUTY_MODE_0, .counter_mode=MCPWM_UP_COUNTER};
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &steer_cfg);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    steer_servo_write(STEER_CENTER);

    // MCPWM: Scan servo (Unit 0, Timer 0)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_SERVO);
    mcpwm_config_t scan_cfg = {.frequency=50, .cmpr_a=0, .cmpr_b=0, .duty_mode=MCPWM_DUTY_MODE_0, .counter_mode=MCPWM_UP_COUNTER};
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &scan_cfg);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    scan_servo_write(90);

    Serial.println("[OK] All actuators");
    show_menu();
}

// ═══════════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    // Non-blocking serial read
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd[cmd_len] = '\0';

                // Global quit
                if (cmd[0] == 'q' && current_mode != 0) {
                    motor_drive(0);
                    current_mode = 0;
                    Serial.println("  Returned to menu.");
                    show_menu();
                }
                // Menu selection
                else if (current_mode == 0) {
                    switch (cmd[0]) {
                        case '1': imu_calibrate(); break;
                        case '2': tof_calibrate(); break;
                        case '3': encoder_calibrate(); break;
                        case '4': steering_calibrate(); break;
                        case '5': scan_calibrate(); break;
                        case '6': motor_calibrate(); break;
                        case '7': placement_calibrate(); break;
                        case '8': align_calibrate(); break;
                        case 'h': show_menu(); break;
                        default: Serial.println("  Unknown. Type h for menu."); break;
                    }
                }
                // Sub-mode handlers
                else if (current_mode == 10 && cmd[0] == 'r') { imu_live(); }
                else if (current_mode == 20) { tof_run_calibration(); }
                else if (current_mode == 21 && cmd[0] == 'r') { tof_live(); }
                else if (current_mode == 30) { encoder_handle(cmd); }
                else if (current_mode == 40) { steering_handle(cmd); }
                else if (current_mode == 50) { scan_handle(cmd); }
                else if (current_mode == 60) { motor_handle(cmd); }
                else if (current_mode == 70) { placement_handle(cmd); }
                else if (current_mode == 80) { align_handle(cmd); }

                cmd_len = 0;
            }
        } else if (cmd_len < 62) {
            cmd[cmd_len++] = c;
        }
    }

    // Encoder display only when motor is actively running
    if (current_mode == 30 && motor_is_running()) {
        static unsigned long last = 0;
        if (millis() - last > 500) {
            last = millis();
            Serial.printf("  [live] L=%u R=%u\n", (unsigned)enc_left, (unsigned)enc_right);
        }
    }
}
