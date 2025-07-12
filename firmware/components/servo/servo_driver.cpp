/*
 * servo_driver.cpp — Scanning Servo (Raw MCPWM Unit 0)
 *
 * Uses raw ESP-IDF MCPWM driver on Unit 0, Timer 0, Operator A.
 * No ESP32Servo library — avoids all conflicts with steering (Unit 1).
 *
 * Sinusoidal motion: angle = center + amplitude * sin(phase)
 */

#include <Arduino.h>
#include <math.h>
#include "driver/mcpwm.h"
#include "pins.h"
#include "servo_config.h"
#include "servo_driver.h"

// ── Motion modes ──────────────────────────────────────────────────────────────
enum ServoMode { MODE_IDLE, MODE_SINE, MODE_HOLD };

// ── State ─────────────────────────────────────────────────────────────────────
static ServoMode     mode        = MODE_IDLE;
static float         phase       = 0.0f;
static float         center      = SERVO_CENTER_DEG;
static float         amplitude   = SERVO_AMPLITUDE_DEG;
static float         speed       = SERVO_SPEED_RAD_S;
static float         current_deg = SERVO_CENTER_DEG;
static unsigned long last_update = 0;

// ── MCPWM write ───────────────────────────────────────────────────────────────
static float clamp_deg(float deg) {
    if (deg < (float)SERVO_LIMIT_MIN) return (float)SERVO_LIMIT_MIN;
    if (deg > (float)SERVO_LIMIT_MAX) return (float)SERVO_LIMIT_MAX;
    return deg;
}

static int last_written_deg = -1;

static void write_servo(float deg) {
    current_deg = clamp_deg(deg);
    int int_deg = (int)(current_deg + 0.5f);
    if (int_deg != last_written_deg) {
        float pulse_us = SERVO_PULSE_MIN_US + (float)int_deg * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180.0f;
        float duty = pulse_us / 20000.0f * 100.0f;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
        last_written_deg = int_deg;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
void servo_init() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_SERVO);
    mcpwm_config_t cfg = {
        .frequency = 50,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    write_servo(center);
    mode = MODE_IDLE;
    last_update = millis();

    Serial.printf("[SERVO] Init: GPIO %d (MCPWM0), center=%.0f°, amp=%.0f°, speed=%.2f rad/s\n",
                  PIN_SERVO, center, amplitude, speed);
}

// ═══════════════════════════════════════════════════════════════════════════════
void servo_update() {
    unsigned long now = millis();
    unsigned long elapsed = now - last_update;
    if (elapsed < SERVO_UPDATE_MS) return;
    last_update = now;

    if (mode != MODE_SINE) return;

    float dt = elapsed / 1000.0f;
    phase += speed * dt;
    if (phase > 2.0f * PI) phase -= 2.0f * PI;

    float target = center + amplitude * sinf(phase);
    write_servo(target);
}

// ═══════════════════════════════════════════════════════════════════════════════
bool servo_handle_serial(const char* cmd) {
    float fval;
    int ival;

    if (strcmp(cmd, "start") == 0) {
        phase = 0.0f;
        mode = MODE_SINE;
        Serial.printf("[SERVO] Sine ON: center=%.0f° amp=%.0f° speed=%.2f rad/s\n",
                      center, amplitude, speed);
        return true;
    }
    if (strcmp(cmd, "stop") == 0) {
        mode = MODE_IDLE;
        Serial.printf("[SERVO] Stopped at %.0f°\n", current_deg);
        return true;
    }
    if (sscanf(cmd, "go %d", &ival) == 1) {
        mode = MODE_HOLD;
        write_servo((float)ival);
        Serial.printf("[SERVO] Hold: %d°\n", (int)current_deg);
        return true;
    }
    if (sscanf(cmd, "center %f", &fval) == 1) {
        center = clamp_deg(fval);
        Serial.printf("[SERVO] Center = %.0f°\n", center);
        return true;
    }
    if (sscanf(cmd, "amp %f", &fval) == 1) {
        if (fval >= 0.0f && fval <= 90.0f) {
            amplitude = fval;
            Serial.printf("[SERVO] Amplitude = %.0f°\n", amplitude);
        }
        return true;
    }
    if (sscanf(cmd, "speed %f", &fval) == 1) {
        if (fval > 0.0f && fval <= 20.0f) {
            speed = fval;
            Serial.printf("[SERVO] Speed = %.2f rad/s\n", speed);
        }
        return true;
    }
    if (strcmp(cmd, "status") == 0) {
        const char* mode_str = (mode == MODE_SINE) ? "SINE" :
                               (mode == MODE_HOLD) ? "HOLD" : "IDLE";
        Serial.printf("[SERVO] %s pos=%.0f° center=%.0f° amp=%.0f° speed=%.2f\n",
                      mode_str, current_deg, center, amplitude, speed);
        return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
void servo_print_help() {
    Serial.println("  start         Begin sinusoidal sweep");
    Serial.println("  stop          Freeze at current position");
    Serial.println("  go <deg>      Hold at specific angle");
    Serial.println("  center <deg>  Set sweep midpoint");
    Serial.println("  amp <deg>     Set sweep amplitude (0–90)");
    Serial.println("  speed <rad/s> Set oscillation speed");
    Serial.println("  status        Show servo settings");
}

// ═══════════════════════════════════════════════════════════════════════════════
float servo_get_position() {
    return current_deg;
}
