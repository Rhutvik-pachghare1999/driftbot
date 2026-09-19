/*
 * motor_driver.cpp — DC Motor + Steering Servo
 *
 * ALL raw MCPWM + NO LEDC anywhere in firmware:
 *   Steering: MCPWM Unit 1, Timer 0, Op A, PIN_STEERING (GPIO 10, 50Hz servo)
 *   Motor:    MCPWM Unit 1, Timer 1, Op A/B, GPIO 11/12 (1kHz H-bridge)
 *
 * Scanning servo uses MCPWM Unit 0 (separate hardware).
 * Zero LEDC calls = zero interference with MCPWM GPIO matrix.
 */

#include <Arduino.h>
#include "driver/mcpwm.h"
#include "pins.h"
#include "motor_config.h"
#include "motor_driver.h"

// ── State ─────────────────────────────────────────────────────────────────
static int  current_steer   = STEER_CENTER;
static int  current_pwm     = 0;
static bool current_forward = true;
static bool motor_active    = false;
static bool watchdog_enabled = false;
static unsigned long last_cmd_time = 0;
#define WATCHDOG_TIMEOUT_MS  1000

// ── Motor direction (volatile + IRAM for ISR safety) ──────────────────────
static volatile int8_t motor_direction = 0;

int8_t IRAM_ATTR motor_get_direction() {
    return motor_direction;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC: motor_get_steering()
// ═══════════════════════════════════════════════════════════════════════════════
int motor_get_steering() {
    return current_steer;
}

// ── Steering (MCPWM Unit 1, Timer 0, Op A — Core 0 exclusive, no WiFi here)
static int last_steer_written = -1;
static volatile int steer_target = STEER_CENTER;

void motor_update_steering_hw() {
    int angle = steer_target;
    if (angle == last_steer_written) return;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    last_steer_written = angle;
    uint32_t pulse_us = 544 + (uint32_t)angle * 1856 / 180;
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_us);
}

static void steer_write_raw(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    steer_target = angle;
}

// ── Motor (MCPWM Unit 1, Timer 1, Op A/B) ────────────────────────────────
static void apply_motor(int pwm, bool forward) {
    float duty = (float)pwm / 255.0f * 100.0f;

    if (pwm == 0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
        motor_active = false;
        motor_direction = 0;
    } else if (forward) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        motor_active = true;
        motor_direction = 1;
    } else {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, duty);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        motor_active = true;
        motor_direction = -1;
    }
    current_pwm = pwm;
    current_forward = forward;
}

// ═══════════════════════════════════════════════════════════════════════════════
void motor_init() {
    // ── Motor enable ──────────────────────────────────────────────────────────
    pinMode(PIN_MOTOR_REN, OUTPUT);
    pinMode(PIN_MOTOR_LEN, OUTPUT);
    digitalWrite(PIN_MOTOR_REN, HIGH);
    digitalWrite(PIN_MOTOR_LEN, HIGH);

    // ── Motor FIRST: MCPWM Unit 1, Timer 1, 1kHz ─────────────────────────────
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, PIN_MOTOR_RPWM);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, PIN_MOTOR_LPWM);
    mcpwm_config_t motor_cfg = {
        .frequency = 1000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &motor_cfg);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, 0);

    // ── Steering LAST: MCPWM Unit 1, Timer 0, 50Hz ───────────────────────────
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PIN_STEERING);
    mcpwm_config_t steer_cfg = {
        .frequency = 50,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &steer_cfg);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    steer_target = STEER_CENTER;
    last_steer_written = -1;
    uint32_t center_us = 544 + (uint32_t)STEER_CENTER * 1856 / 180;
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, center_us);
    current_steer = STEER_CENTER;

    last_cmd_time = millis();
    Serial.printf("[STEER] GPIO%d center=%d°\n", PIN_STEERING, STEER_CENTER);
    Serial.printf("[MOTOR] RPWM=GPIO%d LPWM=GPIO%d\n", PIN_MOTOR_RPWM, PIN_MOTOR_LPWM);
}

// ═══════════════════════════════════════════════════════════════════════════════
void motor_set_speed(float speed_normalized) {
    last_cmd_time = millis();
    if (speed_normalized > 1.0f) speed_normalized = 1.0f;
    if (speed_normalized < -1.0f) speed_normalized = -1.0f;

    bool forward = (speed_normalized >= 0);
    float abs_speed = fabsf(speed_normalized);
    if (abs_speed < 0.05f) { apply_motor(0, true); return; }

    int pwm = (int)(MOTOR_MIN_USEFUL + abs_speed * (MOTOR_MAX - MOTOR_MIN_USEFUL));
    apply_motor(pwm, forward);
}

// ═══════════════════════════════════════════════════════════════════════════════
void motor_set_steering(int angle_deg) {
    last_cmd_time = millis();
    if (angle_deg < STEER_MIN) angle_deg = STEER_MIN;
    if (angle_deg > STEER_MAX) angle_deg = STEER_MAX;
    current_steer = angle_deg;
    steer_write_raw(current_steer);
}

// ═══════════════════════════════════════════════════════════════════════════════
void motor_cmd_vel(float linear_x, float angular_z) {
    last_cmd_time = millis();
    watchdog_enabled = true;

    // Only update motor speed if linear command given
    if (fabsf(linear_x) > 0.01f) {
        motor_set_speed(linear_x / MAX_SPEED_MS);
    } else if (fabsf(angular_z) < 0.01f) {
        // Both zero = full stop + center steering
        motor_set_speed(0);
        motor_set_steering(STEER_CENTER);
    }
    // If only angular given (turn in place), keep current motor speed

    // Only update steering if angular command given
    if (fabsf(angular_z) > 0.01f) {
        float steer_norm = angular_z / MAX_TURN_RATE;
        if (steer_norm > 1.0f) steer_norm = 1.0f;
        if (steer_norm < -1.0f) steer_norm = -1.0f;

        int steer_angle;
        if (steer_norm >= 0) {
            steer_angle = STEER_CENTER + (int)(steer_norm * (STEER_MAX - STEER_CENTER));
        } else {
            steer_angle = STEER_CENTER + (int)(steer_norm * (STEER_CENTER - STEER_MIN));
        }
        motor_set_steering(steer_angle);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
void motor_update() {
    if (watchdog_enabled && motor_active && (millis() - last_cmd_time > WATCHDOG_TIMEOUT_MS)) {
        apply_motor(0, true);
        watchdog_enabled = false;
        Serial.println("[MOTOR] cmd_vel timeout — motor stopped.");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
void motor_stop() {
    apply_motor(0, true);
    steer_write_raw(STEER_CENTER);
    current_steer = STEER_CENTER;
    watchdog_enabled = false;
}

// ═══════════════════════════════════════════════════════════════════════════════
bool motor_handle_serial(const char* cmd) {
    int ival; float fval;

    if (sscanf(cmd, "drive %f", &fval) == 1) {
        motor_set_speed(fval);
        Serial.printf("[MOTOR] pwm=%d %s\n", current_pwm, current_forward ? "FWD" : "REV");
        return true;
    }
    if (sscanf(cmd, "steer %d", &ival) == 1) {
        motor_set_steering(ival);
        Serial.printf("[STEER] %d°\n", current_steer);
        return true;
    }
    if (strcmp(cmd, "brake") == 0) {
        motor_stop();
        Serial.println("[MOTOR] STOPPED");
        return true;
    }
    if (strcmp(cmd, "motor status") == 0) {
        Serial.printf("[MOTOR] pwm=%d %s steer=%d° dir=%d\n",
                      current_pwm, current_forward ? "FWD" : "REV",
                      current_steer, motor_direction);
        return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
void motor_print_help() {
    Serial.println("  drive <-1..1>  Motor speed (-=reverse)");
    Serial.println("  steer <deg>    Steering (115=R, 135=C, 145=L)");
    Serial.println("  brake          Emergency stop");
    Serial.println("  motor status   Show state");
}
