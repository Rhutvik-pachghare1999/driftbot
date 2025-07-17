/*
 * encoder_driver.cpp — Hall Encoder with Direction Detection
 *
 * DESIGN: ISR only counts raw ticks (no function calls in ISR).
 *         Direction is applied when ticks are READ, not in the ISR.
 *         This avoids IRAM/cache issues on ESP32-S3 dual-core.
 *
 * HARDWARE: A3144 Hall Effect Sensor (active LOW, South-pole detect)
 *   - 10 South-facing magnets per wheel (36° apart)
 *   - FALLING edge interrupt + 5ms debounce
 *   - Distance per tick = π × 0.064m / 10 = 0.0201m
 */

#include <Arduino.h>
#include "pins.h"
#include "motor_driver.h"
#include "encoder_driver.h"

// ── Raw tick counters (ISR only increments, never decrements) ─────────────
static volatile uint32_t raw_left  = 0;
static volatile uint32_t raw_right = 0;

// ── Signed accumulators (updated in get functions using motor direction) ───
static int32_t signed_left  = 0;
static int32_t signed_right = 0;
static uint32_t prev_raw_left  = 0;
static uint32_t prev_raw_right = 0;

// ── Debounce (5ms) ────────────────────────────────────────────────────────
#define DEBOUNCE_US  5000

static volatile unsigned long last_left_us  = 0;
static volatile unsigned long last_right_us = 0;

// ── ISRs — MINIMAL: just count, no function calls ────────────────────────
static void IRAM_ATTR isr_left() {
    unsigned long now = micros();
    if (now - last_left_us > DEBOUNCE_US) {
        raw_left++;
        last_left_us = now;
    }
}

static void IRAM_ATTR isr_right() {
    unsigned long now = micros();
    if (now - last_right_us > DEBOUNCE_US) {
        raw_right++;
        last_right_us = now;
    }
}

// ── Update signed counts using motor direction ────────────────────────────
static void update_signed() {
    uint32_t cur_left = raw_left;
    uint32_t cur_right = raw_right;

    uint32_t delta_left  = cur_left - prev_raw_left;
    uint32_t delta_right = cur_right - prev_raw_right;

    int8_t dir = motor_get_direction();
    if (dir >= 0) {
        signed_left  += (int32_t)delta_left;
        signed_right += (int32_t)delta_right;
    } else {
        signed_left  -= (int32_t)delta_left;
        signed_right -= (int32_t)delta_right;
    }

    prev_raw_left = cur_left;
    prev_raw_right = cur_right;
}

// ═══════════════════════════════════════════════════════════════════════════════
void encoder_init() {
    pinMode(PIN_ENC_LEFT, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT), isr_left, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), isr_right, FALLING);

    raw_left = 0;
    raw_right = 0;
    signed_left = 0;
    signed_right = 0;
    prev_raw_left = 0;
    prev_raw_right = 0;

    Serial.printf("[ENC] Init: left=GPIO%d, right=GPIO%d (10 ticks/rev)\n",
                  PIN_ENC_LEFT, PIN_ENC_RIGHT);
}

// ═══════════════════════════════════════════════════════════════════════════════
int32_t encoder_get_left() {
    update_signed();
    return signed_left;
}

int32_t encoder_get_right() {
    update_signed();
    return signed_right;
}

void encoder_reset() {
    noInterrupts();
    raw_left = 0;
    raw_right = 0;
    interrupts();
    signed_left = 0;
    signed_right = 0;
    prev_raw_left = 0;
    prev_raw_right = 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
bool encoder_handle_serial(const char* cmd) {
    if (strcmp(cmd, "enc status") == 0) {
        update_signed();
        Serial.printf("[ENC] left=%d  right=%d  raw_L=%u raw_R=%u  dir=%d\n",
                      (int)signed_left, (int)signed_right,
                      (unsigned)raw_left, (unsigned)raw_right,
                      motor_get_direction());
        return true;
    }
    if (strcmp(cmd, "enc reset") == 0) {
        encoder_reset();
        Serial.println("[ENC] Reset to 0");
        return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
void encoder_print_help() {
    Serial.println("  enc status    Show tick counts (signed + raw)");
    Serial.println("  enc reset     Reset counters to zero");
}
