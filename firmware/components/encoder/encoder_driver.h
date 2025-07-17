/*
 * encoder_driver.h — Hall Encoder Component
 *
 * Two hall sensors on rear wheels (10 magnets per wheel).
 * Uses hardware interrupts to count ticks — never misses a pulse.
 *
 * Publishes: /encoder/left and /encoder/right (std_msgs/Int32)
 * Tick count is cumulative (forward = positive, can be reset via serial).
 */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

// Initialize encoder interrupts. Call once in setup().
void encoder_init();

// Get current tick counts.
int32_t encoder_get_left();
int32_t encoder_get_right();

// Reset tick counts to zero.
void encoder_reset();

// Process serial commands. Returns true if handled.
bool encoder_handle_serial(const char* cmd);

// Print help.
void encoder_print_help();

#endif // ENCODER_DRIVER_H
