/*
 * servo_driver.h — Servo Component Public Interface
 *
 * This is the ONLY file other parts of the code include.
 * It exposes 3 functions — that's it:
 *   servo_init()   → call once in setup()
 *   servo_update() → call every loop() iteration
 *   servo_handle_serial() → call when serial data arrives
 *
 * ADDING A NEW COMPONENT? Follow this same pattern:
 *   components/motor/motor_driver.h
 *   components/imu/imu_driver.h
 * Each exposes init(), update(), handle_serial().
 */

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

// Initialize the servo hardware. Call once in setup().
void servo_init();

// Update servo position based on current mode. Call every loop().
// This is non-blocking — it only acts when SERVO_UPDATE_MS has elapsed.
void servo_update();

// Process a serial command for this component.
// Returns true if the command was handled, false if not recognized.
bool servo_handle_serial(const char* cmd);

// Print available commands to Serial.
void servo_print_help();

// Get current servo position in degrees.
float servo_get_position();

#endif // SERVO_DRIVER_H
