/*
 * tof_driver.h — ToF Sensor Array Public Interface
 *
 * Same pattern as servo_driver.h:
 *   tof_init()           → call once in setup()
 *   tof_update()         → call every loop() (non-blocking)
 *   tof_handle_serial()  → parse commands for this component
 *   tof_print_help()     → show available commands
 *   tof_get_json()       → fill a buffer with the JSON data string
 */

#ifndef TOF_DRIVER_H
#define TOF_DRIVER_H

#include <stdint.h>

// Initialize I2C, XSHUT sequencing, and assign addresses.
// Returns true if all sensors initialized successfully.
bool tof_init();

// Start continuous ranging on all initialized sensors and switch to the
// tight runtime I2C timing. Call after IMU init to avoid bus-load issues.
void tof_start();

// Read sensors if data is ready. Non-blocking.
void tof_update();

// Process serial command. Returns true if handled.
bool tof_handle_serial(const char* cmd);

// Print commands for this component.
void tof_print_help();

// Check if new JSON data is ready to publish (based on TOF_PUBLISH_MS).
bool tof_json_ready();

// Write JSON string into provided buffer. Returns number of chars written.
// Buffer must be at least 256 bytes.
int tof_get_json(char* buf, int buf_size, float servo_deg);

// Get distance reading for a specific sensor (millimeters). Returns 0 if not initialized.
uint16_t tof_get_distance(uint8_t sensor_id);

// Get range status for a specific sensor. 0=valid, other=error.
uint8_t tof_get_status(uint8_t sensor_id);

#endif // TOF_DRIVER_H
