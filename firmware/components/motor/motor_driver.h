/*
 * motor_driver.h — DC Motor + Steering Servo Component
 *
 * Controls:
 *   - BTS7960 H-Bridge: forward/reverse speed (PWM)
 *   - MG90S Servo: steering angle (left/center/right)
 *
 * Safety:
 *   - Watchdog: stops motor if no command received for 500ms
 *   - Speed clamped to MAX_SPEED_MS
 *   - Steering clamped to STEER_MIN/MAX
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

void motor_init();
void motor_set_speed(float speed_normalized);
void motor_set_steering(int angle_deg);
void motor_update();
void motor_cmd_vel(float linear_x, float angular_z);
void motor_stop();
bool motor_handle_serial(const char* cmd);
void motor_print_help();

// Get current motor direction: +1=forward, -1=reverse, 0=stopped.
// IRAM_ATTR — safe to call from encoder ISR.
int8_t motor_get_direction();

// Get current steering setpoint in degrees.
int motor_get_steering();

// Called from Core 1 to write steering MCPWM (must run on same core as scanning servo)
void motor_update_steering_hw();

#endif // MOTOR_DRIVER_H
