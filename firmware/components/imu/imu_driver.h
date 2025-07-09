/*
 * imu_driver.h — MPU6050 IMU Component
 *
 * Reads 6-axis data (accelerometer + gyroscope) from MPU6050
 * on a dedicated I2C bus (Wire1, GPIO 7 SDA, GPIO 17 SCL).
 *
 * Publishes via ros_bridge on /imu/data (sensor_msgs/Imu)
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

// IMU data structure (raw readings converted to SI units)
struct ImuData {
    float accel_x;   // m/s² (acceleration)
    float accel_y;
    float accel_z;
    float gyro_x;    // rad/s (angular velocity)
    float gyro_y;
    float gyro_z;
};

// Initialize MPU6050. Call after Wire.begin() (tof_init does this).
bool imu_init();

// Read latest IMU data. Non-blocking (takes ~1ms for I2C read).
void imu_update();

// Get latest readings.
ImuData imu_get_data();

// Process serial commands. Returns true if handled.
bool imu_handle_serial(const char* cmd);

// Print help.
void imu_print_help();

#endif // IMU_DRIVER_H
