/**
 * imu_encoder_driver.h
 * Module: AS5600 magnetic encoder + IMU yaw source for task_odometry.
 * Board: Wemos D1 R32
 *
 * The AS5600 (I2C addr 0x36) shares the I2C bus with the ICM-20948 IMU.
 * Call imu_gyro_init() once before using any function here.
 *
 * Calibration: set WHEEL_DIAMETER_M in imu_encoder_driver.c to the actual
 * outer wheel diameter, and flip ENCODER_SIGN to -1.0 if the encoder reads
 * negative when the car moves forward.
 */

#ifndef IMU_ENCODER_DRIVER_H
#define IMU_ENCODER_DRIVER_H

#include "../../types.h"

/* ── Legacy EKF API (stubs — not used in bridge-slave mode) ─────────────── */
void    imu_encoder_init(void);
odom_t  imu_encoder_read(void);

/* ── task_odometry API ───────────────────────────────────────────────────── */

/**
 * Read the AS5600 angle and accumulate cumulative distance.
 * Called at 100 Hz by task_odometry.  Requires imu_gyro_init() first.
 */
void imu_encoder_driver_update(void);

/**
 * Return the total forward distance traveled since boot, in metres.
 * Thread-safe for single-writer (task_odometry) / single-reader (bridge_slave_task).
 */
float imu_encoder_driver_get_distance_m(void);

/**
 * Return the current IMU heading in radians via imu_gyro_get_heading().
 * Used by encoder_ackermann_odom_update as the imu_yaw_rad argument.
 */
float imu_encoder_driver_get_yaw_rad(void);

#endif /* IMU_ENCODER_DRIVER_H */
