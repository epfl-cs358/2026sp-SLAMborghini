/**
 * imu_encoder_driver.h
 * Module: IMU and wheel encoder driver.
 * Board: Wemos D1 R32
 * Reads angular velocity and linear acceleration from an MPU6050-compatible IMU
 * over I2C, and counts wheel encoder pulses via GPIO interrupts. Combines these
 * into an odom_t for the EKF fusion module.
 */

#ifndef IMU_ENCODER_DRIVER_H
#define IMU_ENCODER_DRIVER_H

#include "../../types.h"

/**
 * Initialize the I2C peripheral for the IMU and configure GPIO pins for
 * the left and right wheel encoder interrupts.
 * Must be called once before imu_encoder_read().
 */
void imu_encoder_init(void);

/**
 * Read the current IMU and encoder state and return a fused odometry measurement.
 * Integrates encoder ticks since the last call to compute linear displacement.
 * @return odom_t with linear_disp_mm, yaw_rate_imu (rad/s), and dt_ms filled in.
 */
odom_t imu_encoder_read(void);

#endif /* IMU_ENCODER_DRIVER_H */
