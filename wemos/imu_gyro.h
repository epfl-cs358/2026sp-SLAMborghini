/**
 * imu_gyro.h
 * Minimal ICM-20948 gyro-Z driver for ESP-IDF.
 * Pins/address/bias copied from 2025fa-SLAMurai icm_control/config.hpp.
 */
#ifndef IMU_GYRO_H
#define IMU_GYRO_H

#include <stdbool.h>
#include <stdint.h>

/* Returns true if the ICM-20948 was found and configured. */
bool  imu_gyro_init(void);

/* Read gyro Z in rad/s (bias-corrected). Returns 0 on I2C error. */
float imu_gyro_read_z(void);

/**
 * Register a callback polled every 10 ms inside imu_drive_and_track.
 * If the callback returns true the drive is aborted immediately.
 * Pass NULL to disable.  Call once at startup, e.g.:
 *   imu_gyro_set_stop_check(wifi_dashboard_stop_requested);
 */
void imu_gyro_set_stop_check(bool (*fn)(void));

/**
 * Return the latest heading (rad) published by imu_drive_and_track().
 * Updated every POLL_MS (10 ms) during a drive; holds last value between drives.
 * Safe to call from any task — volatile read, no I2C, no blocking.
 * Returns 0.0 before the first drive completes.
 */
float imu_gyro_get_heading(void);

/**
 * Drive motors for drive_ms while integrating gyro Z.
 * Polls the stop-check callback every 10 ms — aborts early if it returns true.
 * Returns the actual heading (start_heading + integrated Δθ).
 */
float imu_drive_and_track(float start_heading_rad, uint32_t drive_ms);

#endif /* IMU_GYRO_H */
