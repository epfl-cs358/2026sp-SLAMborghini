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
 * Return the latest heading (rad).
 * Updated by imu_gyro_update() at 100 Hz from task_odometry.
 * Safe to call from any task — volatile read, no I2C, no blocking.
 */
float imu_gyro_get_heading(void);

/**
 * Integrate gyro Z into the live heading over dt_s seconds.
 * Call once per task_odometry tick (every 10 ms) before reading the heading.
 */
void imu_gyro_update(float dt_s);

/**
 * Average raw gyro Z over `samples` readings (one per 10 ms) to set the
 * runtime bias.  Call once at startup before motors engage (~300 samples = 3 s).
 * Returns the calibrated bias in rad/s.
 */
float imu_gyro_calibrate_bias(int samples);

/**
 * Zero-velocity update: EMA-refine the runtime bias from the current raw
 * gyro reading.  Call only when the car is confirmed stationary (encoder
 * delta < 1 mm for ≥20 consecutive ticks).  Gain = 0.005 (~200 calls to
 * converge 50 %).
 */
void imu_gyro_zupt_update(void);

/**
 * Drive motors for drive_ms while integrating gyro Z.
 * Polls the stop-check callback every 10 ms — aborts early if it returns true.
 * Returns the actual heading (start_heading + integrated Δθ).
 */
float imu_drive_and_track(float start_heading_rad, uint32_t drive_ms);

#endif /* IMU_GYRO_H */
