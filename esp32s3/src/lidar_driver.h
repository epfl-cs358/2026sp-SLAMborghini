/**
 * lidar_driver.h
 * Module: LiDAR driver for RPLiDAR C1 over UART.
 * Board: ESP32-S3
 * Handles UART initialization, scan acquisition, and motor control for the RPLiDAR C1.
 */

#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <stdbool.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/lidar_stub.h"
#endif

/**
 * Initialize the UART peripheral and start the RPLiDAR C1 motor and scan.
 * Must be called once before any calls to lidar_driver_read_scan().
 */
void lidar_driver_init(void);

/**
 * Blocking read of one full 360-degree scan from the RPLiDAR C1.
 * @param out Pointer to a lidar_scan_t struct to populate with scan data.
 * @return true if a complete scan was successfully read, false on error or timeout.
 */
bool lidar_driver_read_scan(lidar_scan_t *out);

/**
 * Stop the RPLiDAR C1 motor and close the UART peripheral.
 * Should be called on shutdown or before entering deep sleep.
 */
void lidar_driver_stop(void);

#endif /* LIDAR_DRIVER_H */
