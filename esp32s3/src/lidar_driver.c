/**
 * lidar_driver.c
 * Module: LiDAR driver for RPLiDAR C1 over UART.
 * Board: ESP32-S3
 * Implementation phase: stub (hardware not yet integrated)
 */

#include "lidar_driver.h"

void lidar_driver_init(void)
{
    // TODO: implement
    // Configure UART pins, baud rate (115200), and send RPLiDAR start scan command.
}

bool lidar_driver_read_scan(lidar_scan_t *out)
{
    // TODO: implement
    // Read UART bytes, parse RPLiDAR C1 response packets, fill out->points and out->count.
    (void)out;
    return false;
}

void lidar_driver_stop(void)
{
    // TODO: implement
    // Send RPLiDAR stop command, wait for motor spin-down, close UART.
}
