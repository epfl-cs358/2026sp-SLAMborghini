#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>
#include "../../types.h"

/**
 * Initialize UART bridge between Wemos and ESP32-S3.
 */
void uart_bridge_init(void);

/**
 * Send odometry packet to ESP32-S3.
 */
bool uart_bridge_send_odom(const odom_t *odom);

/**
 * Receive latest control command from ESP32-S3.
 *
 * Old protocol: one control_frame_t.
 */
bool uart_bridge_recv_control(control_frame_t *out);

/**
 * Receive latest path frame from ESP32-S3.
 *
 * New protocol: short path containing multiple waypoints.
 */
bool uart_bridge_recv_path(path_frame_t *out);

/* Notify ESP32-S3 that the current path has been fully executed. */
bool uart_bridge_send_path_done(void);

#endif /* UART_BRIDGE_H */