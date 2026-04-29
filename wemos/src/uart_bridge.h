#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>
#include "../../types.h"

/**
 * Initialize UART bridge between Wemos and ESP32-S3.
 * Configures UART port, baud rate, pins, and RX buffer.
 */
void uart_bridge_init(void);

/**
 * Send odometry packet to ESP32-S3.
 *
 * @param odom Pointer to odometry structure.
 * @return true if packet sent successfully.
 */
bool uart_bridge_send_odom(const odom_t *odom);

/**
 * Receive latest control command from ESP32-S3.
 *
 * Non-blocking:
 * returns false if no full valid packet available.
 *
 * @param out Pointer to destination control frame.
 * @return true if valid control packet received.
 */
bool uart_bridge_recv_control(control_frame_t *out);

#endif /* UART_BRIDGE_H */