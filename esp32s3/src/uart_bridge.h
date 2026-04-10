/**
 * uart_bridge.h
 * Module: UART bridge — communication between ESP32-S3 and Wemos D1 R32.
 * Board: ESP32-S3
 * Serializes and transmits control_frame_t to the Wemos, and deserializes
 * incoming odom_t frames from the Wemos over a shared UART bus.
 */

#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdbool.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/uart_stub.h"
#endif

/**
 * Configure the UART peripheral: pins, baud rate, and frame format.
 * Must be called once during initialization before any send/recv calls.
 */
void uart_bridge_init(void);

/**
 * Serialize and transmit a control_frame_t to the Wemos D1 R32.
 * @param frame Pointer to the control frame to send.
 * @return true if the frame was sent successfully, false on UART error.
 */
bool uart_bridge_send_control(const control_frame_t *frame);

/**
 * Receive and deserialize an odometry frame from the Wemos D1 R32.
 * Non-blocking: returns false immediately if no new data is available.
 * @param out Pointer to an odom_t struct to populate on success.
 * @return true if a complete odom frame was received, false otherwise.
 */
bool uart_bridge_recv_odom(odom_t *out);

#endif /* UART_BRIDGE_H */
