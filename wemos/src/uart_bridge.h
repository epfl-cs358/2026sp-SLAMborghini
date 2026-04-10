/**
 * uart_bridge.h
 * Module: UART bridge — communication with ESP32-S3.
 * Board: Wemos D1 R32
 * Serializes and sends odom_t frames to the ESP32-S3 SLAM brain, and
 * deserializes incoming control_frame_t commands from the ESP32-S3.
 */

#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdbool.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/uart_stub.h"
#endif

/**
 * Configure the UART peripheral on the Wemos D1 R32 for communication with the ESP32-S3.
 * Sets baud rate, pin assignments, and installs the UART driver.
 * Must be called once during initialization.
 */
void uart_bridge_init(void);

/**
 * Serialize and transmit an odom_t frame to the ESP32-S3 over UART.
 * @param odom Pointer to the odometry measurement to send.
 * @return true if the frame was sent successfully, false on UART error.
 */
bool uart_bridge_send_odom(const odom_t *odom);

/**
 * Receive and deserialize a control_frame_t command from the ESP32-S3.
 * Non-blocking: returns false immediately if no new data is available.
 * @param out Pointer to a control_frame_t struct to populate on success.
 * @return true if a complete control frame was received, false otherwise.
 */
bool uart_bridge_recv_control(control_frame_t *out);

#endif /* UART_BRIDGE_H */
