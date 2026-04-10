/**
 * uart_stub.h
 * Module: UART stub — simulates UART bridge communication for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by integration tests.
 */

// STUB — simulates uart_bridge for offline testing

#ifndef UART_STUB_H
#define UART_STUB_H

#include <stdbool.h>
#include "../../types.h"

/**
 * Simulates a successful UART send: always returns true.
 * Used to test code paths that depend on uart_bridge_send_control() succeeding.
 * Used by: integration tests
 */
bool uart_stub_send_ok(void);

/**
 * Returns a realistic odom_t as if the Wemos had sent 50 mm forward motion,
 * 0.02 rad/s yaw rate, over a 100 ms window.
 * Simulates normal straight-line driving odometry feedback.
 * Used by: integration tests
 */
odom_t uart_stub_recv_odom(void);

#endif /* UART_STUB_H */
