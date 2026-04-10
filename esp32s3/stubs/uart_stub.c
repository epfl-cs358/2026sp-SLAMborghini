// STUB — simulates uart_bridge for offline testing

/**
 * uart_stub.c
 * Module: UART stub — simulates UART bridge communication for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by integration tests.
 */

#include "uart_stub.h"

/*
 * Scenario: UART send always succeeds (no hardware errors, no buffer overflow).
 * Used to isolate test logic from real UART availability.
 * Used by: integration tests
 */
bool uart_stub_send_ok(void)
{
    return true;
}

/*
 * Scenario: Wemos reports 50 mm straight-line motion over 100 ms with tiny yaw drift.
 * Represents a robot driving forward in a corridor at approximately 500 mm/s.
 * Used by: integration tests
 */
odom_t uart_stub_recv_odom(void)
{
    odom_t odom;
    odom.linear_disp_mm = 50.0f;
    odom.yaw_rate_imu   = 0.02f;
    odom.dt_ms          = 100.0f;
    return odom;
}
