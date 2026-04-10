/**
 * uart_bridge.c
 * Module: UART bridge — Wemos D1 R32 ← ESP32-S3.
 * Board: Wemos D1 R32
 *
 * Receives the same 19-byte frame sent by the ESP32-S3:
 *   [0xAA][0xBB][16 bytes: control_frame_t][1 byte: XOR checksum]
 *
 * UART1, 115200 baud, 8N1.
 * RX = GPIO_NUM_4, TX = GPIO_NUM_5  ← adjust to match your schematic.
 * Connect: ESP32-S3 TX (GPIO17) → Wemos RX (GPIO4).
 */

#include "uart_bridge.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

/* ── Pin / peripheral configuration ─────────────────────────────────────── */
#define BRIDGE_UART_PORT   UART_NUM_1
#define BRIDGE_RX_PIN      GPIO_NUM_4    /* connected to ESP32-S3 TX (GPIO17) */
#define BRIDGE_TX_PIN      GPIO_NUM_5    /* connected to ESP32-S3 RX (GPIO16) */
#define BRIDGE_BAUD        115200
#define BRIDGE_RX_BUF      256

/* ── Frame constants (must match esp32s3/uart_bridge.c) ─────────────────── */
#define SYNC_A             0xAAu
#define SYNC_B             0xBBu
#define FRAME_DATA_LEN     ((int)sizeof(control_frame_t))   /* 16 bytes */
#define FRAME_TOTAL_LEN    (2 + FRAME_DATA_LEN + 1)         /* 19 bytes */

void uart_bridge_init(void)
{
    uart_config_t cfg = {
        .baud_rate           = BRIDGE_BAUD,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .source_clk          = UART_SCLK_DEFAULT,
    };
    uart_param_config(BRIDGE_UART_PORT, &cfg);
    uart_set_pin(BRIDGE_UART_PORT,
                 BRIDGE_TX_PIN, BRIDGE_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(BRIDGE_UART_PORT, BRIDGE_RX_BUF, 0, 0, NULL, 0);
}

bool uart_bridge_recv_control(control_frame_t *out)
{
    if (!out) return false;

    /* Peek: is there a full frame in the buffer? */
    size_t available = 0;
    uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
    if (available < (size_t)FRAME_TOTAL_LEN) return false;

    uint8_t buf[FRAME_TOTAL_LEN];

    /* Scan for sync header — discard stale bytes before it */
    while (1) {
        size_t avail2 = 0;
        uart_get_buffered_data_len(BRIDGE_UART_PORT, &avail2);
        if (avail2 < (size_t)FRAME_TOTAL_LEN) return false;

        /* Peek at first byte */
        uint8_t b0;
        uart_read_bytes(BRIDGE_UART_PORT, &b0, 1, 0);
        if (b0 != SYNC_A) continue;   /* not sync A — discard and retry */

        uint8_t b1;
        uart_read_bytes(BRIDGE_UART_PORT, &b1, 1, 0);
        if (b1 != SYNC_B) continue;   /* not sync B — discard and retry */

        /* Read the data + checksum */
        int got = uart_read_bytes(BRIDGE_UART_PORT,
                                  buf, FRAME_DATA_LEN + 1, 0);
        if (got != FRAME_DATA_LEN + 1) return false;

        /* Verify checksum */
        uint8_t ck = 0;
        for (int i = 0; i < FRAME_DATA_LEN; i++) ck ^= buf[i];
        if (ck != buf[FRAME_DATA_LEN]) return false;   /* bad checksum */

        memcpy(out, buf, FRAME_DATA_LEN);
        return true;
    }
}

bool uart_bridge_send_odom(const odom_t *odom)
{
    /* Not needed for debug phase — implemented later */
    (void)odom;
    return false;
}
