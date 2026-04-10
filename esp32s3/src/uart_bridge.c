/**
 * uart_bridge.c
 * Module: UART bridge — ESP32-S3 → Wemos D1 R32.
 * Board: ESP32-S3
 *
 * Frame format (19 bytes):
 *   [0xAA][0xBB][16 bytes: control_frame_t][1 byte: XOR checksum of the 16 data bytes]
 *
 * UART2, 115200 baud, 8N1.
 * TX = GPIO_NUM_17, RX = GPIO_NUM_16  ← adjust to match your schematic.
 */

#include "uart_bridge.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

/* ── Pin / peripheral configuration ─────────────────────────────────────── */
#define BRIDGE_UART_PORT   UART_NUM_2
#define BRIDGE_TX_PIN      GPIO_NUM_17
#define BRIDGE_RX_PIN      GPIO_NUM_16
#define BRIDGE_BAUD        115200
#define BRIDGE_RX_BUF      256

/* ── Frame constants ─────────────────────────────────────────────────────── */
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

bool uart_bridge_send_control(const control_frame_t *frame)
{
    if (!frame) return false;

    uint8_t buf[FRAME_TOTAL_LEN];
    buf[0] = SYNC_A;
    buf[1] = SYNC_B;
    memcpy(&buf[2], frame, FRAME_DATA_LEN);

    /* XOR checksum over the 16 data bytes */
    uint8_t ck = 0;
    for (int i = 0; i < FRAME_DATA_LEN; i++) ck ^= buf[2 + i];
    buf[2 + FRAME_DATA_LEN] = ck;

    int written = uart_write_bytes(BRIDGE_UART_PORT,
                                   (const char *)buf, FRAME_TOTAL_LEN);
    return written == FRAME_TOTAL_LEN;
}

bool uart_bridge_recv_odom(odom_t *out)
{
    /* Not needed for debug phase — implemented later */
    (void)out;
    return false;
}
