/**
 * uart_bridge.c
 * ------------------------------------------------------------
 * Module: UART bridge between Wemos D1 R32 and ESP32-S3
 * Board : Wemos D1 R32
 *
 * Purpose:
 *   - Receive control commands from ESP32-S3
 *   - Send odometry feedback back to ESP32-S3
 *
 * Transport:
 *   UART1 @ 115200 baud, 8N1
 *
 * Wiring:
 *   Wemos RX GPIO4  <- ESP32-S3 TX GPIO17
 *   Wemos TX GPIO5  -> ESP32-S3 RX GPIO16
 *   GND shared between boards
 *
 * Packet format:
 *
 *   Byte 0 : 0xAA           Sync byte A
 *   Byte 1 : 0xBB           Sync byte B
 *   Byte 2 : Message Type
 *   Byte 3 : Payload Length
 *   Byte 4..N : Payload
 *   Last byte : XOR checksum of:
 *               [msg_type][payload_len][payload bytes]
 *
 * Message types:
 *   0x01 = control_frame_t
 *   0x02 = odom_t
 *
 * Notes:
 *   - Non-blocking receive
 *   - Checksum used for corruption detection
 *   - Struct payload copied directly as binary
 * ------------------------------------------------------------
 */

#include "uart_bridge.h"

#include <string.h>

#ifdef ESP_PLATFORM
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#endif

/*
 * Wiring:
 *   Wemos RX GPIO16 <- ESP32-S3 TX GPIO17
 *   Wemos TX GPIO17 -> ESP32-S3 RX GPIO16
 *   GND shared between boards
 */

#define BRIDGE_UART_PORT   UART_NUM_1
#define BRIDGE_TX_PIN GPIO_NUM_17   // TX → S3 RX (GPIO16)
#define BRIDGE_RX_PIN GPIO_NUM_16   // RX ← S3 TX (GPIO17)
#define BRIDGE_UART_BAUD   115200
#define BRIDGE_RX_BUF      512

/* ------------------------------------------------------------
 * Protocol constants
 * ------------------------------------------------------------ */
#define SYNC_A             0xAAu
#define SYNC_B             0xBBu

#define MSG_CONTROL        0x01u
#define MSG_ODOM           0x02u
#define MSG_PATH           0x03u

#define HEADER_LEN         4u
#define MAX_PAYLOAD_LEN    256u

/* ------------------------------------------------------------
 * Compute XOR checksum over byte array
 * ------------------------------------------------------------ */
static uint8_t checksum_xor(const uint8_t *data, uint8_t len)
{
    uint8_t ck = 0;

    for (uint8_t i = 0; i < len; i++) {
        ck ^= data[i];
    }

    return ck;
}

/* ------------------------------------------------------------
 * Initialize UART peripheral
 * Must be called once during startup
 * ------------------------------------------------------------ */
void uart_bridge_init(void)
{
#ifdef ESP_PLATFORM
    uart_config_t cfg = {
        .baud_rate = BRIDGE_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_delete(BRIDGE_UART_PORT);

    uart_param_config(BRIDGE_UART_PORT, &cfg);

    uart_set_pin(
        BRIDGE_UART_PORT,
        BRIDGE_TX_PIN,
        BRIDGE_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    uart_driver_install(BRIDGE_UART_PORT, BRIDGE_RX_BUF, BRIDGE_RX_BUF, 0, NULL, 0);

    uart_flush_input(BRIDGE_UART_PORT);
    printf("[UART DEBUG] init: UART%d TX=%d RX=%d baud=%d\n",
       BRIDGE_UART_PORT,
       BRIDGE_TX_PIN,
       BRIDGE_RX_PIN,
       BRIDGE_UART_BAUD);
#endif
}

/* ------------------------------------------------------------
 * Internal generic packet sender
 * ------------------------------------------------------------ */
static bool send_packet(uint8_t msg_type,
                        const void *payload,
                        uint8_t payload_len)
{
    if (!payload || payload_len > MAX_PAYLOAD_LEN) {
        return false;
    }

#ifdef ESP_PLATFORM
    uint8_t buf[HEADER_LEN + MAX_PAYLOAD_LEN + 1];

    buf[0] = SYNC_A;
    buf[1] = SYNC_B;
    buf[2] = msg_type;
    buf[3] = payload_len;

    memcpy(&buf[4], payload, payload_len);

    /* checksum covers type + length + payload */
    buf[4 + payload_len] =
        checksum_xor(&buf[2], payload_len + 2);

    const int total_len = HEADER_LEN + payload_len + 1;

    int written = uart_write_bytes(
        BRIDGE_UART_PORT,
        (const char *)buf,
        total_len
    );

    return written == total_len;
#else
    (void)msg_type;
    (void)payload;
    (void)payload_len;
    return false;
#endif
}

/* ------------------------------------------------------------
 * Send odometry packet to ESP32-S3
 * ------------------------------------------------------------ */
bool uart_bridge_send_odom(const odom_t *odom)
{
    if (!odom) {
        return false;
    }

    return send_packet(
        MSG_ODOM,
        odom,
        (uint8_t)sizeof(odom_t)
    );
}

/* ------------------------------------------------------------
 * Receive control packet from ESP32-S3
 *
 * Returns:
 *   true  = new valid packet received
 *   false = no packet / bad checksum / incomplete frame
 * ------------------------------------------------------------ */
bool uart_bridge_recv_control(control_frame_t *out)
{
    if (!out) {
        return false;
    }


#ifdef ESP_PLATFORM
    size_t available = 0;
    uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);

    if (available < HEADER_LEN + 1) {
        return false;
    }

    while (available >= HEADER_LEN + 1) {
        uint8_t byte = 0;

        /* Search sync byte A */
        uart_read_bytes(BRIDGE_UART_PORT, &byte, 1, 0);

        if (byte != SYNC_A) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        /* Search sync byte B */
        uart_read_bytes(BRIDGE_UART_PORT, &byte, 1, 0);

        if (byte != SYNC_B) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        uint8_t msg_type = 0;
        uint8_t payload_len = 0;

        if (uart_read_bytes(BRIDGE_UART_PORT, &msg_type, 1, 0) != 1) {
            return false;
        }

        if (uart_read_bytes(BRIDGE_UART_PORT, &payload_len, 1, 0) != 1) {
            return false;
        }

        /* Must be control packet */
        if (msg_type != MSG_CONTROL) {
            return false;
        }

        if (payload_len != sizeof(control_frame_t) ||
            payload_len > MAX_PAYLOAD_LEN) {
            return false;
        }

        uint8_t payload[MAX_PAYLOAD_LEN];
        uint8_t received_ck = 0;

                int got_payload = uart_read_bytes(BRIDGE_UART_PORT,
                                          payload,
                                          payload_len,
                                          pdMS_TO_TICKS(100));

        if (got_payload != payload_len) {
            printf("[UART DEBUG] payload read failed: got=%d expected=%u\n",
                   got_payload,
                   (unsigned)payload_len);
            return false;
        }

        int got_ck = uart_read_bytes(BRIDGE_UART_PORT,
                                     &received_ck,
                                     1,
                                     pdMS_TO_TICKS(20));

        if (got_ck != 1) {
            printf("[UART DEBUG] checksum read failed: got=%d\n", got_ck);
            return false;
        }

        /* Recompute checksum */
        uint8_t check_buf[2 + MAX_PAYLOAD_LEN];

        check_buf[0] = msg_type;
        check_buf[1] = payload_len;

        memcpy(&check_buf[2], payload, payload_len);

        uint8_t computed_ck =
            checksum_xor(check_buf, payload_len + 2);

        if (computed_ck != received_ck) {
            return false;
        }

        memcpy(out, payload, sizeof(control_frame_t));
        return true;
    }
#endif

    return false;
}

bool uart_bridge_recv_path(path_frame_t *out)
{
    if (!out) {
        return false;
    }

#ifdef ESP_PLATFORM
    size_t available = 0;
    uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);

    if (available < HEADER_LEN + 1) {
        return false;
    }

    while (available >= HEADER_LEN + 1) {
        uint8_t byte = 0;

        /* Search sync byte A */
        uart_read_bytes(BRIDGE_UART_PORT, &byte, 1, 0);

        if (byte != SYNC_A) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        /* Search sync byte B */
        uart_read_bytes(BRIDGE_UART_PORT, &byte, 1, 0);

        if (byte != SYNC_B) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        uint8_t msg_type = 0;
        uint8_t payload_len = 0;

        if (uart_read_bytes(BRIDGE_UART_PORT, &msg_type, 1, 0) != 1) {
            return false;
        }

        if (uart_read_bytes(BRIDGE_UART_PORT, &payload_len, 1, 0) != 1) {
            return false;
        }

        printf("[UART DEBUG] rx msg_type=0x%02X payload_len=%u\n",
               (unsigned)msg_type,
               (unsigned)payload_len);

        /* Must be path packet */
        if (msg_type != MSG_PATH) {
            printf("[UART DEBUG] not a path packet\n");
            return false;
        }

                printf("[UART DEBUG] sizeof(path_frame_t)=%u\n",
               (unsigned)sizeof(path_frame_t));

        if (payload_len != sizeof(path_frame_t) ||
            payload_len > MAX_PAYLOAD_LEN) {
            printf("[UART DEBUG] bad path size: payload_len=%u expected=%u\n",
                   (unsigned)payload_len,
                   (unsigned)sizeof(path_frame_t));
            return false;
        }

        uint8_t payload[MAX_PAYLOAD_LEN];
        uint8_t received_ck = 0;

                int got_payload = uart_read_bytes(BRIDGE_UART_PORT,
                                          payload,
                                          payload_len,
                                          pdMS_TO_TICKS(100));

        if (got_payload != payload_len) {
            printf("[UART DEBUG] payload read failed: got=%d expected=%u\n",
                   got_payload,
                   (unsigned)payload_len);
            return false;
        }

                int got_ck = uart_read_bytes(BRIDGE_UART_PORT,
                                     &received_ck,
                                     1,
                                     pdMS_TO_TICKS(20));

        if (got_ck != 1) {
            printf("[UART DEBUG] checksum read failed: got=%d\n", got_ck);
            return false;
        }

        uint8_t check_buf[2 + MAX_PAYLOAD_LEN];

        check_buf[0] = msg_type;
        check_buf[1] = payload_len;

        memcpy(&check_buf[2], payload, payload_len);

        uint8_t computed_ck =
            checksum_xor(check_buf, payload_len + 2);

        if (computed_ck != received_ck) {
            printf("[UART DEBUG] checksum mismatch: computed=0x%02X received=0x%02X\n",
                   (unsigned)computed_ck,
                   (unsigned)received_ck);
            return false;
        }

        memcpy(out, payload, sizeof(path_frame_t));

        printf("[UART DEBUG] path payload decoded: length=%u\n",
               (unsigned)out->length);

        if (out->length == 0 ||
            out->length > MAX_SHARED_PATH_POINTS) {
            printf("[UART DEBUG] invalid path length: %u\n",
                   (unsigned)out->length);
            return false;
        }

        return true;
    }
#endif

    return false;
}