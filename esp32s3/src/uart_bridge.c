#include "uart_bridge.h"
#include "../../hardware_pins.h"

#include <string.h>

#ifdef ESP_PLATFORM
#include "driver/uart.h"
#include "driver/gpio.h"
#endif

/* BRIDGE_UART_PORT, BRIDGE_TX_PIN, BRIDGE_RX_PIN, BRIDGE_BAUD from hardware_pins.h */
#define BRIDGE_RX_BUF      512

#define SYNC_A             0xAAu
#define SYNC_B             0xBBu

#define MSG_CONTROL        0x01u
#define MSG_ODOM           0x02u
#define MSG_PATH           0x03u

#define HEADER_LEN         4u
#define MAX_PAYLOAD_LEN    256u

static uint8_t checksum_xor(const uint8_t *data, uint8_t len)
{
    uint8_t ck = 0;
    for (uint8_t i = 0; i < len; i++) {
        ck ^= data[i];
    }
    return ck;
}

void uart_bridge_init(void)
{
#ifdef ESP_PLATFORM
    uart_config_t cfg = {
        .baud_rate = BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_param_config(BRIDGE_UART_PORT, &cfg);
    uart_set_pin(BRIDGE_UART_PORT,
                 BRIDGE_TX_PIN,
                 BRIDGE_RX_PIN,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_driver_install(BRIDGE_UART_PORT, BRIDGE_RX_BUF, 512, 0, NULL, 0);
#endif
}

static bool send_packet(uint8_t msg_type, const void *payload, uint8_t payload_len)
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

    /* checksum covers: msg_type + payload_len + payload */
    buf[4 + payload_len] = checksum_xor(&buf[2], payload_len + 2);

    const int total_len = HEADER_LEN + payload_len + 1;
    int written = uart_write_bytes(BRIDGE_UART_PORT,
                                   (const char *)buf,
                                   total_len);

    return written == total_len;
#else
    (void)msg_type;
    (void)payload;
    (void)payload_len;
    return false;
#endif
}

bool uart_bridge_send_control(const control_frame_t *frame)
{
    if (!frame) {
        return false;
    }

    return send_packet(MSG_CONTROL,
                       frame,
                       (uint8_t)sizeof(control_frame_t));
}

bool uart_bridge_send_path(const path_frame_t *path_frame)
{
    if (!path_frame) {
        return false;
    }

    if (path_frame->length == 0 ||
        path_frame->length > MAX_SHARED_PATH_POINTS) {
        return false;
    }

    return send_packet(
        MSG_PATH,
        path_frame,
        (uint8_t)sizeof(path_frame_t)
    );
}

bool uart_bridge_recv_odom(odom_t *out)
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

        uart_read_bytes(BRIDGE_UART_PORT, &byte, 1, 0);
        if (byte != SYNC_A) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

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

        if (msg_type != MSG_ODOM) {
            return false;
        }

        if (payload_len != sizeof(odom_t) || payload_len > MAX_PAYLOAD_LEN) {
            return false;
        }

        uint8_t payload[MAX_PAYLOAD_LEN];
        uint8_t received_ck = 0;

        if (uart_read_bytes(BRIDGE_UART_PORT, payload, payload_len, 0) != payload_len) {
            return false;
        }

        if (uart_read_bytes(BRIDGE_UART_PORT, &received_ck, 1, 0) != 1) {
            return false;
        }

        uint8_t check_buf[2 + MAX_PAYLOAD_LEN];
        check_buf[0] = msg_type;
        check_buf[1] = payload_len;
        memcpy(&check_buf[2], payload, payload_len);

        uint8_t computed_ck = checksum_xor(check_buf, payload_len + 2);

        if (computed_ck != received_ck) {
            return false;
        }

        memcpy(out, payload, sizeof(odom_t));
        return true;
    }
#endif

    return false;
}