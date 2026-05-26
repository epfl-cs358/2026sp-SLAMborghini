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

/* Wire protocol version: bump when odom_wire_t layout changes so both boards
 * fail loudly (size mismatch) rather than silently decode garbage. v2: added
 * consumed_wp_idx + consumed_path_id (6 → 10 bytes). */
#define MSG_CONTROL        0x01u
#define MSG_ODOM           0x02u
#define MSG_PATH_DONE      0x04u
#define MSG_PATH_CHUNK     0x06u  /* ESP32-S3 → Wemos: streaming chunk       */
#define MSG_CHUNK_NACK     0x07u  /* Wemos → ESP32-S3: out-of-order signal   */

#define HEADER_LEN         4u
#define MAX_PAYLOAD_LEN    255u

static uint8_t checksum_xor(const uint8_t *data, size_t len)
{
    uint8_t ck = 0;
    for (size_t i = 0; i < len; i++) {
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

static bool send_packet(uint8_t msg_type, const void *payload, size_t payload_len)
{
    if (!payload || payload_len > MAX_PAYLOAD_LEN) {
        return false;
    }

#ifdef ESP_PLATFORM
    uint8_t buf[HEADER_LEN + MAX_PAYLOAD_LEN + 1];

    buf[0] = SYNC_A;
    buf[1] = SYNC_B;
    buf[2] = msg_type;
    buf[3] = (uint8_t)payload_len;

    memcpy(&buf[4], payload, payload_len);

    /* checksum covers: msg_type + payload_len + payload */
    buf[4 + payload_len] = checksum_xor(&buf[2], payload_len + 2);

    const size_t total_len = HEADER_LEN + payload_len + 1u;
    int written = uart_write_bytes(BRIDGE_UART_PORT,
                                   (const char *)buf,
                                   total_len);

    return written == (int)total_len;
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
    return send_packet(MSG_CONTROL, frame, (uint8_t)sizeof(control_frame_t));
}

bool uart_bridge_send_path_chunk(const path_chunk_t *chunk)
{
    if (!chunk || chunk->count == 0 || chunk->count > PATH_CHUNK_WP_COUNT) {
        return false;
    }
    return send_packet(MSG_PATH_CHUNK, chunk, (uint8_t)sizeof(path_chunk_t));
}

/* ── Compact odom wire format v2 — must match wemos/src/uart_bridge.c exactly ─
 * v2 adds consumed_wp_idx and consumed_path_id so the streamer can advance.    */
typedef struct __attribute__((packed)) {
    int16_t  disp_x16;          /* linear_disp_mm × 16;  0.0625 mm/lsb  */
    int16_t  yaw_mrad_s;        /* yaw_rate_imu × 1000;  1 mrad/s/lsb   */
    uint8_t  dt_ms;             /* integration window in ms              */
    uint8_t  seq;               /* low 8 bits of sequence counter        */
    uint16_t consumed_wp_idx;   /* global path index last consumed by PP */
    uint16_t consumed_path_id;  /* path_id that index belongs to         */
} odom_wire_t;  /* 10 bytes */
_Static_assert(sizeof(odom_wire_t) == 10u,
               "odom_wire_t size mismatch — reflash both boards together");

/* Flags set by the receive loop; cleared by the respective query functions. */
static bool     s_path_done       = false;
static bool     s_nack_pending    = false;
static uint16_t s_nack_path_id    = 0;
static uint16_t s_nack_exp_start  = 0;

bool uart_bridge_recv_odom(odom_t *out)
{
    if (!out) {
        return false;
    }

#ifdef ESP_PLATFORM
    size_t available = 0;
    uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);

    while (available >= HEADER_LEN + 1) {
        uint8_t byte = 0;

        uart_read_bytes(BRIDGE_UART_PORT, &byte, 1, 0);
        if (byte != SYNC_A) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        if (uart_read_bytes(BRIDGE_UART_PORT, &byte, 1, pdMS_TO_TICKS(5)) != 1) break;
        if (byte != SYNC_B) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        uint8_t msg_type    = 0;
        uint8_t payload_len = 0;

        if (uart_read_bytes(BRIDGE_UART_PORT, &msg_type,    1, pdMS_TO_TICKS(5)) != 1) break;
        if (uart_read_bytes(BRIDGE_UART_PORT, &payload_len, 1, pdMS_TO_TICKS(5)) != 1) break;

        if (msg_type != MSG_ODOM &&
            msg_type != MSG_PATH_DONE &&
            msg_type != MSG_CHUNK_NACK) {
            uint8_t skip[MAX_PAYLOAD_LEN + 1u];
            size_t skip_len = (size_t)payload_len + 1u;
            if (skip_len > 0u)
                uart_read_bytes(BRIDGE_UART_PORT, skip, skip_len, pdMS_TO_TICKS(50));
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        uint8_t payload[MAX_PAYLOAD_LEN];
        uint8_t received_ck = 0;

        if (uart_read_bytes(BRIDGE_UART_PORT, payload,      payload_len, pdMS_TO_TICKS(50)) != (int)payload_len) break;
        if (uart_read_bytes(BRIDGE_UART_PORT, &received_ck, 1,           pdMS_TO_TICKS(10)) != 1) break;

        uint8_t check_buf[2u + MAX_PAYLOAD_LEN];
        check_buf[0] = msg_type;
        check_buf[1] = payload_len;
        memcpy(&check_buf[2], payload, payload_len);

        if (checksum_xor(check_buf, (size_t)payload_len + 2u) != received_ck) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        if (msg_type == MSG_PATH_DONE) {
            s_path_done = true;
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        if (msg_type == MSG_CHUNK_NACK) {
            /* 4-byte payload: path_id (2) + expected_start (2) */
            if (payload_len == 4u) {
                uint16_t pid, exp;
                memcpy(&pid, &payload[0], 2);
                memcpy(&exp, &payload[2], 2);
                s_nack_pending   = true;
                s_nack_path_id   = pid;
                s_nack_exp_start = exp;
            }
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        /* MSG_ODOM */
        if (payload_len != sizeof(odom_wire_t)) {
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &available);
            continue;
        }

        odom_wire_t w;
        memcpy(&w, payload, sizeof(odom_wire_t));
        out->linear_disp_mm  = (float)w.disp_x16   / 16.0f;
        out->yaw_rate_imu    = (float)w.yaw_mrad_s / 1000.0f;
        out->dt_ms           = (float)w.dt_ms;
        out->seq             = w.seq;
        out->consumed_wp_idx  = w.consumed_wp_idx;
        out->consumed_path_id = w.consumed_path_id;
        return true;
    }
#endif

    return false;
}

bool uart_bridge_recv_path_done(void)
{
    if (s_path_done) {
        s_path_done = false;
        return true;
    }
    return false;
}

bool uart_bridge_recv_chunk_nack(uint16_t *out_path_id, uint16_t *out_expected_start)
{
    if (s_nack_pending) {
        if (out_path_id)       *out_path_id       = s_nack_path_id;
        if (out_expected_start) *out_expected_start = s_nack_exp_start;
        s_nack_pending = false;
        return true;
    }
    return false;
}
