/**
 * uart_bridge.c  —  Wemos D1 R32
 *
 * Streaming path protocol (v2):
 *   ESP32-S3 → Wemos : MSG_PATH_CHUNK (0x06) — 8-waypoint chunks
 *   Wemos → ESP32-S3 : MSG_ODOM (0x02)       — 10-byte odom_wire_t v2
 *                      MSG_PATH_DONE (0x04)   — path complete
 *                      MSG_CHUNK_NACK (0x07)  — out-of-order chunk
 */

#include "uart_bridge.h"
#include "../../hardware_pins.h"

#include <string.h>

#ifdef ESP_PLATFORM
#include "driver/uart.h"
#include "driver/gpio.h"
#endif

#define WEMOS_UART_PORT    WEMOS_BRIDGE_UART_PORT
#define WEMOS_UART_TX_PIN  WEMOS_BRIDGE_TX_PIN
#define WEMOS_UART_RX_PIN  WEMOS_BRIDGE_RX_PIN
#define WEMOS_UART_BAUD    WEMOS_BRIDGE_BAUD
#define WEMOS_UART_RX_BUF  1024

#define SYNC_A             0xAAu
#define SYNC_B             0xBBu

#define MSG_CONTROL        0x01u
#define MSG_ODOM           0x02u
#define MSG_PATH_DONE      0x04u
#define MSG_PATH_CHUNK     0x06u
#define MSG_CHUNK_NACK     0x07u

#define HEADER_LEN         4u
#define MAX_PAYLOAD_LEN    255u

static uint8_t checksum_xor(const uint8_t *data, size_t len)
{
    uint8_t ck = 0;
    for (size_t i = 0; i < len; i++) ck ^= data[i];
    return ck;
}

void uart_bridge_init(void)
{
#ifdef ESP_PLATFORM
    uart_config_t cfg = {
        .baud_rate = WEMOS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_delete(WEMOS_UART_PORT);
    uart_param_config(WEMOS_UART_PORT, &cfg);
    uart_set_pin(WEMOS_UART_PORT,
                 WEMOS_UART_TX_PIN, WEMOS_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(WEMOS_UART_PORT, WEMOS_UART_RX_BUF, WEMOS_UART_RX_BUF, 0, NULL, 0);
    uart_flush_input(WEMOS_UART_PORT);
    printf("[UART] init: UART%d TX=%d RX=%d baud=%d\n",
           WEMOS_UART_PORT, WEMOS_UART_TX_PIN, WEMOS_UART_RX_PIN, WEMOS_UART_BAUD);
#endif
}

static bool send_packet(uint8_t msg_type, const void *payload, size_t payload_len)
{
    if (!payload || payload_len > MAX_PAYLOAD_LEN) return false;

#ifdef ESP_PLATFORM
    uint8_t buf[HEADER_LEN + MAX_PAYLOAD_LEN + 1];
    buf[0] = SYNC_A;
    buf[1] = SYNC_B;
    buf[2] = msg_type;
    buf[3] = (uint8_t)payload_len;
    memcpy(&buf[4], payload, payload_len);
    buf[4 + payload_len] = checksum_xor(&buf[2], payload_len + 2);
    const size_t total_len = HEADER_LEN + payload_len + 1u;
    return uart_write_bytes(WEMOS_UART_PORT, (const char *)buf, total_len) == (int)total_len;
#else
    (void)msg_type; (void)payload; (void)payload_len;
    return false;
#endif
}

/* ── odom wire format v2 — must match esp32s3/src/uart_bridge.c exactly ──────
 * v2 adds consumed_wp_idx (2B) + consumed_path_id (2B) → 10 bytes total.     */
typedef struct __attribute__((packed)) {
    int16_t  disp_x16;          /* linear_disp_mm × 16;  0.0625 mm/lsb  */
    int16_t  yaw_mrad_s;        /* yaw_rate_imu × 1000;  1 mrad/s/lsb   */
    uint8_t  dt_ms;
    uint8_t  seq;
    uint16_t consumed_wp_idx;
    uint16_t consumed_path_id;
} odom_wire_t;  /* 10 bytes */
_Static_assert(sizeof(odom_wire_t) == 10u,
               "odom_wire_t size mismatch — reflash both boards together");

bool uart_bridge_send_odom(const odom_t *odom)
{
    if (!odom) return false;

    float disp = odom->linear_disp_mm;
    float yaw  = odom->yaw_rate_imu;
    if (disp >  2047.0f) disp =  2047.0f;
    if (disp < -2047.0f) disp = -2047.0f;
    if (yaw  >  32.0f)   yaw  =  32.0f;
    if (yaw  < -32.0f)   yaw  = -32.0f;

    odom_wire_t w = {
        .disp_x16         = (int16_t)(disp * 16.0f),
        .yaw_mrad_s       = (int16_t)(yaw  * 1000.0f),
        .dt_ms            = (odom->dt_ms > 255.0f) ? 255u : (uint8_t)odom->dt_ms,
        .seq              = (uint8_t)odom->seq,
        .consumed_wp_idx  = odom->consumed_wp_idx,
        .consumed_path_id = odom->consumed_path_id,
    };
    return send_packet(MSG_ODOM, &w, (uint8_t)sizeof(odom_wire_t));
}

/* ── Inbound packet state ─────────────────────────────────────────────────── */
static bool            s_have_chunk      = false;
static path_chunk_t    s_pending_chunk;

static bool            s_have_override   = false;
static control_frame_t s_pending_override;

static void drain_pending_packets(void)
{
#ifdef ESP_PLATFORM
    for (;;) {
        size_t avail = 0;
        uart_get_buffered_data_len(WEMOS_UART_PORT, &avail);
        if (avail == 0) return;

        uint8_t b = 0;
        if (uart_read_bytes(WEMOS_UART_PORT, &b, 1, 0) != 1) return;
        if (b != SYNC_A) continue;

        if (uart_read_bytes(WEMOS_UART_PORT, &b, 1, pdMS_TO_TICKS(5)) != 1) return;
        if (b != SYNC_B) continue;

        uint8_t msg_type = 0, payload_len = 0;
        if (uart_read_bytes(WEMOS_UART_PORT, &msg_type,    1, pdMS_TO_TICKS(5)) != 1) return;
        if (uart_read_bytes(WEMOS_UART_PORT, &payload_len, 1, pdMS_TO_TICKS(5)) != 1) return;

        uint8_t payload[MAX_PAYLOAD_LEN];
        if (uart_read_bytes(WEMOS_UART_PORT, payload, payload_len,
                            pdMS_TO_TICKS(50)) != (int)payload_len) return;

        uint8_t received_ck = 0;
        if (uart_read_bytes(WEMOS_UART_PORT, &received_ck, 1, pdMS_TO_TICKS(10)) != 1) return;

        uint8_t check_buf[2 + MAX_PAYLOAD_LEN];
        check_buf[0] = msg_type;
        check_buf[1] = payload_len;
        memcpy(&check_buf[2], payload, payload_len);
        if (checksum_xor(check_buf, (size_t)payload_len + 2u) != received_ck) continue;

        if (msg_type == MSG_CONTROL && payload_len == sizeof(control_frame_t)) {
            memcpy(&s_pending_override, payload, sizeof(control_frame_t));
            s_have_override = true;
            return;  /* priority: process override before chunks in same window */
        }

        if (msg_type == MSG_PATH_CHUNK && payload_len == sizeof(path_chunk_t)) {
            path_chunk_t tmp;
            memcpy(&tmp, payload, sizeof(path_chunk_t));
            if (tmp.count > 0 && tmp.count <= PATH_CHUNK_WP_COUNT) {
                s_pending_chunk = tmp;
                s_have_chunk    = true;
                return;  /* One chunk per call — prevents chunk2 overwriting chunk1
                          * when both arrive in the same 50 ms PP tick window.     */
            }
        }
        /* Unknown types silently dropped. */
    }
#endif
}

bool uart_bridge_recv_path_chunk(path_chunk_t *out)
{
    if (!out) return false;
    drain_pending_packets();
    if (s_have_chunk) {
        *out         = s_pending_chunk;
        s_have_chunk = false;
        return true;
    }
    return false;
}

bool uart_bridge_send_path_done(void)
{
    uint8_t done = 1u;
    return send_packet(MSG_PATH_DONE, &done, 1u);
}

bool uart_bridge_send_chunk_nack(uint16_t path_id, uint16_t expected_start)
{
    uint8_t payload[4];
    memcpy(&payload[0], &path_id,        2);
    memcpy(&payload[2], &expected_start, 2);
    return send_packet(MSG_CHUNK_NACK, payload, 4u);
}

bool uart_bridge_recv_control_override(control_frame_t *out)
{
    if (!out) return false;
    drain_pending_packets();
    if (s_have_override) {
        *out            = s_pending_override;
        s_have_override = false;
        return true;
    }
    return false;
}
