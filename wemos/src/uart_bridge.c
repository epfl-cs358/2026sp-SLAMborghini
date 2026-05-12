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
 *   UART @ 921600 baud, 8N1  (baud from WEMOS_BRIDGE_BAUD in hardware_pins.h)
 *
 * Wiring:
 *   Wemos RX GPIO16 <- ESP32-S3 TX GPIO17
 *   Wemos TX GPIO17 -> ESP32-S3 RX GPIO16
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
#include "../../hardware_pins.h"

#include <string.h>

#ifdef ESP_PLATFORM
#include "driver/uart.h"
#include "driver/gpio.h"
#endif

/* Local aliases — rest of file uses these names regardless of which
 * board's symbols hardware_pins.h exposes. */
#define BRIDGE_UART_PORT   WEMOS_BRIDGE_UART_PORT
#define BRIDGE_TX_PIN      WEMOS_BRIDGE_TX_PIN
#define BRIDGE_RX_PIN      WEMOS_BRIDGE_RX_PIN
#define BRIDGE_UART_BAUD   WEMOS_BRIDGE_BAUD
#define BRIDGE_RX_BUF      1024

/* ------------------------------------------------------------
 * Protocol constants
 * ------------------------------------------------------------ */
#define SYNC_A             0xAAu
#define SYNC_B             0xBBu

#define MSG_CONTROL        0x01u
#define MSG_ODOM           0x02u
#define MSG_PATH           0x03u
#define MSG_PATH_DONE      0x04u
#define MSG_PATH_ACK       0x05u

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

/* ── Compact wire format for MSG_ODOM ────────────────────────────────────────
 * odom_t has 4 floats (16 bytes) but the actual value ranges are tiny:
 *   linear_disp_mm  ≤ ±25 mm per 50 ms tick  → int16_t × 16 lsb/mm
 *   yaw_rate_imu    ≤ ±10 rad/s               → int16_t in mrad/s
 *   dt_ms           1–255 ms                  → uint8_t
 *   seq             gap-detection only         → uint8_t (wraps at 256)
 * Wire payload: 6 bytes (was 16).  On-wire total: 11 bytes (was 22).
 * ──────────────────────────────────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    int16_t  disp_x16;   /* linear_disp_mm × 16;  0.0625 mm/lsb; ±2047.9 mm  */
    int16_t  yaw_mrad_s; /* yaw_rate_imu × 1000;  1 mrad/s/lsb;  ±32.767 r/s */
    uint8_t  dt_ms;      /* integration window ms; 1 ms/lsb; max 255 ms       */
    uint8_t  seq;        /* low 8 bits of sequence counter                     */
} odom_wire_t;

/* Send odometry packet to ESP32-S3 (compact wire encoding). */
bool uart_bridge_send_odom(const odom_t *odom)
{
    if (!odom) return false;

    /* Clamp before encoding to avoid int16_t overflow on sensor glitch. */
    float disp   = odom->linear_disp_mm;
    float yaw    = odom->yaw_rate_imu;
    if (disp >  2047.0f) disp =  2047.0f;
    if (disp < -2047.0f) disp = -2047.0f;
    if (yaw  >  32.0f)   yaw  =  32.0f;
    if (yaw  < -32.0f)   yaw  = -32.0f;

    odom_wire_t w = {
        .disp_x16   = (int16_t)(disp * 16.0f),
        .yaw_mrad_s = (int16_t)(yaw  * 1000.0f),
        .dt_ms      = (odom->dt_ms > 255.0f) ? 255u : (uint8_t)odom->dt_ms,
        .seq        = (uint8_t)odom->seq,
    };
    return send_packet(MSG_ODOM, &w, (uint8_t)sizeof(odom_wire_t));
}

/* ------------------------------------------------------------
 * Pending-slot state — one slot per inbound message type.
 * drain_pending_packets() is the sole UART reader for inbound
 * traffic; both recv_control and recv_path just check their slot.
 * ------------------------------------------------------------ */
static bool            s_have_control  = false;
static control_frame_t s_pending_control;
static bool            s_have_path     = false;
static path_frame_t    s_pending_path;

/* Read every complete, valid packet currently in the UART FIFO.
 * Latest-wins per message type — older packets are overwritten. */
static void drain_pending_packets(void)
{
#ifdef ESP_PLATFORM
    for (;;) {
        size_t avail = 0;
        uart_get_buffered_data_len(BRIDGE_UART_PORT, &avail);
        if (avail == 0) return;

        /* Scan for SYNC_A */
        uint8_t b = 0;
        if (uart_read_bytes(BRIDGE_UART_PORT, &b, 1, 0) != 1) return;
        if (b != SYNC_A) continue;

        /* Expect SYNC_B immediately after */
        if (uart_read_bytes(BRIDGE_UART_PORT, &b, 1, pdMS_TO_TICKS(5)) != 1) return;
        if (b != SYNC_B) continue;

        uint8_t msg_type = 0, payload_len = 0;
        if (uart_read_bytes(BRIDGE_UART_PORT, &msg_type,    1, pdMS_TO_TICKS(5)) != 1) return;
        if (uart_read_bytes(BRIDGE_UART_PORT, &payload_len, 1, pdMS_TO_TICKS(5)) != 1) return;

        uint8_t payload[MAX_PAYLOAD_LEN];
        if (uart_read_bytes(BRIDGE_UART_PORT, payload, payload_len,
                            pdMS_TO_TICKS(50)) != (int)payload_len) return;

        uint8_t received_ck = 0;
        if (uart_read_bytes(BRIDGE_UART_PORT, &received_ck, 1,
                            pdMS_TO_TICKS(10)) != 1) return;

        uint8_t check_buf[2 + MAX_PAYLOAD_LEN];
        check_buf[0] = msg_type;
        check_buf[1] = payload_len;
        memcpy(&check_buf[2], payload, payload_len);
        if (checksum_xor(check_buf, (uint8_t)(payload_len + 2u)) != received_ck) continue;

        if (msg_type == MSG_CONTROL && payload_len == sizeof(control_frame_t)) {
            memcpy(&s_pending_control, payload, sizeof(control_frame_t));
            s_have_control = true;
        } else if (msg_type == MSG_PATH && payload_len == sizeof(path_frame_t)) {
            path_frame_t tmp;
            memcpy(&tmp, payload, sizeof(path_frame_t));
            if (tmp.length > 0 && tmp.length <= MAX_SHARED_PATH_POINTS) {
                s_pending_path = tmp;
                s_have_path    = true;
            }
        }
        /* Unknown types are silently dropped (already consumed). */
    }
#endif
}

/* ------------------------------------------------------------
 * Receive control packet from ESP32-S3
 * ------------------------------------------------------------ */
bool uart_bridge_recv_control(control_frame_t *out)
{
    if (!out) return false;
    drain_pending_packets();
    if (s_have_control) {
        *out           = s_pending_control;
        s_have_control = false;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------
 * Receive path frame from ESP32-S3
 * ------------------------------------------------------------ */
bool uart_bridge_recv_path(path_frame_t *out)
{
    if (!out) return false;
    drain_pending_packets();
    if (s_have_path) {
        *out        = s_pending_path;
        s_have_path = false;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------
 * Send path-done notification to ESP32-S3
 * ------------------------------------------------------------ */
bool uart_bridge_send_path_done(void)
{
    uint8_t done = 1u;
    return send_packet(MSG_PATH_DONE, &done, 1u);
}

/* ------------------------------------------------------------
 * Send path-ACK to ESP32-S3 (called immediately after recv_path)
 * payload = number of waypoints received, as a cross-check
 * ------------------------------------------------------------ */
bool uart_bridge_send_path_ack(uint8_t path_len)
{
    return send_packet(MSG_PATH_ACK, &path_len, 1u);
}