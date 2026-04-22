/**
 * lidar_driver.c
 * Module: RPLiDAR C1 UART driver — minimal first-test version.
 * Board: ESP32 / Wemos D1 R32
 *
 * ⚠️  MINIMAL VERSION — written to enable the first real hardware test.
 *     Only the legacy scan command (0xA5 0x20) is implemented.
 *     Express scan, health-check, and graceful stop are NOT implemented.
 *     Error recovery is minimal: a timeout aborts the scan and returns
 *     whatever points were collected.
 *
 * Wiring — same pins as 2025fa-SLAMurai/code/firmware/wemos/include/
 *          LIDAR_control/config.hpp (see also LIDAR_control.cpp Serial2.begin):
 *   LIDAR_RX_PIN = 17  → ESP32 UART RX (LiDAR device TX wire here)
 *   LIDAR_TX_PIN = 16  → ESP32 UART TX (LiDAR device RX wire here)
 *   LIDAR_BAUDRATE = 460800 (UART_NUM_1 here; SLAMurai uses Serial2 on the same pins)
 *   LiDAR VCC → 5 V, GND → GND
 *   MOTOCTL — not used; motor from VCC (SLAMurai approach).
 *
 * RPLiDAR C1 legacy scan packet (5 bytes):
 *   Byte 0 : [quality:6][start_bit:1][inv_start_bit:1]
 *   Byte 1 : [angle_q6_low7:7][check_bit:1]   (check_bit must be 1)
 *   Byte 2 : [angle_q6_high8:8]
 *   Byte 3 : [dist_q2_low8:8]
 *   Byte 4 : [dist_q2_high8:8]
 *
 *   angle_deg  = ((byte2 << 8 | byte1) >> 1) / 64.0
 *   dist_mm    = (byte4 << 8 | byte3) / 4.0
 *   new scan starts when start_bit == 1
 */

#include "lidar_driver.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "lidar_drv";

/* ── Hardware config — values == 2025fa-SLAMurai LIDAR_control/config.hpp ─ */
#define LIDAR_UART_PORT   UART_NUM_1
#define LIDAR_UART_RX     16          /* GPIO 16 = Serial2 default RX: LiDAR TX → ESP */
#define LIDAR_UART_TX     17          /* GPIO 17 = Serial2 default TX: ESP → LiDAR RX */
#define LIDAR_BAUD        460800      /* LIDAR_BAUDRATE */
#define LIDAR_RX_BUF      5000        /* matches SLAMurai LIDAR_SERIAL_BUFFER_SIZE */

/* ── Commands ────────────────────────────────────────────────────────────── */
static const uint8_t CMD_SCAN[] = { 0xA5, 0x20 };
static const uint8_t CMD_STOP[] = { 0xA5, 0x25 };

/* ── Response descriptor (7 bytes, follows each command) ─────────────────── */
#define RESP_DESC_LEN 7

/* ── Scan packet size ────────────────────────────────────────────────────── */
#define PKT_LEN 5

/* ── Max range filter (skip readings beyond this distance) ───────────────── */
#define MAX_RANGE_MM 6000.0f
#define MIN_RANGE_MM  100.0f   /* ignore points closer than 10 cm (self-hits) */


/* ════════════════════════════════════════════════════════════════════════════
 * lidar_driver_init
 * ════════════════════════════════════════════════════════════════════════════ */
void lidar_driver_init(void)
{
    /* ── UART init ──────────────────────────────────────────────────────── */
    uart_config_t ucfg = {
        .baud_rate  = LIDAR_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(LIDAR_UART_PORT, &ucfg);

    /* Match Arduino Serial1.begin() — explicitly pull-up RX so the idle line
     * is high. ESP-IDF uart_set_pin does not do this, leaving RX floating. */
    gpio_reset_pin(LIDAR_UART_RX);
    gpio_set_direction(LIDAR_UART_RX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LIDAR_UART_RX, GPIO_PULLUP_ONLY);

    uart_set_pin(LIDAR_UART_PORT,
                 LIDAR_UART_TX, LIDAR_UART_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LIDAR_UART_PORT, LIDAR_RX_BUF, 0, 0, NULL, 0);

    /* Let the LiDAR boot before sending any command */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Flush any garbage in the RX buffer */
    uart_flush(LIDAR_UART_PORT);

    /* Check if LIDAR is already streaming (leftover from a prior session) */
    uint8_t probe;
    int pre = uart_read_bytes(LIDAR_UART_PORT, &probe, 1, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Pre-command RX probe: %d byte(s) available (0x%02X)", pre, pre > 0 ? probe : 0);

    /* ── Send SCAN command ──────────────────────────────────────────────── */
    int sent = uart_write_bytes(LIDAR_UART_PORT, (const char *)CMD_SCAN, sizeof(CMD_SCAN));
    ESP_LOGI(TAG, "CMD_SCAN sent: %d byte(s)", sent);

    /* ── Read 7-byte response descriptor ────────────────────────────────── */
    uint8_t desc[RESP_DESC_LEN];
    int n = uart_read_bytes(LIDAR_UART_PORT, desc, RESP_DESC_LEN, pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Response descriptor: %d/7 bytes — 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             n,
             n > 0 ? desc[0] : 0, n > 1 ? desc[1] : 0, n > 2 ? desc[2] : 0,
             n > 3 ? desc[3] : 0, n > 4 ? desc[4] : 0, n > 5 ? desc[5] : 0,
             n > 6 ? desc[6] : 0);

    if (n < RESP_DESC_LEN) {
        ESP_LOGW(TAG, "Response descriptor incomplete (%d/7 bytes) — LiDAR may not be connected", n);
    } else if (desc[0] != 0xA5 || desc[1] != 0x5A) {
        ESP_LOGW(TAG, "Response descriptor sync mismatch (0x%02X 0x%02X)", desc[0], desc[1]);
    } else {
        ESP_LOGI(TAG, "RPLiDAR C1 scan started");
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * lidar_driver_read_scan
 *
 * Reads one complete 360° scan.
 * Strategy:
 *   1. Search byte-by-byte for a packet with start_bit=1 (new rotation).
 *   2. Collect subsequent 5-byte packets until the next start_bit=1.
 *   3. Timeout after 3 s; return whatever was collected (count > 0 = partial OK).
 * ════════════════════════════════════════════════════════════════════════════ */
bool lidar_driver_read_scan(lidar_scan_t *out)
{
    if (!out) return false;
    out->count = 0;

    /* ── Phase 1: find the start of a new rotation ──────────────────────── */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    bool synced = false;
    uint8_t b;

    while (!synced && xTaskGetTickCount() < deadline) {
        if (uart_read_bytes(LIDAR_UART_PORT, &b, 1, pdMS_TO_TICKS(50)) != 1) continue;

        bool start_bit = (b & 0x01) != 0;   /* bit 0 = start flag (per SLAMurai) */
        bool inv_bit   = (b & 0x02) != 0;   /* bit 1 = inverted start flag */
        if (!start_bit || inv_bit) continue;   /* not a start packet byte-0 */

        /* Read remaining 4 bytes of this start packet */
        uint8_t rest[4];
        if (uart_read_bytes(LIDAR_UART_PORT, rest, 4, pdMS_TO_TICKS(50)) != 4) continue;
        if (!(rest[0] & 0x01)) continue;       /* check_bit must be 1 */

        float angle = (uint16_t)((rest[1] << 8) | rest[0]) >> 1;
        angle /= 64.0f;
        float dist  = (uint16_t)((rest[3] << 8) | rest[2]) / 4.0f;

        if (dist >= MIN_RANGE_MM && dist <= MAX_RANGE_MM && out->count < 460) {
            out->points[out->count].r_mm       = dist;
            out->points[out->count].theta_deg  = angle;
            out->points[out->count].intensity  = (b >> 2) & 0x3F;
            out->count++;
        }
        synced = true;
    }

    if (!synced) {
        ESP_LOGW(TAG, "lidar_driver_read_scan: no start packet — LiDAR not spinning?");
        return false;
    }


    /* ── Phase 2: collect packets until the next start_bit=1 ───────────── */
    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);   /* one rotation ≤ 200 ms */

    while (xTaskGetTickCount() < deadline) {
        uint8_t pkt[PKT_LEN];
        if (uart_read_bytes(LIDAR_UART_PORT, pkt, PKT_LEN, pdMS_TO_TICKS(50)) != PKT_LEN) continue;

        bool start_bit = (pkt[0] & 0x01) != 0;   /* bit 0 = start flag (per SLAMurai) */
        bool inv_bit   = (pkt[0] & 0x02) != 0;   /* bit 1 = inverted start flag */
        bool check_bit = (pkt[1] & 0x01) != 0;

        /* Validate packet */
        if ((start_bit ^ inv_bit) != 1) continue;
        if (!check_bit) continue;

        if (start_bit) break;   /* new rotation — scan complete */

        float angle = (uint16_t)((pkt[2] << 8) | pkt[1]) >> 1;
        angle /= 64.0f;
        float dist  = (uint16_t)((pkt[4] << 8) | pkt[3]) / 4.0f;
        uint8_t q   = (pkt[0] >> 2) & 0x3F;

        if (dist >= MIN_RANGE_MM && dist <= MAX_RANGE_MM && q > 0 && out->count < 460) {
            out->points[out->count].r_mm      = dist;
            out->points[out->count].theta_deg = angle;
            out->points[out->count].intensity = q;
            out->count++;
        }
    }

    ESP_LOGD(TAG, "Scan: %u points", out->count);
    return out->count > 10;   /* need at least a few points for a useful scan */
}


/* ════════════════════════════════════════════════════════════════════════════
 * lidar_driver_stop
 * ════════════════════════════════════════════════════════════════════════════ */
void lidar_driver_stop(void)
{
    uart_write_bytes(LIDAR_UART_PORT, (const char *)CMD_STOP, sizeof(CMD_STOP));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_driver_delete(LIDAR_UART_PORT);
    ESP_LOGI(TAG, "LiDAR stopped");
}
