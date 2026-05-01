/**
 * lidar_driver.c
 * Module: RPLiDAR C1 UART driver — minimal first-test version.
 * Board: ESP32-S3 (SLAM brain)
 *
 * ⚠️  MINIMAL VERSION — written to enable the first real hardware test.
 *     Only the legacy scan command (0xA5 0x20) is implemented.
 *     Express scan, health-check, and graceful stop are NOT implemented.
 *     Error recovery is minimal: a timeout aborts the scan and returns
 *     whatever points were collected.
 *
 * Wiring — see hardware_pins.h:
 *   LIDAR_UART_RX = GPIO16  → ESP32-S3 RX (LiDAR TX wire)
 *   LIDAR_UART_TX = GPIO15  → ESP32-S3 TX (LiDAR RX wire)
 *   LIDAR_BAUD    = 460800
 *   LiDAR VCC → 5 V, GND → GND
 *   MOTOCTL — not used; motor powered from VCC.
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
#include "../../hardware_pins.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "lidar_drv";

#define LIDAR_RX_BUF      5000

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
    vTaskDelay(pdMS_TO_TICKS(2000));   /* longer wait — let motor spin up */

    /* Drain anything the LiDAR auto-streamed on power-up */
    uart_flush(LIDAR_UART_PORT);

    /* ── Raw RX sniff: detect if LiDAR is already streaming ─────────────── */
    uint8_t sniff[16];
    int ns = uart_read_bytes(LIDAR_UART_PORT, sniff, sizeof(sniff), pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Raw RX sniff (no cmd): %d byte(s)", ns);

    if (ns > 0) {
        /* LiDAR is already scanning (leftover from prior session) — stop it */
        ESP_LOGI(TAG, "LiDAR already streaming, sending CMD_STOP first");
        uart_write_bytes(LIDAR_UART_PORT, (const char *)CMD_STOP, sizeof(CMD_STOP));
        vTaskDelay(pdMS_TO_TICKS(500));
        uart_flush(LIDAR_UART_PORT);
    }

    /* ── Send SCAN command ──────────────────────────────────────────────── */
    int sent = uart_write_bytes(LIDAR_UART_PORT, (const char *)CMD_SCAN, sizeof(CMD_SCAN));
    ESP_LOGI(TAG, "CMD_SCAN sent: %d byte(s) on TX=GPIO%d RX=GPIO%d",
             sent, LIDAR_UART_TX, LIDAR_UART_RX);

    /* ── Read 7-byte response descriptor ────────────────────────────────── */
    uint8_t desc[RESP_DESC_LEN];
    int n = uart_read_bytes(LIDAR_UART_PORT, desc, RESP_DESC_LEN, pdMS_TO_TICKS(2000));
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

    /* Sliding-window parser: reads one byte at a time into a 5-byte window.
     * When the window holds a valid packet (S^!S==1, check_bit==1) it is
     * consumed; otherwise the window shifts by one byte (self-healing sync).
     * Collecting starts on the first valid start packet (S=1) and ends on
     * the next, giving exactly one 360° scan. */

    uint8_t  win[5];
    int      wlen       = 0;
    bool     collecting = false;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);

    while (xTaskGetTickCount() < deadline && out->count < 460) {
        uint8_t b;
        if (uart_read_bytes(LIDAR_UART_PORT, &b, 1, pdMS_TO_TICKS(50)) != 1) continue;

        win[wlen++] = b;
        if (wlen < 5) continue;

        bool s  = (win[0] & 0x01) != 0;   /* start bit */
        bool ns = (win[0] & 0x02) != 0;   /* inverted start bit */
        bool ck = (win[1] & 0x01) != 0;   /* check bit */

        if ((s ^ ns) != 1 || !ck) {
            /* Invalid packet — slide forward one byte and try again */
            memmove(win, win + 1, 4);
            wlen = 4;
            continue;
        }

        /* Valid 5-byte packet consumed */
        wlen = 0;

        if (s && collecting) break;    /* second start → one full rotation done */

        float angle = (uint16_t)((win[2] << 8) | win[1]) >> 1;
        angle /= 64.0f;
        float dist  = (uint16_t)((win[4] << 8) | win[3]) / 4.0f;
        uint8_t q   = (win[0] >> 2) & 0x3F;

        if (s) collecting = true;      /* first start packet — begin collecting */

        if (collecting && dist >= MIN_RANGE_MM && dist <= MAX_RANGE_MM && q > 0) {
            out->points[out->count].r_mm      = dist;
            out->points[out->count].theta_deg = angle;
            out->points[out->count].intensity = q;
            out->count++;
        }
    }

    if (!collecting) {
        ESP_LOGW(TAG, "lidar_driver_read_scan: no start packet — LiDAR not spinning?");
        return false;
    }

    ESP_LOGI(TAG, "Scan: %u points", out->count);
    return out->count > 10;
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
