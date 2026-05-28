/**
 * lidar_driver.c
 * Module: RPLiDAR C1 UART driver.
 * Board: ESP32-S3 (SLAM brain)
 *
 * ── RPLiDAR C1 legacy scan packet (5 bytes) ─────────────────────────────────
 *   Byte 0 : [quality:6][start_bit:1][inv_start_bit:1]
 *   Byte 1 : [angle_q6_low7:7][check_bit:1]   (check_bit must be 1)
 *   Byte 2 : [angle_q6_high8:8]
 *   Byte 3 : [dist_q2_low8:8]
 *   Byte 4 : [dist_q2_high8:8]
 *
 *   angle_deg  = ((byte2<<8 | byte1) >> 1) / 64.0
 *   dist_mm    = (byte4<<8 | byte3) / 4.0
 *   new scan starts when start_bit == 1
 *
 * ── Timing design ───────────────────────────────────────────────────────────
 * The LiDAR spins at ~10 Hz (100 ms/rotation) and streams packets continuously
 * over UART at 460800 baud.  Each 5-byte packet takes ≈ 0.087 ms to arrive.
 *
 * Old problem: uart_read_bytes called once per byte with a 50 ms timeout.
 *   One brief FIFO stall → 50 ms penalty → scan acquisition routinely
 *   exceeded the 100 ms rotation period (observed: ~130 ms total).
 *
 * Fix: read in 64-byte chunks with a 2 ms timeout.  Stall penalty is capped
 * at 2 ms per chunk.  The static s_rb buffer persists between calls; bytes
 * left over from one call are consumed at the start of the next, eliminating
 * the per-call re-sync cost.
 *
 * ── No-return beams ─────────────────────────────────────────────────────────
 * When the LiDAR gets no echo (open space, absorptive surface, beyond ~8 m)
 * it outputs a packet with quality = 0 and dist = 0.  Previously these were
 * silently dropped, leaving the map UNKNOWN in open directions.
 * Now they are stored with r_mm = 0 so lidar_to_map can mark free space
 * along the ray up to max_range_mm — the ray is clear, just very long.
 *
 * ── Seed-packet carry ───────────────────────────────────────────────────────
 * When the second start-bit packet ends a scan, it is also the first packet
 * of the next scan.  We save it as s_seed[] so the following call can
 * immediately start collecting without searching for a start-bit.
 */

#include "lidar_driver.h"
#include "../../hardware_pins.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "lidar_drv";

#define LIDAR_RX_BUF      5000

/* ── Commands ────────────────────────────────────────────────────────────── */
static const uint8_t CMD_SCAN[] = { 0xA5, 0x20 };
static const uint8_t CMD_STOP[] = { 0xA5, 0x25 };

#define RESP_DESC_LEN 7
#define PKT_LEN       5

/* ── Self-hit filter (ignore returns closer than this) ───────────────────── */
#define MIN_RANGE_MM 100u

/* ═══════════════════════════════════════════════════════════════════════════
 * Bulk read buffer — persistent across calls.
 * RBUF_CHUNK bytes are read from the UART FIFO in one call.  Any bytes
 * remaining after a scan boundary are consumed at the start of the next call;
 * this eliminates per-call re-sync searches.
 * ═══════════════════════════════════════════════════════════════════════════ */
#define RBUF_CAP     256
#define RBUF_CHUNK    64
#define RBUF_TMO_MS    2

static uint8_t s_rb[RBUF_CAP];
static int     s_rb_head = 0;
static int     s_rb_tail = 0;

/* ── Seed packet from the previous scan's terminal start-bit ─────────────── */
static bool    s_have_seed = false;
static uint8_t s_seed[PKT_LEN] = {0};

/* Read one byte from the bulk buffer, refilling from UART as needed.
 * Returns false only if the per-scan deadline has passed. */
static bool rb_read(uint8_t *b, TickType_t deadline)
{
    while (s_rb_head >= s_rb_tail) {
        if (xTaskGetTickCount() >= deadline) return false;
        s_rb_head = s_rb_tail = 0;
        int n = uart_read_bytes(LIDAR_UART_PORT, s_rb, RBUF_CHUNK,
                                pdMS_TO_TICKS(RBUF_TMO_MS));
        if (n > 0) s_rb_tail = n;
    }
    *b = s_rb[s_rb_head++];
    return true;
}

static void rb_reset(void)
{
    s_rb_head   = 0;
    s_rb_tail   = 0;
    s_have_seed = false;
}


/* ════════════════════════════════════════════════════════════════════════════
 * lidar_driver_init
 * ════════════════════════════════════════════════════════════════════════════ */
void lidar_driver_init(void)
{
    rb_reset();

    uart_config_t ucfg = {
        .baud_rate  = LIDAR_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(LIDAR_UART_PORT, &ucfg);

    gpio_reset_pin(LIDAR_UART_RX);
    gpio_set_direction(LIDAR_UART_RX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LIDAR_UART_RX, GPIO_PULLUP_ONLY);

    uart_set_pin(LIDAR_UART_PORT,
                 LIDAR_UART_TX, LIDAR_UART_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LIDAR_UART_PORT, LIDAR_RX_BUF, 0, 0, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(2000));
    uart_flush(LIDAR_UART_PORT);

    uint8_t sniff[16];
    int ns = uart_read_bytes(LIDAR_UART_PORT, sniff, sizeof(sniff), pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Raw RX sniff (no cmd): %d byte(s)", ns);

    if (ns > 0) {
        ESP_LOGI(TAG, "LiDAR already streaming, sending CMD_STOP first");
        uart_write_bytes(LIDAR_UART_PORT, (const char *)CMD_STOP, sizeof(CMD_STOP));
        vTaskDelay(pdMS_TO_TICKS(500));
        uart_flush(LIDAR_UART_PORT);
    }

    int sent = uart_write_bytes(LIDAR_UART_PORT, (const char *)CMD_SCAN, sizeof(CMD_SCAN));
    ESP_LOGI(TAG, "CMD_SCAN sent: %d byte(s) on TX=GPIO%d RX=GPIO%d",
             sent, LIDAR_UART_TX, LIDAR_UART_RX);

    uint8_t desc[RESP_DESC_LEN];
    int n = uart_read_bytes(LIDAR_UART_PORT, desc, RESP_DESC_LEN, pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Response descriptor: %d/7 bytes — "
             "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             n,
             n > 0 ? desc[0] : 0, n > 1 ? desc[1] : 0, n > 2 ? desc[2] : 0,
             n > 3 ? desc[3] : 0, n > 4 ? desc[4] : 0, n > 5 ? desc[5] : 0,
             n > 6 ? desc[6] : 0);

    if (n < RESP_DESC_LEN) {
        ESP_LOGW(TAG, "Response descriptor incomplete — LiDAR may not be connected");
    } else if (desc[0] != 0xA5 || desc[1] != 0x5A) {
        ESP_LOGW(TAG, "Response descriptor sync mismatch");
    } else {
        ESP_LOGI(TAG, "RPLiDAR C1 scan started");
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * lidar_driver_read_scan
 *
 * Reads one complete 360° scan.  The scan starts at the first observed
 * start-bit packet and ends when the next start-bit is detected.  That
 * terminal packet is saved as s_seed[] so the FOLLOWING call resumes
 * immediately without a re-sync search.
 *
 * Point classification:
 *   quality == 0                → no-return; stored with r_mm = 0.
 *                                 lidar_to_map treats r=0 as "mark free to
 *                                 max_range_mm" (no obstacle endpoint).
 *   0 < dist < MIN_RANGE_MM     → self-hit; dropped.
 *   dist >= MIN_RANGE_MM        → valid return; stored as-is.
 *                                 The map's max_range_mm parameter clips
 *                                 very far returns during ray-marching.
 * ════════════════════════════════════════════════════════════════════════════ */
bool lidar_driver_read_scan(lidar_scan_t *out)
{
    if (!out) return false;
    out->count              = 0;
    out->scan_start_us      = 0;
    out->rotation_period_us = 0;

    uint8_t win[PKT_LEN];
    int     wlen        = 0;
    bool    collecting  = false;
    int64_t scan_start_us = 0;

    /* If the previous call saved the first packet of the new scan, seed it. */
    if (s_have_seed) {
        memcpy(win, s_seed, PKT_LEN);
        wlen        = PKT_LEN;
        s_have_seed = false;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);

    while (xTaskGetTickCount() < deadline && out->count < 460) {

        /* Fill the sliding window one byte at a time from the bulk buffer */
        if (wlen < PKT_LEN) {
            uint8_t b;
            if (!rb_read(&b, deadline)) {
                /* Deadline hit — return partial scan if it has enough points */
                if (collecting && out->count > 10) break;
                /* Otherwise: timeout without finding a scan boundary */
                ESP_LOGW(TAG, "lidar_driver_read_scan: deadline hit (partial %u pts)",
                         (unsigned)out->count);
                s_have_seed = false;
                return false;
            }
            win[wlen++] = b;
            if (wlen < PKT_LEN) continue;
        }

        /* Validate the 5-byte window */
        bool s  = (win[0] & 0x01) != 0;
        bool ns = (win[0] & 0x02) != 0;
        bool ck = (win[1] & 0x01) != 0;

        if ((s ^ ns) != 1 || !ck) {
            /* Invalid packet — slide forward one byte and retry */
            memmove(win, win + 1, PKT_LEN - 1);
            wlen = PKT_LEN - 1;
            continue;
        }

        /* Valid 5-byte packet ─────────────────────────────────────────────── */

        if (s && collecting) {
            /* Second start-bit marks the end of this scan and the beginning
             * of the next.  Save as seed so the next call skips re-sync. */
            memcpy(s_seed, win, PKT_LEN);
            s_have_seed = true;
            wlen = 0;

            int64_t scan_end_us = esp_timer_get_time();
            out->scan_start_us      = (uint32_t)scan_start_us;
            out->rotation_period_us = (scan_end_us > scan_start_us)
                                      ? (uint32_t)(scan_end_us - scan_start_us)
                                      : 100000u;
            break;
        }

        wlen = 0;  /* consume the packet */

        uint16_t angle_q6   = (uint16_t)(((uint16_t)(win[2] << 8) | win[1]) >> 1);
        uint16_t dist_q2    = (uint16_t)((win[4] << 8) | win[3]);
        uint16_t angle_cdeg = (uint16_t)(((uint32_t)angle_q6 * 100u + 32u) / 64u);
        uint16_t dist_mm    = (uint16_t)(((uint32_t)dist_q2 + 2u) / 4u);
        uint8_t  q          = (win[0] >> 2) & 0x3F;
        if (angle_cdeg >= 36000u) angle_cdeg = (uint16_t)(angle_cdeg - 36000u);

        if (s) {
            collecting    = true;
            scan_start_us = esp_timer_get_time();
        }

        if (!collecting || out->count >= 460) continue;

        if (q == 0) {
            /* No-return beam — r=0 signals "free along ray" to lidar_to_map */
            out->points[out->count++] = (lidar_scan_point_t){
                .r_mm         = 0u,
                .theta_cdeg   = angle_cdeg,
                .intensity    = 0,
                .timestamp_us = 0,
            };
        } else if (dist_mm >= MIN_RANGE_MM) {
            /* Valid return — upper range limit applied by lidar_to_map */
            out->points[out->count++] = (lidar_scan_point_t){
                .r_mm         = dist_mm,
                .theta_cdeg   = angle_cdeg,
                .intensity    = q,
                .timestamp_us = 0,
            };
        }
        /* else: q>0 but dist < MIN_RANGE_MM → self-hit, drop */
    }

    if (!collecting) {
        ESP_LOGW(TAG, "lidar_driver_read_scan: no start packet — LiDAR not spinning?");
        s_have_seed = false;
        return false;
    }

    /* Back-compute per-point timestamps from angular position within rotation.
     * Includes no-return beams (intensity=0) — their angles are valid. */
    if (out->count > 0 && out->rotation_period_us > 0) {
        uint16_t theta_start = out->points[0].theta_cdeg;
        for (uint16_t i = 0; i < out->count; i++) {
            int32_t off = (int32_t)out->points[i].theta_cdeg - (int32_t)theta_start;
            if (off < 0) off += 36000;
            out->points[i].timestamp_us = out->scan_start_us
                + (uint32_t)(((uint64_t)(uint32_t)off * out->rotation_period_us) / 36000u);
        }
    }

    ESP_LOGI(TAG, "Scan: %u pts  period=%lu µs",
             out->count, (unsigned long)out->rotation_period_us);
    return out->count > 10;
}


/* ════════════════════════════════════════════════════════════════════════════
 * lidar_driver_stop
 * ════════════════════════════════════════════════════════════════════════════ */
void lidar_driver_stop(void)
{
    rb_reset();
    uart_write_bytes(LIDAR_UART_PORT, (const char *)CMD_STOP, sizeof(CMD_STOP));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_driver_delete(LIDAR_UART_PORT);
    ESP_LOGI(TAG, "LiDAR stopped");
}
