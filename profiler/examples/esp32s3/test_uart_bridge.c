/**
 * test_uart_bridge.c — UART stress test: ESP32-S3 sender.
 *
 * REQUIRES HARDWARE: Wemos D1 R32 running test_uart_echo (wemos profiler)
 *   S3 TX GPIO17 → Wemos RX GPIO16
 *   S3 RX GPIO16 ← Wemos TX GPIO17
 *   GND shared — 115200 baud, 8N1
 *
 * Works standalone too (send-only path, no Wemos): drop rate stays 0,
 * recv_total stays 0, but send timing and TX buffer fill are still valid.
 *
 * ── Wire math ─────────────────────────────────────────────────────────────────
 *   control_frame_t  16 B payload → packet = 21 B → wire time = 1.82 ms
 *   path_frame_t    242 B payload → packet = 247 B → wire time = 21.5 ms
 *   odom_t (reply)   16 B payload → packet = 21 B → wire time = 1.82 ms
 *
 *   Wire capacity: 11520 B/s @ 115200 baud
 *   Ctrl-only cliff:        11520 / 21        ≈ 548 Hz
 *   Mixed (90%ctrl+10%path): 11520 / 43.6 B  ≈ 264 Hz  ← expected cliff
 *
 *   S3 RX buffer: 512 B → holds ~24 odom packets before overflow
 *
 * ── Stage schedule ───────────────────────────────────────────────────────────
 *   Stage   Hz    Packets   Period ms   Notes
 *     0     10       50      100        baseline
 *     1     20      100       50
 *     2     50      250       20
 *     3    100      500       10
 *     4    200     1000        5
 *     5    500     2000        2
 *     6   BURST   1000        0        no delay — find absolute max
 *
 *   Every 10th packet within a stage is a path_frame (247 B) instead of a
 *   control_frame (21 B). This drives total wire traffic above the baud ceiling
 *   at high Hz and is where drops first appear.
 *
 * ── Per-cycle measurements ────────────────────────────────────────────────────
 *   ctrl_us   uart_bridge_send_control() wall time
 *   path_us   uart_bridge_send_path()    wall time (every 10th)
 *   recv_us   drain all pending odom replies — uart_bridge_recv_odom()
 *   rx_depth  uart_get_buffered_data_len() before drain (S3 RX backlog)
 *
 * ── Per-stage summary ─────────────────────────────────────────────────────────
 *   sends_ok / sends_fail (ctrl + path separately)
 *   recvs_ok (odom packets received back from Wemos)
 *   ctrl_us min/max/avg  path_us min/max/avg
 *   rx_depth max (approaching 512 = overflow)
 *   actual_hz = sends_ok / elapsed_s
 *   Cliff flags:
 *     TX_OVERFLOW    sends_fail > 0
 *     RX_NEAR_FULL   rx_depth_max > 400
 *     LATENCY_SPIKE  avg ctrl_us > 3 × stage-0 baseline
 *
 * ── Memory/stack snapshot: at start and at each stage boundary ────────────────
 *
 * Board: ESP32-S3
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/uart_bridge.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/uart.h"
#include <string.h>
#include <math.h>

static const char *TAG = "TEST_UART";

/* ── UART port — must match uart_bridge.c (from hardware_pins.h) ─────────── */
#ifndef BRIDGE_UART_PORT
#  include "../../../hardware_pins.h"
#endif

/* ── Stage definitions ───────────────────────────────────────────────────── */
#define N_STAGES 7

typedef struct {
    uint32_t hz;        /* target send rate, 0 = burst (no delay) */
    uint32_t n_packets; /* total packets per stage                 */
} stage_def_t;

static const stage_def_t k_stages[N_STAGES] = {
    {   10,   50 },
    {   20,  100 },
    {   50,  250 },
    {  100,  500 },
    {  200, 1000 },
    {  500, 2000 },
    {    0, 1000 }, /* BURST */
};

/* Every N-th packet is a path_frame (large) — triggers wire saturation sooner */
#define PATH_EVERY_N  10u

/* RX near-full warning threshold (S3 RX buf = 512 B) */
#define RX_NEAR_FULL_THRESH  400u

/* ── Per-stage statistics ────────────────────────────────────────────────── */
typedef struct {
    uint32_t ctrl_sends_ok;
    uint32_t ctrl_sends_fail;
    uint32_t path_sends_ok;
    uint32_t path_sends_fail;
    uint32_t recvs_ok;

    uint32_t ctrl_us_min;
    uint32_t ctrl_us_max;
    float    ctrl_us_avg;

    uint32_t path_us_min;
    uint32_t path_us_max;
    float    path_us_avg;

    uint32_t rx_depth_max;
    uint32_t elapsed_ms;
} stage_stat_t;

static stage_stat_t s_stats[N_STAGES];
static float        s_baseline_ctrl_us = 0.0f; /* stage-0 avg, for spike detection */

/* ── Profiler task handle ────────────────────────────────────────────────── */
static task_profile_t s_profile;

task_profile_t *uart_bridge_task_get_profile(void) { return &s_profile; }

/* ── Build a dummy path frame with valid length ──────────────────────────── */
static void make_path_frame(path_frame_t *pf, uint32_t seq)
{
    pf->length   = 5u;
    pf->reserved = 0u;
    for (int i = 0; i < 5; i++) {
        pf->waypoints[i].x        = (float)(seq * 100 + i * 10);
        pf->waypoints[i].y        = 0.0f;
        pf->waypoints[i].theta    = 0.0f;
        pf->waypoints[i].v_target = 200.0f;
    }
}

/* ── Drain all pending odom packets; return count received ───────────────── */
static uint32_t drain_recv(uint32_t *out_recv_us)
{
    uint32_t count = 0;
    odom_t   odom;
    int64_t  t0 = esp_timer_get_time();

    while (uart_bridge_recv_odom(&odom)) {
        count++;
    }

    *out_recv_us = (uint32_t)(esp_timer_get_time() - t0);
    return count;
}

/* ── Log memory + stack snapshot ─────────────────────────────────────────── */
static void log_memory(const char *context)
{
    uint32_t heap_free   = esp_get_free_heap_size();
    uint32_t heap_min    = esp_get_minimum_free_heap_size();
    uint32_t largest_blk = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    uint32_t frag_pct    = heap_free > 0u
                           ? (uint32_t)(100u - largest_blk * 100u / heap_free) : 0u;
    uint32_t stack_free  = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

    ESP_LOGI(TAG, "=MEM= [%s] heap=%" PRIu32 "B  wm=%" PRIu32 "B"
             "  blk=%" PRIu32 "B  frag=%" PRIu32 "%%  stack=%" PRIu32 "B",
             context, heap_free, heap_min, largest_blk, frag_pct, stack_free);
}

/* ── Print stage summary and cliff warnings ──────────────────────────────── */
static void print_stage_summary(int si, const char *label)
{
    const stage_stat_t *s = &s_stats[si];
    uint32_t total_sends = s->ctrl_sends_ok + s->ctrl_sends_fail
                         + s->path_sends_ok + s->path_sends_fail;
    uint32_t actual_hz = s->elapsed_ms > 0u
                         ? (uint32_t)((s->ctrl_sends_ok + s->path_sends_ok)
                                      * 1000u / s->elapsed_ms)
                         : 0u;

    ESP_LOGI(TAG,
             "══ STAGE %s SUMMARY ══  actual=%" PRIu32 "Hz"
             "  total=%"PRIu32"  ctrl_ok=%"PRIu32"  ctrl_fail=%"PRIu32
             "  path_ok=%"PRIu32"  path_fail=%"PRIu32
             "  odom_rx=%"PRIu32,
             label, actual_hz, total_sends,
             s->ctrl_sends_ok, s->ctrl_sends_fail,
             s->path_sends_ok, s->path_sends_fail,
             s->recvs_ok);

    ESP_LOGI(TAG,
             "  ctrl_us  min=%"PRIu32"  max=%"PRIu32"  avg=%.1f",
             s->ctrl_us_min == UINT32_MAX ? 0u : s->ctrl_us_min,
             s->ctrl_us_max,
             (double)s->ctrl_us_avg);

    if (s->path_sends_ok + s->path_sends_fail > 0u) {
        ESP_LOGI(TAG,
                 "  path_us  min=%"PRIu32"  max=%"PRIu32"  avg=%.1f",
                 s->path_us_min == UINT32_MAX ? 0u : s->path_us_min,
                 s->path_us_max,
                 (double)s->path_us_avg);
    }

    ESP_LOGI(TAG, "  rx_depth_max=%" PRIu32 "B  (limit=512B)", s->rx_depth_max);

    /* ── Cliff flags ─────────────────────────────────────────────────────── */
    bool cliff = false;

    if (s->ctrl_sends_fail > 0u || s->path_sends_fail > 0u) {
        ESP_LOGW(TAG,
                 "  *** TX_OVERFLOW: ctrl_fail=%"PRIu32"  path_fail=%"PRIu32
                 " — uart_write_bytes() returned short at %s Hz",
                 s->ctrl_sends_fail, s->path_sends_fail, label);
        cliff = true;
    }
    if (s->rx_depth_max > RX_NEAR_FULL_THRESH) {
        ESP_LOGW(TAG,
                 "  *** RX_NEAR_FULL: rx_depth_max=%" PRIu32
                 "B > 400B — S3 RX buffer filling fast at %s Hz",
                 s->rx_depth_max, label);
        cliff = true;
    }
    if (s_baseline_ctrl_us > 0.0f &&
        s->ctrl_us_avg > s_baseline_ctrl_us * 3.0f) {
        ESP_LOGW(TAG,
                 "  *** LATENCY_SPIKE: ctrl_avg=%.1f µs > 3× baseline=%.1f µs"
                 " — UART driver stalling at %s Hz",
                 (double)s->ctrl_us_avg, (double)s_baseline_ctrl_us, label);
        cliff = true;
    }
    if (!cliff) {
        ESP_LOGI(TAG, "  [OK] No cliff detected at %s Hz", label);
    }
}

/* ── Rate-limited delay: yield most of the wait, spin the tail ──────────── */
static void precise_wait_until(int64_t target_us)
{
    int64_t remaining;
    while ((remaining = target_us - esp_timer_get_time()) > 1000LL) {
        vTaskDelay(1);
    }
    /* Spin the final <1 ms — keeps us off CPU most of the time */
    while (esp_timer_get_time() < target_us) {}
}

/* ── Main task ───────────────────────────────────────────────────────────── */
void uart_bridge_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "uart_bridge_test",
                      6144,
                      4,
                      1);   /* core 1 — UART I/O */

    task_data_profile_init(&s_profile.data_profile,
                           "control_frame_t",
                           sizeof(control_frame_t),
                           false, false, false,
                           "wemos_uart");

    /* ── Static RAM breakdown ────────────────────────────────────────────── */
    ESP_LOGI(TAG, "=== STATIC RAM ===");
    ESP_LOGI(TAG, "  control_frame_t  %u B  → UART packet %u B (wire %.2f ms)",
             (unsigned)sizeof(control_frame_t),
             (unsigned)(sizeof(control_frame_t) + 5u),
             (double)(sizeof(control_frame_t) + 5u) * 10.0 / 115200.0 * 1000.0);
    ESP_LOGI(TAG, "  path_frame_t     %u B  → UART packet %u B (wire %.2f ms)",
             (unsigned)sizeof(path_frame_t),
             (unsigned)(sizeof(path_frame_t) + 5u),
             (double)(sizeof(path_frame_t) + 5u) * 10.0 / 115200.0 * 1000.0);
    ESP_LOGI(TAG, "  odom_t (reply)   %u B  → UART packet %u B (wire %.2f ms)",
             (unsigned)sizeof(odom_t),
             (unsigned)(sizeof(odom_t) + 5u),
             (double)(sizeof(odom_t) + 5u) * 10.0 / 115200.0 * 1000.0);
    ESP_LOGI(TAG, "  Wire capacity:   11520 B/s @115200 baud");
    ESP_LOGI(TAG, "  S3 RX buf:       512 B  → ~%u odom pkts before overflow",
             (unsigned)(512u / (sizeof(odom_t) + 5u)));
    ESP_LOGI(TAG, "  Mixed cliff est: ~264 Hz (90%% ctrl + 10%% path)");

    log_memory("init");
    uart_bridge_init();
    vTaskDelay(pdMS_TO_TICKS(100));   /* let UART settle */
    log_memory("post-uart-init");

    control_frame_t ctrl = {
        .tx       = 100.0f,
        .ty       = 0.0f,
        .t_heading = 0.0f,
        .t_speed  = 200.0f,
    };
    path_frame_t pf;
    uint32_t seq = 0u;

    ESP_LOGI(TAG,
             "──────────────────────────────────────────────────────────────");
    ESP_LOGI(TAG, "UART stress test begin — %d stages", N_STAGES);
    ESP_LOGI(TAG, "Connect Wemos running wemos profiler with ENABLE_UART_ECHO=1");
    ESP_LOGI(TAG,
             "──────────────────────────────────────────────────────────────");

    for (int si = 0; si < N_STAGES; si++) {
        const stage_def_t *stage = &k_stages[si];
        char label[16];
        if (stage->hz == 0u)
            (void)snprintf(label, sizeof(label), "BURST");
        else
            (void)snprintf(label, sizeof(label), "%" PRIu32, stage->hz);

        /* ── Init per-stage stats ─────────────────────────────────────────── */
        stage_stat_t *s = &s_stats[si];
        s->ctrl_sends_ok   = 0u;
        s->ctrl_sends_fail = 0u;
        s->path_sends_ok   = 0u;
        s->path_sends_fail = 0u;
        s->recvs_ok        = 0u;
        s->ctrl_us_min     = UINT32_MAX;
        s->ctrl_us_max     = 0u;
        s->ctrl_us_avg     = 0.0f;
        s->path_us_min     = UINT32_MAX;
        s->path_us_max     = 0u;
        s->path_us_avg     = 0.0f;
        s->rx_depth_max    = 0u;
        s->elapsed_ms      = 0u;

        int64_t period_us = (stage->hz > 0u) ? (1000000LL / (int64_t)stage->hz) : 0LL;

        ESP_LOGI(TAG, "── Stage %d: %s Hz  packets=%"PRIu32"  period=%" PRId64 " µs",
                 si, label, stage->n_packets, period_us);

        int64_t stage_start = esp_timer_get_time();
        int64_t next_send   = stage_start;

        uint32_t ctrl_count_for_avg = 0u;
        uint32_t path_count_for_avg = 0u;

        task_profile_cycle_begin(&s_profile);

        for (uint32_t pkt = 0u; pkt < stage->n_packets; pkt++) {

            /* ── Rate control ─────────────────────────────────────────────── */
            if (period_us > 0LL) {
                precise_wait_until(next_send);
                next_send += period_us;
            }

            /* ── Check RX depth before drain ─────────────────────────────── */
            size_t rx_avail = 0u;
            uart_get_buffered_data_len(BRIDGE_UART_PORT, &rx_avail);
            if ((uint32_t)rx_avail > s->rx_depth_max)
                s->rx_depth_max = (uint32_t)rx_avail;

            /* ── Send: every PATH_EVERY_N is a large path frame ──────────── */
            bool use_path = (pkt % PATH_EVERY_N == PATH_EVERY_N - 1u);

            if (use_path) {
                make_path_frame(&pf, seq);
                PROFILE_CPU_BEGIN(path_send);
                bool ok = uart_bridge_send_path(&pf);
                uint32_t path_us;
                PROFILE_CPU_END(path_send, &path_us);

                if (ok) {
                    s->path_sends_ok++;
                    if (path_us < s->path_us_min) s->path_us_min = path_us;
                    if (path_us > s->path_us_max) s->path_us_max = path_us;
                    path_count_for_avg++;
                    s->path_us_avg += ((float)path_us - s->path_us_avg)
                                      / (float)path_count_for_avg;
                } else {
                    s->path_sends_fail++;
                    ESP_LOGW(TAG, "[%s] path send FAILED pkt=%"PRIu32, label, pkt);
                }
            } else {
                ctrl.tx = (float)(seq * 10u);
                PROFILE_CPU_BEGIN(ctrl_send);
                bool ok = uart_bridge_send_control(&ctrl);
                uint32_t ctrl_us;
                PROFILE_CPU_END(ctrl_send, &ctrl_us);

                if (ok) {
                    s->ctrl_sends_ok++;
                    if (ctrl_us < s->ctrl_us_min) s->ctrl_us_min = ctrl_us;
                    if (ctrl_us > s->ctrl_us_max) s->ctrl_us_max = ctrl_us;
                    ctrl_count_for_avg++;
                    s->ctrl_us_avg += ((float)ctrl_us - s->ctrl_us_avg)
                                      / (float)ctrl_count_for_avg;
                } else {
                    s->ctrl_sends_fail++;
                    ESP_LOGW(TAG, "[%s] ctrl send FAILED pkt=%"PRIu32, label, pkt);
                }
            }

            seq++;

            /* ── Drain RX every packet ───────────────────────────────────── */
            uint32_t recv_us = 0u;
            s->recvs_ok += drain_recv(&recv_us);

            /* Per-packet log (first 5 and every 100 thereafter) */
            if (pkt < 5u || pkt % 100u == 0u) {
                ESP_LOGI(TAG,
                         "  [%s] pkt=%4"PRIu32"  %s_ok=%c"
                         "  ctrl_avg=%.1fµs  path_avg=%.1fµs"
                         "  rx_depth=%uB  odom_rx=%"PRIu32,
                         label, pkt,
                         use_path ? "path" : "ctrl",
                         use_path
                             ? (s->path_sends_fail == 0u ? 'Y' : 'N')
                             : (s->ctrl_sends_fail == 0u ? 'Y' : 'N'),
                         (double)s->ctrl_us_avg,
                         (double)s->path_us_avg,
                         (unsigned)rx_avail,
                         s->recvs_ok);
            }
        }

        task_profile_cycle_end(&s_profile);

        int64_t elapsed_us = esp_timer_get_time() - stage_start;
        s->elapsed_ms = (uint32_t)(elapsed_us / 1000LL);

        /* Save baseline from stage 0 for latency spike detection */
        if (si == 0 && s->ctrl_us_avg > 0.0f)
            s_baseline_ctrl_us = s->ctrl_us_avg;

        /* ── Drain any remaining RX from this stage ───────────────────────── */
        uint32_t drain_us = 0u;
        uint32_t drained = drain_recv(&drain_us);
        s->recvs_ok += drained;
        if (drained > 0u) {
            ESP_LOGI(TAG, "  [%s] post-stage drain: %"PRIu32" odom pkts in %"PRIu32"µs",
                     label, drained, drain_us);
        }

        task_data_profile_update(&s_profile.data_profile,
                                 s->ctrl_sends_ok + s->path_sends_ok);

        print_stage_summary(si, label);
        log_memory(label);

        /* ── Inter-stage flush ────────────────────────────────────────────── */
        if (si < N_STAGES - 1) {
            ESP_LOGI(TAG, "  [%s] inter-stage flush 500 ms …", label);
            vTaskDelay(pdMS_TO_TICKS(500));
            /* Drain anything that arrived during flush */
            drain_us = 0u;
            drained  = drain_recv(&drain_us);
            if (drained > 0u)
                ESP_LOGI(TAG, "  flush drained %"PRIu32" odom pkts", drained);
        }
    }

    /* ── Final report ────────────────────────────────────────────────────── */
    ESP_LOGI(TAG,
             "══════════════ UART STRESS COMPLETE ══════════════");
    ESP_LOGI(TAG, "%-8s  %6s  %6s  %8s  %8s  %8s  %5s  %5s",
             "Stage", "Hz", "Sent", "CtrlFail", "PathFail",
             "OdomRx", "CTavg", "RXmax");
    for (int si = 0; si < N_STAGES; si++) {
        const stage_stat_t *s = &s_stats[si];
        char label[16];
        if (k_stages[si].hz == 0u)
            (void)snprintf(label, sizeof(label), "BURST");
        else
            (void)snprintf(label, sizeof(label), "%" PRIu32, k_stages[si].hz);

        uint32_t actual_hz = s->elapsed_ms > 0u
            ? (s->ctrl_sends_ok + s->path_sends_ok) * 1000u / s->elapsed_ms
            : 0u;

        ESP_LOGI(TAG, "%-8s  %6"PRIu32"  %6"PRIu32"  %8"PRIu32"  %8"PRIu32
                 "  %8"PRIu32"  %5.1f  %5"PRIu32,
                 label, actual_hz,
                 s->ctrl_sends_ok + s->path_sends_ok,
                 s->ctrl_sends_fail, s->path_sends_fail,
                 s->recvs_ok,
                 (double)s->ctrl_us_avg,
                 s->rx_depth_max);
    }
    ESP_LOGI(TAG, "══════════════════════════════════════════════════");

    /* ── Stay alive for monitor_task to dump profiles ────────────────────── */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "(stress complete — idle)");
    }
}
