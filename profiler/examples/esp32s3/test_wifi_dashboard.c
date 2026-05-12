/**
 * test_wifi_dashboard.c — Detailed WiFi dashboard profiling.
 *
 * Profiles every public API individually:
 *   broadcast_state, broadcast_scan, broadcast_path, log, update (map)
 *
 * Reports every cycle:
 *   - µs per API call
 *   - queue depth before/after the burst (detects consumer backpressure)
 *   - messages queued / dropped this cycle
 *
 * Reports every 5 cycles:
 *   - free heap (current + watermark)
 *   - largest contiguous free block (fragmentation indicator)
 *   - test task stack high-water mark
 *
 * Reports every 20 cycles (if PROFILER_USE_PERFMON is defined):
 *   - Xtensa perfmon: IPC, D-cache stall%, I-cache stall%, branch mispred%
 *     measured over wifi_dashboard_broadcast_scan() — the most compute-heavy call
 *
 * Static RAM breakdown logged once at init:
 *   s_map_buf      2517 B   (50×50 map frame)
 *   s_pose_buf       24 B
 *   s_scan_buf      363 B   (3 + 90×4)
 *   s_path_buf      122 B   (2 + 15×8)
 *   s_log_bufs      400 B   (5 × 80)
 *   s_last_cells   2500 B   (delta shadow)
 *   s_new_cells    2500 B   (current sample)
 *   s_scan_pts      360 B   (90 × 4)
 *   dash_msg_t queue = 24 × sizeof(dash_msg_t)
 *
 * REQUIRES HARDWARE: WiFi AP.
 * Without AP, wifi_dashboard_init() fails after 10 s — all API calls no-op safely.
 *
 * WiFi credentials: set at build time via platformio.ini build_flags:
 *   -DTEST_WIFI_SSID='"MySSID"'
 *   -DTEST_WIFI_PASS='"MyPass"'
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/wifi_dashboard.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

static const char *TAG = "TEST_WIFI";

#ifndef TEST_WIFI_SSID
#  define TEST_WIFI_SSID "SPOT-iot"
#endif
#ifndef TEST_WIFI_PASS
#  define TEST_WIFI_PASS "RacailleSalutaireMigration8052"
#endif

/* Map geometry — matches production settings */
#define MAP_W    10000.0f
#define MAP_H    10000.0f
#define MAP_STEP   200.0f
#define ROBOT_X   5000.0f
#define ROBOT_Y   5000.0f

/* Synthetic scan: 360 points, realistic range variation */
#define N_SCAN_PTS 360

static task_profile_t  s_profile;
static lidar_scan_t    s_scan;    /* ~5.5 KB static */
static quadtree_map_t  s_map;

/* ── Synthetic scan (used to stress broadcast_scan encode/downsample path) ── */
static void build_scan(lidar_scan_t *scan, uint32_t tick)
{
    scan->count             = N_SCAN_PTS;
    scan->scan_start_us     = (uint32_t)esp_timer_get_time();
    scan->rotation_period_us = 100000;
    for (int i = 0; i < N_SCAN_PTS; i++) {
        scan->points[i].theta_deg  = (float)i;
        scan->points[i].r_mm       = 1500.0f + 800.0f * sinf((float)(i + tick) * 0.04f);
        scan->points[i].intensity  = 180;
    }
}

/* ── Synthetic path — 15 waypoints in a circle, used to test broadcast_path ── */
static void build_path(path_frame_t *frame)
{
    frame->length = MAX_SHARED_PATH_POINTS;
    for (uint8_t i = 0; i < frame->length; i++) {
        float a = (float)i * (6.2832f / (float)frame->length);
        frame->waypoints[i].x        = ROBOT_X + 800.0f * cosf(a);
        frame->waypoints[i].y        = ROBOT_Y + 800.0f * sinf(a);
        frame->waypoints[i].theta    = a;
        frame->waypoints[i].v_target = 300.0f;
    }
}

#ifdef PROFILER_USE_PERFMON
/* Wrapper for perfmon: measures only broadcast_scan ── */
typedef struct { lidar_scan_t *scan; pose_t *pose; } scan_work_t;
static void _scan_work(void *arg)
{
    scan_work_t *w = (scan_work_t *)arg;
    wifi_dashboard_broadcast_scan(w->scan, w->pose);
}
#endif

task_profile_t *wifi_dashboard_task_get_profile(void) { return &s_profile; }

void wifi_dashboard_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "wifi_dashboard_test",
                      8192,
                      2,    /* priority — below SLAM (5/4/3), above idle */
                      0);   /* core 0 — WiFi stack lives on core 0 */

    /* One "item" = one message posted to the dash queue this cycle */
    task_data_profile_init(&s_profile.data_profile,
                           "dash_msg_t",
                           sizeof(pose_t),   /* representative: pose msg */
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    /* ── Static RAM breakdown (logged once) ──────────────────────────────── */
    ESP_LOGI(TAG, "=== STATIC RAM BREAKDOWN ===");
    ESP_LOGI(TAG, "  s_map_buf      2517 B  (50x50 map frame)");
    ESP_LOGI(TAG, "  s_pose_buf       24 B");
    ESP_LOGI(TAG, "  s_scan_buf      363 B  (3 + 90x4)");
    ESP_LOGI(TAG, "  s_path_buf      122 B  (2 + 15x8)");
    ESP_LOGI(TAG, "  s_log_bufs      400 B  (5 x 80)");
    ESP_LOGI(TAG, "  s_last_cells   2500 B  (delta shadow)");
    ESP_LOGI(TAG, "  s_new_cells    2500 B  (current sample)");
    ESP_LOGI(TAG, "  s_scan_pts      360 B  (90 x 4)");
    /* dash_msg_t: 4(type)+4(dirty)+union{124(path)|80(log)|22(pose)} = ~132 B */
    ESP_LOGI(TAG, "  dash_msg_t queue 24 slots x ~132 B = ~3168 B");
    ESP_LOGI(TAG, "  TOTAL STATIC ~%u B",
             (unsigned)(2517u + 24u + 363u + 122u + 400u + 2500u + 2500u + 360u + 3168u));

    /* ── Heap before WiFi init ─────────────────────────────────────────────── */
    uint32_t heap_before_init = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Heap before wifi_dashboard_init: %" PRIu32 " B", heap_before_init);

    /* ── Seed map ─────────────────────────────────────────────────────────── */
    quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);
    for (float dx = -2000.0f; dx <= 2000.0f; dx += 200.0f)
        for (float dy = -2000.0f; dy <= 2000.0f; dy += 200.0f)
            quadtree_map_insert(&s_map, ROBOT_X + dx, ROBOT_Y + dy, CLASS_FREE);
    /* Ring of obstacles so map delta is non-trivial */
    for (int i = 0; i < 72; i++) {
        float a = (float)i * 5.0f * (3.14159f / 180.0f);
        quadtree_map_insert(&s_map,
                            ROBOT_X + 2200.0f * cosf(a),
                            ROBOT_Y + 2200.0f * sinf(a),
                            CLASS_WALL);
    }

    /* ── WiFi init ─────────────────────────────────────────────────────────── */
    wifi_dashboard_init(TEST_WIFI_SSID, TEST_WIFI_PASS);

    uint32_t heap_after_init = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Heap after  wifi_dashboard_init: %" PRIu32 " B  (WiFi cost: %" PRIu32 " B)",
             heap_after_init, heap_before_init - heap_after_init);

    /* Register map with dashboard — must happen after init */
    pose_t robot_pose = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };
    wifi_dashboard_update(&s_map, &robot_pose);

    path_frame_t path;
    build_path(&path);

    uint32_t tick = 0;

#ifdef PROFILER_USE_PERFMON
    task_perfmon_profile_t pm;
    task_perfmon_profile_init(&pm, "broadcast_scan", -1, 1);
    scan_work_t scan_work = { .scan = &s_scan, .pose = &robot_pose };
#endif

    while (1) {
        task_profile_cycle_begin(&s_profile);

        build_scan(&s_scan, tick);

        /* Simulate slow robot drift */
        robot_pose.theta = (float)tick * 0.05f;

        uint8_t qdepth_before = wifi_dashboard_queue_depth();

        /* ── broadcast_state ───────────────────────────────────────────────── */
        float fx = ROBOT_X + 1200.0f * cosf((float)tick * 0.1f);
        float fy = ROBOT_Y + 1200.0f * sinf((float)tick * 0.1f);

        PROFILE_CPU_BEGIN(state);
        wifi_dashboard_broadcast_state(&robot_pose, fx, fy, true,
                                       (uint16_t)(tick % 65536u));
        uint32_t state_us;
        PROFILE_CPU_END(state, &state_us);

        /* ── broadcast_scan ────────────────────────────────────────────────── */
        PROFILE_CPU_BEGIN(scan);
        wifi_dashboard_broadcast_scan(&s_scan, &robot_pose);
        uint32_t scan_us;
        PROFILE_CPU_END(scan, &scan_us);

        /* ── broadcast_path ────────────────────────────────────────────────── */
        PROFILE_CPU_BEGIN(path_call);
        wifi_dashboard_broadcast_path(&path);
        uint32_t path_us;
        PROFILE_CPU_END(path_call, &path_us);

        /* ── wifi_dashboard_log ────────────────────────────────────────────── */
        char log_buf[64];
        snprintf(log_buf, sizeof(log_buf), "tick=%" PRIu32 " theta=%.2f",
                 tick, (double)robot_pose.theta);
        PROFILE_CPU_BEGIN(log_call);
        wifi_dashboard_log(log_buf);
        uint32_t log_us;
        PROFILE_CPU_END(log_call, &log_us);

        /* ── wifi_dashboard_update (map) — 1 Hz ───────────────────────────── */
        uint32_t update_us = 0;
        if (tick % 10u == 0u) {
            PROFILE_CPU_BEGIN(map_update);
            wifi_dashboard_update(&s_map, &robot_pose);
            PROFILE_CPU_END(map_update, &update_us);
        }

        uint8_t qdepth_after = wifi_dashboard_queue_depth();
        uint8_t msgs_queued  = (qdepth_after > qdepth_before)
                               ? (qdepth_after - qdepth_before) : 0u;

        task_data_profile_update(&s_profile.data_profile, msgs_queued);
        task_profile_cycle_end(&s_profile);

        /* ── Per-cycle timing log ──────────────────────────────────────────── */
        if (qdepth_after > 12u) {
            ESP_LOGW(TAG,
                     "[BACKPRESSURE] qdepth=%u/24 — dash_task lagging",
                     qdepth_after);
        }

        ESP_LOGI(TAG,
                 "tick=%-4" PRIu32
                 "  state=%4" PRIu32 "µs"
                 "  scan=%5" PRIu32 "µs"
                 "  path=%4" PRIu32 "µs"
                 "  log=%3" PRIu32 "µs"
                 "%s"
                 "  qdepth=%u→%u  queued=%u",
                 tick,
                 state_us, scan_us, path_us, log_us,
                 update_us ? "  [MAP_UPDATE]" : "",
                 qdepth_before, qdepth_after, msgs_queued);

        /* ── Memory snapshot every 5 cycles ───────────────────────────────── */
        if (tick % 5u == 0u) {
            uint32_t heap_free    = esp_get_free_heap_size();
            uint32_t heap_min     = esp_get_minimum_free_heap_size();
            uint32_t largest_blk  = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
            uint32_t stack_hwm    = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

            ESP_LOGI(TAG,
                     "=MEM= heap_free=%" PRIu32 "B  watermark=%" PRIu32 "B"
                     "  largest_blk=%" PRIu32 "B  frag=%u%%"
                     "  stack_free=%" PRIu32 "B",
                     heap_free, heap_min, largest_blk,
                     (heap_free > 0u) ? (uint32_t)(100u - largest_blk * 100u / heap_free) : 0u,
                     stack_hwm);
        }

#ifdef PROFILER_USE_PERFMON
        /* ── Perfmon on broadcast_scan every 20 cycles ─────────────────────── */
        if (tick % 20u == 0u) {
            task_perfmon_collect(&pm, _scan_work, &scan_work, -1, 1);
            ESP_LOGI(TAG, "=PERFMON broadcast_scan=");
            task_perfmon_profile_dump(&pm);
        }
#endif

        tick++;
        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}
