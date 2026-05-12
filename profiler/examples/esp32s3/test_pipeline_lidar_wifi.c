/**
 * test_pipeline_lidar_wifi.c — Real LiDAR → QuadTree Map → WiFi Dashboard pipeline.
 *
 * Two tasks running concurrently, mirroring the production ESP32-S3 architecture:
 *
 *   lidar_wifi_scan_task  (core 1, priority 5):
 *     lidar_driver_read_scan() → lidar_to_map() → mark_dirty() → broadcast_scan()
 *
 *   lidar_wifi_dash_task  (core 0, priority 2):
 *     wifi_dashboard_update() + broadcast_state() driven by scan_task rate
 *
 *   _dash_task (internal, priority 1):
 *     spawned by wifi_dashboard_init() — drains queue, sends WebSocket frames
 *
 * REQUIRES HARDWARE:
 *   - RPLiDAR C1 on UART (GPIO14 RX, GPIO13 TX, 460800 baud)
 *   - WiFi AP reachable at compile-time credentials
 *
 * ── Per-cycle output (scan_task, ~7–10 Hz driven by LiDAR RPM) ─────────────
 *   scan_us     lidar_driver_read_scan() wall time (~132 ms nominal)
 *   mutex_us    time blocked waiting for g_map_mutex
 *   l2m_us      lidar_to_map() ray-march compute time
 *   dirty_us    wifi_dashboard_mark_dirty() — mutex acquire only (~µs)
 *   bcast_us    wifi_dashboard_broadcast_scan() — downsample + queue post
 *
 * ── Per-cycle output (dash_task, 10 Hz) ─────────────────────────────────────
 *   update_us   wifi_dashboard_update() — tile dirty-rect compute + queue post
 *   state_us    wifi_dashboard_broadcast_state() — struct copy + queue post
 *   qdepth      dash queue fill before/after (>12/24 = backpressure)
 *
 * ── Memory snapshot (every 5 scan cycles) ───────────────────────────────────
 *   heap_free, heap_watermark, largest_block, frag%, each task stack HWM
 *
 * ── Perfmon (every 20 scan cycles, requires -DPROFILER_USE_PERFMON) ─────────
 *   IPC, D-cache stall%, I-cache stall%, branch mispred% measured on lidar_to_map()
 *
 * ── FreeRTOS task table (every 10 s via monitor_task) ───────────────────────
 *   vTaskGetRunTimeStats() prints CPU% per task — stalls visible as low%
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/lidar_driver.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/lidar_to_map.h"
#include "../../../esp32s3/src/wifi_dashboard.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <math.h>

static const char *TAG_SCAN = "LW_SCAN";
static const char *TAG_DASH = "LW_DASH";

#ifndef TEST_WIFI_SSID
#  define TEST_WIFI_SSID "SPOT-iot"
#endif
#ifndef TEST_WIFI_PASS
#  define TEST_WIFI_PASS "RacailleSalutaireMigration8052"
#endif

#define MAP_W        10000.0f
#define MAP_H        10000.0f
#define MAP_STEP       200.0f
#define MAX_RANGE_MM  4000.0f
#define RAY_STEP_MM    100.0f
#define ROBOT_X       5000.0f
#define ROBOT_Y       5000.0f

/* ── Shared state between the two tasks ─────────────────────────────────── */
static SemaphoreHandle_t g_map_mutex    = NULL;
static quadtree_map_t    g_map;                 /* 96 KB pool — static */
static pose_t            g_pose         = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };
static volatile bool     g_ready        = false; /* set by scan_task after WiFi+LiDAR init */

/* ── Per-task profiles ───────────────────────────────────────────────────── */
static task_profile_t s_scan_profile;
static task_profile_t s_dash_profile;

task_profile_t *lidar_wifi_scan_task_get_profile(void) { return &s_scan_profile; }
task_profile_t *lidar_wifi_dash_task_get_profile(void) { return &s_dash_profile; }

/* ── Memory snapshot helper ──────────────────────────────────────────────── */
static void _log_memory(const char *tag)
{
    uint32_t heap_free   = esp_get_free_heap_size();
    uint32_t heap_min    = esp_get_minimum_free_heap_size();
    uint32_t largest_blk = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    uint32_t frag_pct    = (heap_free > 0u)
                           ? (uint32_t)(100u - largest_blk * 100u / heap_free)
                           : 0u;
    uint32_t stack_free  = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

    ESP_LOGI(tag,
             "=MEM= heap=%" PRIu32 "B  wm=%" PRIu32 "B"
             "  blk=%" PRIu32 "B  frag=%" PRIu32 "%%  stack=%" PRIu32 "B",
             heap_free, heap_min, largest_blk, frag_pct, stack_free);
}

#ifdef PROFILER_USE_PERFMON
/* Perfmon wrapper: measures lidar_to_map() only ─────────────────────────── */
typedef struct {
    quadtree_map_t   *map;
    lidar_scan_t     *scan;
    pose_t           *pose;
    map_dirty_rect_t *dirty;
} l2m_work_t;

static void _l2m_work(void *arg)
{
    l2m_work_t *w = (l2m_work_t *)arg;
    lidar_to_map(w->map, w->scan, w->pose, MAX_RANGE_MM, RAY_STEP_MM, w->dirty);
}
#endif


/* ════════════════════════════════════════════════════════════════════════════
 * lidar_wifi_scan_task — core 1, priority 5
 *
 * Mirrors production scan_task:
 *   read scan → integrate map → mark dirty → broadcast scan to dashboard
 * ════════════════════════════════════════════════════════════════════════════ */
void lidar_wifi_scan_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_scan_profile, "lw_scan",
                      10240,  /* 10 KB — lidar_scan_t is 5.5 KB on stack risk */
                      5,      /* priority — must not be starved by WiFi tasks */
                      1);     /* core 1 — UART I/O */

    task_data_profile_init(&s_scan_profile.data_profile,
                           "lidar_scan_point_t",
                           sizeof(lidar_scan_point_t),
                           false, false, false, NULL);

    /* Init WiFi + HTTP server first — this blocks up to 10 s */
    ESP_LOGI(TAG_SCAN, "Connecting to WiFi …");
    wifi_dashboard_init(TEST_WIFI_SSID, TEST_WIFI_PASS);
    ESP_LOGI(TAG_SCAN, "WiFi ready. Initialising LiDAR …");

    lidar_driver_init();

    g_map_mutex = xSemaphoreCreateMutex();
    configASSERT(g_map_mutex);
    quadtree_map_init(&g_map, MAP_W, MAP_H, MAP_STEP);

    /* Signal dash_task that init is complete */
    g_ready = true;
    ESP_LOGI(TAG_SCAN, "Pipeline running — open live_dashboard.html and connect to ws://<ip>/ws");

    static lidar_scan_t s_scan;   /* ~5.5 KB — static to avoid stack overflow */

#ifdef PROFILER_USE_PERFMON
    task_perfmon_profile_t pm;
    task_perfmon_profile_init(&pm, "lidar_to_map", -1, 1);
    map_dirty_rect_t pm_dirty = {0};
    l2m_work_t pm_work = { &g_map, &s_scan, &g_pose, &pm_dirty };
#endif

    uint32_t scan_count = 0;

    while (1) {
        task_profile_cycle_begin(&s_scan_profile);

        /* ── Stage 1: acquire real LiDAR scan (blocking ~132 ms) ──────────── */
        PROFILE_CPU_BEGIN(scan_read);
        bool ok = lidar_driver_read_scan(&s_scan);
        uint32_t scan_us;
        PROFILE_CPU_END(scan_read, &scan_us);

        if (!ok) {
            task_data_profile_update(&s_scan_profile.data_profile, 0);
            task_profile_cycle_end(&s_scan_profile);
            ESP_LOGW(TAG_SCAN, "lidar_driver_read_scan() failed — no hardware?");
            continue;
        }

        /* ── Stage 2: integrate scan into shared map (under mutex) ─────────── */
        PROFILE_CPU_BEGIN(mutex_wait);
        xSemaphoreTake(g_map_mutex, portMAX_DELAY);
        uint32_t mutex_us;
        PROFILE_CPU_END(mutex_wait, &mutex_us);

        map_dirty_rect_t dirty = {0};

        PROFILE_CPU_BEGIN(l2m);
        lidar_to_map(&g_map, &s_scan, &g_pose, MAX_RANGE_MM, RAY_STEP_MM, &dirty);
        uint32_t l2m_us;
        PROFILE_CPU_END(l2m, &l2m_us);

        uint16_t map_nodes = g_map.count;
        bool pool_full     = qt_is_pool_full(&g_map);
        xSemaphoreGive(g_map_mutex);

        /* ── Stage 3: push dirty rect + scan to dashboard ───────────────────── */
        PROFILE_CPU_BEGIN(mark_dirty);
        wifi_dashboard_mark_dirty(&dirty);
        uint32_t dirty_us;
        PROFILE_CPU_END(mark_dirty, &dirty_us);

        PROFILE_CPU_BEGIN(bcast_scan);
        wifi_dashboard_broadcast_scan(&s_scan, &g_pose);
        uint32_t bcast_us;
        PROFILE_CPU_END(bcast_scan, &bcast_us);

        task_data_profile_update(&s_scan_profile.data_profile, s_scan.count);
        task_profile_cycle_end(&s_scan_profile);

        /* ── Cycle log ─────────────────────────────────────────────────────── */
        uint32_t total_us = s_scan_profile.cpu_cycles_last / PROFILER_CPU_FREQ_MHZ;
        if (total_us > 200000u) {
            ESP_LOGW(TAG_SCAN, "SLOW CYCLE %" PRIu32 " µs (budget 200 ms)",
                     total_us);
        }
        if (mutex_us > 1000u) {
            ESP_LOGW(TAG_SCAN, "MUTEX STALL %" PRIu32 " µs — dash_task holding map?",
                     mutex_us);
        }

        ESP_LOGI(TAG_SCAN,
                 "scan=%5" PRIu32 "µs  mutex=%4" PRIu32 "µs"
                 "  l2m=%5" PRIu32 "µs  dirty=%3" PRIu32 "µs"
                 "  bcast=%4" PRIu32 "µs"
                 "  pts=%u  nodes=%u  total=%5" PRIu32 "µs",
                 scan_us, mutex_us, l2m_us, dirty_us, bcast_us,
                 s_scan.count, map_nodes, total_us);

        /* ── Memory snapshot every 5 scans ─────────────────────────────────── */
        scan_count++;
        if (scan_count % 5u == 0u) {
            _log_memory(TAG_SCAN);
        }

#ifdef PROFILER_USE_PERFMON
        /* ── Perfmon on lidar_to_map every 20 scans ─────────────────────────── */
        if (scan_count % 20u == 0u) {
            /* Run lidar_to_map once more under perfmon (results discarded from map) */
            task_perfmon_collect(&pm, _l2m_work, &pm_work, -1, 1);
            ESP_LOGI(TAG_SCAN, "=PERFMON lidar_to_map=");
            task_perfmon_profile_dump(&pm);
        }
#endif

        /* Reset map when pool approaches saturation */
        if (pool_full) {
            xSemaphoreTake(g_map_mutex, portMAX_DELAY);
            ESP_LOGW(TAG_SCAN, "map pool full (%u nodes) — resetting", g_map.count);
            qt_free(&g_map);
            quadtree_map_init(&g_map, MAP_W, MAP_H, MAP_STEP);
            xSemaphoreGive(g_map_mutex);
            wifi_dashboard_log("WARN: map pool full — map reset");
        }

        /* No vTaskDelay — lidar_driver_read_scan() is the natural rate limiter */
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * lidar_wifi_dash_task — core 0, priority 2
 *
 * Mirrors production plan_task's dashboard calls:
 *   wifi_dashboard_update() (1 Hz, internally throttled)
 *   wifi_dashboard_broadcast_state() (10 Hz)
 * ════════════════════════════════════════════════════════════════════════════ */
void lidar_wifi_dash_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_dash_profile, "lw_dash",
                      4096,
                      2,    /* priority — below scan (5), above _dash_task (1) */
                      0);   /* core 0 — WiFi stack lives on core 0 */

    task_data_profile_init(&s_dash_profile.data_profile,
                           "dash_msg_t",
                           sizeof(pose_t),
                           false, false, false, NULL);

    /* Wait for scan_task to finish WiFi + LiDAR init */
    while (!g_ready) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG_DASH, "dash_task running");

    uint32_t cycle = 0;

    while (1) {
        task_profile_cycle_begin(&s_dash_profile);

        uint8_t qdepth_before = wifi_dashboard_queue_depth();

        /* ── wifi_dashboard_update (map → tile resample → queue) ───────────── */
        PROFILE_CPU_BEGIN(map_update);
        wifi_dashboard_update(&g_map, &g_pose);
        uint32_t update_us;
        PROFILE_CPU_END(map_update, &update_us);

        /* ── wifi_dashboard_broadcast_state (pose → queue) ─────────────────── */
        float fx = ROBOT_X + 500.0f;   /* no frontier in this test */
        float fy = ROBOT_Y + 500.0f;

        PROFILE_CPU_BEGIN(state);
        wifi_dashboard_broadcast_state(&g_pose, fx, fy, false, 0u);
        uint32_t state_us;
        PROFILE_CPU_END(state, &state_us);

        uint8_t qdepth_after = wifi_dashboard_queue_depth();

        task_data_profile_update(&s_dash_profile.data_profile,
                                 (qdepth_after > qdepth_before)
                                 ? (uint32_t)(qdepth_after - qdepth_before) : 0u);
        task_profile_cycle_end(&s_dash_profile);

        /* ── Cycle log ─────────────────────────────────────────────────────── */
        if (qdepth_after > 12u) {
            ESP_LOGW(TAG_DASH,
                     "[BACKPRESSURE] qdepth=%u/24 — _dash_task cannot drain fast enough",
                     qdepth_after);
        }

        ESP_LOGI(TAG_DASH,
                 "update=%4" PRIu32 "µs  state=%3" PRIu32 "µs"
                 "  qdepth=%u→%u",
                 update_us, state_us, qdepth_before, qdepth_after);

        /* ── Memory snapshot every 5 cycles ────────────────────────────────── */
        cycle++;
        if (cycle % 5u == 0u) {
            _log_memory(TAG_DASH);
        }

        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}
