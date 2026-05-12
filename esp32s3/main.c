/**
 * esp32s3/main.c  —  Phase 1: LiDAR → Quadtree → Dashboard
 *
 * ════════════════════════════════════════════════════════════════════════════
 * WHAT THIS DOES
 * ════════════════════════════════════════════════════════════════════════════
 * One task on Core 0 (priority 7) runs the full sensing pipeline:
 *   1. Read one 360° LiDAR scan  (~100 ms, waits for one motor rotation)
 *   2. Ray-march the scan into the quadtree occupancy map  (~5 ms)
 *   3. Push the updated map + scan overlay to the browser dashboard
 *   4. Sleep 100 ms to yield CPU to WiFi / httpd / _dash_task
 *   Repeat.
 *
 * No odometry, no planning, no motors.  Just the sensing pipeline.
 *
 * ════════════════════════════════════════════════════════════════════════════
 * TASKS
 * ════════════════════════════════════════════════════════════════════════════
 *   task_lidar_slam   Core 0  prio 7   stack 6 KB
 *       The whole pipeline: scan → lidar_to_map → dashboard notify.
 *       Yields 100 ms after each scan so the WiFi stack gets CPU.
 *
 *   task_perf_mon     Core 0  prio 1   stack 4 KB
 *       Prints heap, stack watermarks, scan rate, l2m latency, and
 *       FreeRTOS per-task CPU % every 10 seconds.
 *
 *   _dash_task        Core 0  prio 1   stack 4 KB   (inside wifi_dashboard.c)
 *       Created by wifi_dashboard_init().  Sole WebSocket sender — drains the
 *       24-slot message queue and calls httpd_ws_send_frame_async().
 *
 * ════════════════════════════════════════════════════════════════════════════
 * MEMORY LAYOUT
 * ════════════════════════════════════════════════════════════════════════════
 *
 *   WHERE    WHAT                           SIZE        HOW ALLOCATED
 *   ───────  ─────────────────────────────  ──────────  ──────────────────────
 *   BSS      s_map  (header only)           24 B        global static
 *   HEAP     s_map.pool  (quadtree nodes)   96 KB       quadtree_map_init()
 *            8000 nodes × 12 B/node
 *
 *   BSS      s_scan  (lidar_scan_t)         5.4 KB      static local in task
 *            460 points × 12 B/point + 12 B header
 *            *** declared `static` inside task_lidar_slam so it goes to BSS,
 *                NOT onto the 6 KB task stack.  Without static it would
 *                immediately overflow the stack. ***
 *
 *   HEAP     task stacks
 *            task_lidar_slam                6 KB        xTaskCreatePinnedToCore
 *            task_perf_mon                  4 KB        xTaskCreate
 *            _dash_task  (wifi_dash)        4 KB        xTaskCreate (inside init)
 *
 *   BSS      wifi_dashboard.c internal      ~12 KB      global statics in that TU
 *            (map buf 2517 B + scan buf 363 B + pose/path/log bufs + shadow grid)
 *
 *   TOTAL HEAP:  ~96 KB pool + ~14 KB task stacks  ≈ 110 KB
 *   TOTAL BSS:   ~17 KB
 *   ESP32-S3 has ~320 KB free DRAM after IDF + WiFi → comfortable margin.
 *
 * ════════════════════════════════════════════════════════════════════════════
 * CPU PRIORITY NOTE
 * ════════════════════════════════════════════════════════════════════════════
 *   IDF WiFi driver tasks run at priority 23 — they will always preempt our
 *   task (prio 7) so the radio stays alive.
 *   IDF LwIP/TCP stack runs at priority 18 — also above us.
 *   httpd server runs at priority 5 — BELOW our task.
 *   → The 100 ms vTaskDelay after each scan is mandatory: it gives httpd
 *     time to flush TCP ACKs so the WebSocket does not stall.
 */

/* ── Wi-Fi credentials — fill in before flashing ───────────────────────── */
#define WIFI_SSID      "SPOT-iot"
#define WIFI_PASSWORD  "RacailleSalutaireMigration8052"

/* ── Only the modules we actually use in Phase 1 ────────────────────────── */
#include "../hardware_pins.h"
#include "src/lidar_driver.h"
#include "src/lidar_to_map.h"
#include "src/quadtree_map.h"
#include "src/wifi_dashboard.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"   /* esp_get_free_heap_size / esp_get_minimum_free_heap_size */
#include <stdbool.h>
#include <stdio.h>


/* ════════════════════════════════════════════════════════════════════════════
 * Shared state
 * ════════════════════════════════════════════════════════════════════════════ */

/* Map header in BSS; 96 KB pool allocated on heap by quadtree_map_init(). */
static quadtree_map_t s_map;

/* Fixed pose — robot starts at map centre, no odometry in Phase 1. */
static pose_t s_pose;

/* Task handles — saved so task_perf_mon can read stack watermarks. */
static TaskHandle_t s_h_lidar = NULL;
static TaskHandle_t s_h_perf  = NULL;

/* ── Diagnostic counters ─────────────────────────────────────────────────── *
 * Written by task_lidar_slam, read by task_perf_mon.                         *
 * volatile: guarantees the compiler does not cache stale values across tasks. *
 * No mutex: perf_mon is allowed to read a count that is one behind.          *
 *                                                                             *
 * Two separate stage timers let you see what actually dominates:             *
 *   s_read_*  — time inside lidar_driver_read_scan() (mostly UART blocking)  *
 *   s_l2m_*   — time inside lidar_to_map() (ray marching + quadtree writes)  *
 * If read >> l2m the bottleneck is UART/LiDAR speed, not your algorithm.     *
 * If l2m >> read the bottleneck is the map algorithm (cache / compute).      */
static volatile uint32_t s_scans_ok   = 0;   /* successful full scans        */
static volatile uint32_t s_scans_bad  = 0;   /* failed / short scans         */

static volatile uint32_t s_read_us_tot = 0;  /* cumulative scan-read µs      */
static volatile uint32_t s_read_us_max = 0;  /* worst scan-read µs           */

static volatile uint32_t s_l2m_calls  = 0;   /* lidar_to_map() invocations   */
static volatile uint32_t s_l2m_us_tot = 0;   /* cumulative lidar_to_map µs   */
static volatile uint32_t s_l2m_us_max = 0;   /* worst single lidar_to_map µs */


/* ════════════════════════════════════════════════════════════════════════════
 * task_lidar_slam  —  Core 0, priority 7
 * ════════════════════════════════════════════════════════════════════════════ */
static void task_lidar_slam(void *arg)
{
    (void)arg;

    /* *** 5.4 KB scan buffer — MUST be static ***
     * If this were a plain local variable it would sit on the 6 KB task stack
     * and leave only ~600 B for the rest of the function — instant overflow.
     * `static` moves it to BSS, completely outside the task stack. */
    static lidar_scan_t scan;

    int64_t last_scan_bcast_us = 0;   /* used to throttle scan broadcast */

    for (;;) {

        /* ── 1. Acquire one full 360° scan ──────────────────────────────── *
         * lidar_driver_read_scan() blocks until the LiDAR motor completes   *
         * one rotation (~100 ms at 10 Hz).  If the UART ring buffer already  *
         * holds a complete buffered scan it returns in ~0 ms.               */
        int64_t t_read  = esp_timer_get_time();
        bool    read_ok = lidar_driver_read_scan(&scan);
        uint32_t read_us = (uint32_t)(esp_timer_get_time() - t_read);

        if (!read_ok || scan.count < 10) {
            s_scans_bad++;
            vTaskDelay(pdMS_TO_TICKS(10));   /* brief back-off on failure */
            continue;
        }
        s_scans_ok++;
        s_read_us_tot += read_us;
        if (read_us > s_read_us_max) s_read_us_max = read_us;

        /* ── 2. Ray-march scan into quadtree map ────────────────────────── *
         * lidar_to_map() walks each beam from the sensor origin, marks cells *
         * as FREE along the ray, and marks the endpoint as OCCUPIED.         *
         * LIDAR_PROCESS_RANGE_MM and LIDAR_MAP_RADIUS_MM are from            *
         * lidar_to_map.h (3500 mm and 1500 mm respectively).                 */
        map_dirty_rect_t dirty;
        int64_t t0      = esp_timer_get_time();
        lidar_to_map(&s_map, &scan, &s_pose,
                     LIDAR_PROCESS_RANGE_MM,   /* hard range filter: 3500 mm  */
                     150.0f,                    /* ray-march step: 150 mm/cell */
                     &dirty);
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - t0);

        s_l2m_calls++;
        s_l2m_us_tot += elapsed;
        if (elapsed > s_l2m_us_max) s_l2m_us_max = elapsed;

        /* ── 3. Notify dashboard ─────────────────────────────────────────── *
         * wifi_dashboard_mark_dirty(): records which map region changed so   *
         * the dashboard can send a small delta frame instead of the full map.*
         *                                                                     *
         * wifi_dashboard_update(): throttled to 1 Hz inside wifi_dashboard.c.*
         * Posts a DASH_MAP_MSG to the 24-slot queue; _dash_task consumes it. */
        wifi_dashboard_mark_dirty(&dirty);
        wifi_dashboard_update(&s_map, &s_pose);

        /* Scan overlay at 2 Hz — 500 ms gate */
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_scan_bcast_us >= 500000LL) {
            wifi_dashboard_broadcast_scan(&scan, &s_pose);
            last_scan_bcast_us = now_us;
        }

        /* Robot pose dot on the dashboard (no frontier target in Phase 1) */
        wifi_dashboard_broadcast_state(&s_pose, 0.0f, 0.0f, false, 0);

        /* ── 4. Yield ────────────────────────────────────────────────────── *
         * httpd (prio 5) sits BELOW us.  Without this delay, httpd never gets*
         * CPU to send TCP ACKs and the WebSocket stalls after the first frame.*
         * 100 ms is enough for httpd to flush one or two outgoing frames.    */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * task_perf_mon  —  Core 0, priority 1
 *
 * Prints four diagnostic sections every 10 seconds.
 *
 * [PERF] heap
 *   free_now      — current free heap bytes
 *   min_ever      — lowest free heap since boot (shows worst-case usage)
 *   largest_blk   — largest contiguous free block (indicator of fragmentation)
 *   frag%         — 100 - (largest_blk / free_now) × 100
 *                   High frag means many small holes; large allocs will fail
 *                   even when free_now looks healthy.
 *
 * [PERF] stacks
 *   Bytes remaining between the current stack pointer and the stack guard.
 *   (uxTaskGetStackHighWaterMark × sizeof(StackType_t))
 *   If this reaches 0 → stack-overflow panic.
 *
 * [PERF] lidar / l2m
 *   Delta counters over the 10 s window → scan rate, drop %, l2m latency.
 *
 * [PERF] cpu
 *   FreeRTOS vTaskGetRunTimeStats() — absolute ticks + % CPU per task.
 *   Requires CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y (set in sdkconfig.defaults).
 * ════════════════════════════════════════════════════════════════════════════ */
static void task_perf_mon(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000));   /* wait for all tasks to settle */

    /* Previous-interval snapshots for delta calculation */
    uint32_t prev_ok = 0, prev_bad = 0, prev_l2m = 0, prev_us = 0;
    uint32_t prev_read_us = 0;

    for (;;) {
        /* ── Heap ───────────────────────────────────────────────────────── */
        uint32_t free_now = (uint32_t)esp_get_free_heap_size();
        uint32_t free_min = (uint32_t)esp_get_minimum_free_heap_size();
        uint32_t largest  = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
        uint32_t frag_pct = free_now ? (100u - largest * 100u / free_now) : 0u;

        printf("[PERF] heap    free=%lu B  min_ever=%lu B"
               "  largest_blk=%lu B  frag=%lu%%\n",
               (unsigned long)free_now,  (unsigned long)free_min,
               (unsigned long)largest,   (unsigned long)frag_pct);

        /* ── Stack watermarks ───────────────────────────────────────────── */
        printf("[PERF] stacks  lidar=%lu B  perf=%lu B\n",
               (unsigned long)(uxTaskGetStackHighWaterMark(s_h_lidar) * sizeof(StackType_t)),
               (unsigned long)(uxTaskGetStackHighWaterMark(s_h_perf)  * sizeof(StackType_t)));

        /* ── LiDAR scan pipeline ────────────────────────────────────────── */
        uint32_t now_ok  = s_scans_ok,  now_bad = s_scans_bad;
        uint32_t now_l2m = s_l2m_calls, now_us  = s_l2m_us_tot;

        uint32_t d_ok  = now_ok  - prev_ok;
        uint32_t d_bad = now_bad - prev_bad;
        uint32_t d_l2m = now_l2m - prev_l2m;
        uint32_t d_us  = now_us  - prev_us;
        uint32_t avg   = d_l2m ? d_us / d_l2m : 0u;
        uint32_t total = d_ok + d_bad;

        printf("[PERF] lidar   scans_ok=%lu  scans_bad=%lu  drop=%.0f%%  rate=%.1f/s\n",
               (unsigned long)d_ok,  (unsigned long)d_bad,
               total ? (double)d_bad * 100.0 / total : 0.0,
               (double)d_ok / 10.0);

        /* ── Per-stage breakdown ────────────────────────────────────────── *
         * scan_read: time blocked inside lidar_driver_read_scan()            *
         *            (mostly waiting for UART bytes / motor rotation)         *
         * l2m:       time inside lidar_to_map() (ray march + quadtree)       *
         * If scan_read >> l2m → bottleneck is UART speed, not the algorithm. *
         * If l2m >> scan_read  → optimise ray marching / quadtree access.    */
        uint32_t now_read_us = s_read_us_tot;
        uint32_t d_read_us   = now_read_us - prev_read_us;
        uint32_t avg_read    = d_ok ? d_read_us / d_ok : 0u;

        printf("[PERF] stage  scan_read: avg=%lu us  worst_ever=%lu us\n",
               (unsigned long)avg_read,  (unsigned long)s_read_us_max);
        printf("[PERF] stage  l2m:       avg=%lu us  worst_ever=%lu us  calls=%lu\n",
               (unsigned long)avg, (unsigned long)s_l2m_us_max, (unsigned long)d_l2m);

        /* Advance snapshots */
        prev_ok      = now_ok;  prev_bad = now_bad;
        prev_l2m     = now_l2m; prev_us  = now_us;
        prev_read_us = now_read_us;

        /* ── FreeRTOS per-task CPU usage ────────────────────────────────── */
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
        static char rt_buf[1024];
        vTaskGetRunTimeStats(rt_buf);
        printf("[PERF] cpu (task / ticks / %%cpu):\n%s\n", rt_buf);
#endif

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    /* ── LiDAR ─────────────────────────────────────────────────────────────── */
    lidar_driver_init();

    /* ── Quadtree map ──────────────────────────────────────────────────────── *
     * 10 m × 10 m arena, 50 mm minimum leaf-cell size.                        *
     * Internally allocates the 96 KB node pool on the heap.                   *
     * After this call the heap drops by ~96 KB — all subsequent allocs        *
     * (task stacks, wifi bufs) come from the remaining ~220 KB.               */
    quadtree_map_init(&s_map, 10000.0f, 10000.0f, 50.0f);

    /* Robot starts at map centre; stays fixed until odometry is wired in. */
    s_pose = (pose_t){ .x = 5000.0f, .y = 5000.0f, .theta = 0.0f };

    /* ── Dashboard ─────────────────────────────────────────────────────────── *
     * Connects to Wi-Fi (blocks up to 10 s), starts the httpd server, and     *
     * spawns _dash_task (Core 0, prio 1, 4 KB stack).                         *
     * Must be called AFTER quadtree_map_init so heap fragmentation is low.    */
    wifi_dashboard_init(WIFI_SSID, WIFI_PASSWORD);

    /* ── Tasks ─────────────────────────────────────────────────────────────── */
    xTaskCreatePinnedToCore(task_lidar_slam, "lscan",    6144, NULL, 7, &s_h_lidar, 0);
    xTaskCreate(             task_perf_mon,  "perf_mon", 4096, NULL, 1, &s_h_perf);
}
