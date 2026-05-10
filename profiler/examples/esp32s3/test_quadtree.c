/**
 * test_quadtree.c — Profiled QuadTreeMap stress test.
 *
 * Exercises qt_update(), qt_query_const(), and qt_iterate_occupied()
 * on a single map instance. Resets the map when the pool saturates.
 *
 * Board: ESP32-S3
 * Rate:  10 Hz (100 ms delay)
 * Budget warning: >20 ms cycle time
 *
 * Per cycle:
 *   - 200 qt_update() calls (alternating HIT/MISS at random positions)
 *   - 50  qt_query_const() calls
 *   - 1   qt_iterate_occupied() pass (counted via callback)
 *   - Pool-full reset with re-seed
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <math.h>

static const char *TAG = "TEST_QUADTREE";

#define MAP_W    10000.0f
#define MAP_H    10000.0f
#define MAP_STEP   200.0f
#define N_UPDATES    200
#define N_QUERIES     50

static task_profile_t s_profile;
static uint32_t       s_occupied_count;

static void count_occupied_cb(float cx, float cy, int8_t value, void *ud)
{
    (void)cx; (void)cy; (void)value;
    (*(uint32_t *)ud)++;
}

task_profile_t *quadtree_task_get_profile(void) { return &s_profile; }

void quadtree_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "quadtree_test",
                      6144,
                      3,
                      0);   /* core 0 — compute-intensive */

    /* Item = one QTNode insertion — track how many nodes the pool holds */
    task_data_profile_init(&s_profile.data_profile,
                           "QTNode",
                           sizeof(QTNode),
                           /*heap_allocated=*/ true,   /* pool itself is heap-allocated */
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    srand((unsigned)esp_timer_get_time());

    QuadTreeMap map;
    qt_init(&map, 0.0f, MAP_W, 0.0f, MAP_H);

    while (1) {
        task_profile_cycle_begin(&s_profile);

        /* ── Update phase ──────────────────────────────────────────────────── */
        PROFILE_CPU_BEGIN(update);
        for (int i = 0; i < N_UPDATES; i++) {
            float x = (float)(rand() % (int)MAP_W);
            float y = (float)(rand() % (int)MAP_H);
            int8_t delta = (i & 1) ? QT_HIT_INC : QT_MISS_DEC;
            qt_update(&map, x, y, delta);
        }
        uint32_t update_us;
        PROFILE_CPU_END(update, &update_us);

        /* ── Query phase ───────────────────────────────────────────────────── */
        PROFILE_CPU_BEGIN(query);
        for (int i = 0; i < N_QUERIES; i++) {
            float x = (float)(rand() % (int)MAP_W);
            float y = (float)(rand() % (int)MAP_H);
            (void)qt_query_const(&map, x, y);
        }
        uint32_t query_us;
        PROFILE_CPU_END(query, &query_us);

        /* ── Iterate occupied ──────────────────────────────────────────────── */
        PROFILE_CPU_BEGIN(iterate);
        s_occupied_count = 0;
        qt_iterate_occupied(&map, count_occupied_cb, &s_occupied_count);
        uint32_t iter_us;
        PROFILE_CPU_END(iterate, &iter_us);

        task_data_profile_update(&s_profile.data_profile, (uint32_t)map.count);

        task_profile_cycle_end(&s_profile);

        uint32_t cycle_us = s_profile.cpu_cycles_last / 240;
        if (cycle_us > 20000) {
            ESP_LOGW(TAG, "cycle %" PRIu32 " µs over 20 ms budget "
                     "(upd %" PRIu32 " qry %" PRIu32 " iter %" PRIu32 " µs)",
                     cycle_us, update_us, query_us, iter_us);
        }

        ESP_LOGI(TAG, "nodes=%u occupied=%" PRIu32 " mem=%u B "
                 "upd=%" PRIu32 " qry=%" PRIu32 " iter=%" PRIu32 " µs",
                 map.count, s_occupied_count, (unsigned)qt_memory_bytes(&map),
                 update_us, query_us, iter_us);

        /* Reset when pool is almost full to avoid stalling qt_update */
        if (qt_is_pool_full(&map)) {
            ESP_LOGW(TAG, "pool full at %u nodes — resetting map", map.count);
            qt_free(&map);
            qt_init(&map, 0.0f, MAP_W, 0.0f, MAP_H);
        }

        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}
