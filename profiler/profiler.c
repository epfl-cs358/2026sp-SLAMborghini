/**
 * profiler.c — implementation of the SLAMborghini profiling framework.
 *
 * All code is inside #if PROFILER_ENABLED so that a production build with
 * PROFILER_ENABLED=0 produces an empty translation unit.
 */

#include "profiler.h"

#if PROFILER_ENABLED

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"

static const char *TAG = "PROFILER";

/* ── Heap trace buffer (conditionally compiled) ────────────────────────────── */
#ifdef CONFIG_HEAP_TRACING_STANDALONE
heap_trace_record_t _profiler_heap_trace_buf[PROFILER_HEAP_TRACE_RECORDS];
#endif

/* ── Global registry ───────────────────────────────────────────────────────── */
/* Written from app_main only — no mutex required. */
static task_profile_t  *s_tasks[PROFILER_MAX_TASKS];
static int              s_task_count;
static queue_profile_t *s_queues[PROFILER_MAX_QUEUES];
static int              s_queue_count;

/* ── Internal helpers ──────────────────────────────────────────────────────── */

static uint32_t heap_free_dram(void)
{
    return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

static uint32_t heap_frag_pct_dram(void)
{
    uint32_t free_sz    = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t largest    = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (free_sz == 0) return 0;
    return (uint32_t)(((float)(free_sz - largest) / (float)free_sz) * 100.0f);
}

/* ── task_profile ──────────────────────────────────────────────────────────── */

void task_profile_init(task_profile_t *p, const char *name,
                       uint32_t stack_size, UBaseType_t priority,
                       BaseType_t core_id)
{
    memset(p, 0, sizeof(*p));
    p->task_name         = name;
    p->stack_size        = stack_size;
    p->priority          = priority;
    p->core_id           = core_id;
    /* Sentinel values — updated on first cycle_end */
    p->cpu_cycles_min    = UINT32_MAX;
    p->stack_hwm_lowest  = UINT32_MAX;
    p->heap_free_min     = UINT32_MAX;
    p->heap_delta_max    = INT32_MAX;
}

void task_profile_cycle_begin(task_profile_t *p)
{
    p->_heap_free_before   = heap_free_dram();
    p->_cycle_start_cycles = esp_cpu_get_cycle_count();
}

void task_profile_cycle_end(task_profile_t *p)
{
    /* ── CPU ──────────────────────────────────────────────────────────────── */
    uint32_t elapsed = esp_cpu_get_cycle_count() - p->_cycle_start_cycles;
    p->cpu_cycles_last = elapsed;
    if (elapsed < p->cpu_cycles_min) p->cpu_cycles_min = elapsed;
    if (elapsed > p->cpu_cycles_max) p->cpu_cycles_max = elapsed;
    /* Incremental mean avoids float overflow on very long runs */
    p->cpu_cycles_avg += ((float)elapsed - p->cpu_cycles_avg)
                         / (float)(p->cycle_count + 1u);
    p->cycle_count++;

    /* ── Stack ────────────────────────────────────────────────────────────── */
    /* IDF-FreeRTOS (v5.x): uxTaskGetStackHighWaterMark returns bytes, not words */
    uint32_t hwm = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    p->stack_hwm_current = hwm;
    if (hwm < p->stack_hwm_lowest) p->stack_hwm_lowest = hwm;
    p->stack_bytes_used  = p->stack_size - hwm;
    p->stack_pct_used    = (p->stack_size > 0u)
                           ? ((float)p->stack_bytes_used / (float)p->stack_size * 100.0f)
                           : 0.0f;
    if (hwm < PROFILER_STACK_WARN_BYTES) {
        ESP_LOGW(TAG, "[%s] stack low — %" PRIu32 " B remaining (HWM)",
                 p->task_name, hwm);
    }

    /* ── Heap (end-of-cycle delta) ────────────────────────────────────────── */
    uint32_t free_now    = heap_free_dram();
    p->heap_delta_last   = (int32_t)free_now - (int32_t)p->_heap_free_before;
    if (p->heap_delta_last < p->heap_delta_max)
        p->heap_delta_max = p->heap_delta_last;
    if (free_now < p->heap_free_min) p->heap_free_min = free_now;
    p->heap_frag_pct = heap_frag_pct_dram();
    if (p->heap_frag_pct > PROFILER_HEAP_FRAG_WARN_PCT) {
        ESP_LOGW(TAG, "[%s] DRAM fragmentation %" PRIu32 "%%",
                 p->task_name, p->heap_frag_pct);
    }
}

void task_profile_dump(const task_profile_t *p)
{
    /* Safe conversion: sentinel min stays 0 if never updated */
    uint32_t cyc_min = (p->cpu_cycles_min == UINT32_MAX) ? 0u : p->cpu_cycles_min;
    uint32_t us_last = p->cpu_cycles_last / PROFILER_CPU_FREQ_MHZ;
    uint32_t us_min  = cyc_min            / PROFILER_CPU_FREQ_MHZ;
    uint32_t us_max  = p->cpu_cycles_max  / PROFILER_CPU_FREQ_MHZ;
    uint32_t us_avg  = (uint32_t)(p->cpu_cycles_avg / (float)PROFILER_CPU_FREQ_MHZ);

    uint32_t hwm_low  = (p->stack_hwm_lowest == UINT32_MAX) ? 0u : p->stack_hwm_lowest;
    uint32_t heap_min = (p->heap_free_min    == UINT32_MAX) ? 0u : p->heap_free_min;
    int32_t  dw       = (p->heap_delta_max == INT32_MAX)  ? 0  : p->heap_delta_max;

    ESP_LOGI(TAG, "+==================================================+");
    ESP_LOGI(TAG, "| Task: %-42s |", p->task_name);
    ESP_LOGI(TAG, "+==================================================+");
    ESP_LOGI(TAG, "| stack=%" PRIu32 " B  prio=%d  core=%d  runs=%" PRIu32,
             p->stack_size, (int)p->priority, (int)p->core_id, p->cycle_count);
    ESP_LOGI(TAG, "+-- CPU (us) --------------------------------------|");
    ESP_LOGI(TAG, "| last=%" PRIu32 "  min=%" PRIu32 "  max=%" PRIu32 "  avg=%" PRIu32,
             us_last, us_min, us_max, us_avg);
    ESP_LOGI(TAG, "+-- Stack (bytes) ---------------------------------|");
    ESP_LOGI(TAG, "| hwm_now=%" PRIu32 "  hwm_lowest=%" PRIu32
             "  used=%" PRIu32 "  %.1f%%",
             p->stack_hwm_current, hwm_low,
             p->stack_bytes_used, p->stack_pct_used);
    ESP_LOGI(TAG, "+-- Heap DRAM -------------------------------------|");
    ESP_LOGI(TAG, "| delta_last=%" PRId32 " B  worst=%" PRId32
             " B  free_min=%" PRIu32 " B  frag=%" PRIu32 "%%",
             p->heap_delta_last, dw, heap_min, p->heap_frag_pct);
    task_data_profile_dump(&p->data_profile);
    ESP_LOGI(TAG, "+==================================================+");
}

/* ── task_data_profile ─────────────────────────────────────────────────────── */

void task_data_profile_init(task_data_profile_t *dp,
                             const char *type_name, size_t item_size,
                             bool heap_allocated, bool dma_memory,
                             bool passes_ownership, const char *consumer)
{
    memset(dp, 0, sizeof(*dp));
    dp->output_type_name   = type_name;
    dp->item_size          = item_size;
    dp->heap_allocated     = heap_allocated;
    dp->dma_memory         = dma_memory;
    dp->passes_ownership   = passes_ownership;
    dp->consumer_task_name = consumer;
    dp->items_min          = UINT32_MAX;
}

void task_data_profile_update(task_data_profile_t *dp, uint32_t n_items)
{
    dp->items_this_cycle = n_items;
    dp->bytes_this_cycle = (uint32_t)((uint64_t)n_items * dp->item_size);

    if (n_items < dp->items_min) dp->items_min = n_items;
    if (n_items > dp->items_max) dp->items_max = n_items;

    /* Exponential moving average */
    dp->items_avg_ema = PROFILER_DATA_EMA_ALPHA * (float)n_items
                        + (1.0f - PROFILER_DATA_EMA_ALPHA) * dp->items_avg_ema;

    if (dp->bytes_this_cycle > dp->peak_allocation_bytes)
        dp->peak_allocation_bytes = dp->bytes_this_cycle;

    dp->total_bytes_produced += dp->bytes_this_cycle;

    snprintf(dp->layout_desc, sizeof(dp->layout_desc),
             "%" PRIu32 " x %zu-byte structs", n_items, dp->item_size);
}

void task_data_profile_dump(const task_data_profile_t *dp)
{
    if (!dp->output_type_name) return;
    uint32_t min_items = (dp->items_min == UINT32_MAX) ? 0u : dp->items_min;

    ESP_LOGI(TAG, "+-- Data Output ------------------------------------|");
    ESP_LOGI(TAG, "| type=%-20s  sz=%zu B  heap=%d  dma=%d",
             dp->output_type_name, dp->item_size,
             (int)dp->heap_allocated, (int)dp->dma_memory);
    ESP_LOGI(TAG, "| %s", dp->layout_desc);
    ESP_LOGI(TAG, "| items: last=%" PRIu32 "  min=%" PRIu32 "  max=%" PRIu32
             "  ema=%.1f",
             dp->items_this_cycle, min_items, dp->items_max, dp->items_avg_ema);
    ESP_LOGI(TAG, "| peak_alloc=%" PRIu32 " B  total=%" PRIu64 " B",
             dp->peak_allocation_bytes, dp->total_bytes_produced);
    if (dp->passes_ownership && dp->consumer_task_name) {
        ESP_LOGI(TAG, "| ownership -> %s", dp->consumer_task_name);
    }
}

/* ── queue_profile ─────────────────────────────────────────────────────────── */

void queue_profile_init(queue_profile_t *qp, const char *name,
                        QueueHandle_t handle, size_t item_size, uint32_t depth)
{
    memset(qp, 0, sizeof(*qp));
    qp->queue_name = name;
    qp->handle     = handle;
    qp->item_size  = item_size;
    qp->depth      = depth;
}

BaseType_t profiled_queue_send(queue_profile_t *qp, const void *item,
                               TickType_t timeout)
{
    BaseType_t result = xQueueSend(qp->handle, item, timeout);
    if (result == pdTRUE) {
        qp->total_sends++;
        qp->total_bytes_transferred += qp->item_size;
        UBaseType_t fill = uxQueueMessagesWaiting(qp->handle);
        if ((uint32_t)fill > qp->peak_fill) qp->peak_fill = (uint32_t)fill;
    } else {
        qp->total_drops++;
        ESP_LOGW(TAG, "[Q:%s] DROP — total=%" PRIu32 " (consumer too slow?)",
                 qp->queue_name, qp->total_drops);
    }
    return result;
}

BaseType_t profiled_queue_receive(queue_profile_t *qp, void *item,
                                  TickType_t timeout)
{
    BaseType_t result = xQueueReceive(qp->handle, item, timeout);
    if (result == pdTRUE) {
        qp->total_receives++;
        /* bytes_transferred is counted at send time — don't double-count here */
    }
    return result;
}

void queue_profile_dump(const queue_profile_t *qp)
{
    ESP_LOGI(TAG, "+-- Queue: %-38s |", qp->queue_name);
    ESP_LOGI(TAG, "| item_sz=%zu B  depth=%" PRIu32 "  peak_fill=%" PRIu32,
             qp->item_size, qp->depth, qp->peak_fill);
    ESP_LOGI(TAG, "| sends=%" PRIu32 "  recvs=%" PRIu32 "  drops=%" PRIu32
             "  bytes=%" PRIu64,
             qp->total_sends, qp->total_receives,
             qp->total_drops, qp->total_bytes_transferred);
    if (qp->total_drops > 0) {
        ESP_LOGW(TAG, "| [WARN] %" PRIu32 " drops — consumer too slow for producer!",
                 qp->total_drops);
    }
    ESP_LOGI(TAG, "+--------------------------------------------------+");
}

/* ── Registry ──────────────────────────────────────────────────────────────── */

void profiler_register_task(task_profile_t *p)
{
    if (s_task_count < (int)PROFILER_MAX_TASKS) {
        s_tasks[s_task_count++] = p;
    } else {
        ESP_LOGW(TAG, "task registry full — increase PROFILER_MAX_TASKS");
    }
}

void profiler_register_queue(queue_profile_t *qp)
{
    if (s_queue_count < (int)PROFILER_MAX_QUEUES) {
        s_queues[s_queue_count++] = qp;
    } else {
        ESP_LOGW(TAG, "queue registry full — increase PROFILER_MAX_QUEUES");
    }
}

/* ── profiler_dump_all ─────────────────────────────────────────────────────── */

void profiler_dump_all(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "##################################################");
    ESP_LOGI(TAG, "##          PROFILER FULL DUMP                  ##");
    ESP_LOGI(TAG, "##################################################");

    /* ── 1. System-wide heap snapshot ──────────────────────────────────────── */
    uint32_t dram_free   = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t dram_min    = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t dram_block  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t dma_free    = heap_caps_get_free_size(MALLOC_CAP_DMA);
    uint32_t psram_free  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "DRAM  : free=%" PRIu32 " B  min_ever=%" PRIu32
             " B  largest_block=%" PRIu32 " B",
             dram_free, dram_min, dram_block);
    ESP_LOGI(TAG, "DMA   : free=%" PRIu32 " B", dma_free);
    ESP_LOGI(TAG, "PSRAM : free=%" PRIu32 " B", psram_free);

    /* ── 2. FreeRTOS per-task runtime stats ────────────────────────────────── */
    /* Requires CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y in sdkconfig */
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    {
        /* Static: 3 KB in .bss — acceptable for profiling builds only */
        static char stats_buf[3072];
        vTaskGetRunTimeStats(stats_buf);
        ESP_LOGI(TAG, "--- vTaskGetRunTimeStats ---\n%s", stats_buf);
    }
#endif

    /* ── 3. Full task table via uxTaskGetSystemState ────────────────────────── */
    /* Requires CONFIG_FREERTOS_USE_TRACE_FACILITY=y */
#ifdef CONFIG_FREERTOS_USE_TRACE_FACILITY
    {
        static TaskStatus_t tbl[30];   /* static: ~44 B * 30 = ~1320 B in .bss */
        uint32_t total_rt = 0;
        UBaseType_t n = uxTaskGetSystemState(tbl, 30, &total_rt);

        static const char *state_str[] = {
            "Running", "Ready", "Blocked", "Suspended", "Deleted", "Invalid"
        };

        ESP_LOGI(TAG, "--- Task Table (%" PRIu32 " tasks, total_rt=%" PRIu32 ") ---",
                 (uint32_t)n, total_rt);
        ESP_LOGI(TAG, "%-16s %-10s %4s %4s %8s",
                 "Name", "State", "Prio", "Core", "HWM(B)");

        for (UBaseType_t i = 0; i < n; i++) {
            int si = (int)tbl[i].eCurrentState;
            const char *st = (si >= 0 && si <= 5) ? state_str[si] : "?";
#ifdef CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
            int core = (int)tbl[i].xCoreID;
#else
            int core = -1;
#endif
            ESP_LOGI(TAG, "%-16s %-10s %4d %4d %8" PRIu32,
                     tbl[i].pcTaskName, st,
                     (int)tbl[i].uxCurrentPriority,
                     core,
                     (uint32_t)tbl[i].usStackHighWaterMark);
        }
    }
#endif /* CONFIG_FREERTOS_USE_TRACE_FACILITY */

    /* ── 4. Per-task profiles ──────────────────────────────────────────────── */
    for (int i = 0; i < s_task_count; i++) {
        task_profile_dump(s_tasks[i]);
    }

    /* ── 5. Per-queue profiles ─────────────────────────────────────────────── */
    for (int i = 0; i < s_queue_count; i++) {
        queue_profile_dump(s_queues[i]);
    }

    ESP_LOGI(TAG, "##################################################");
    ESP_LOGI(TAG, "");
}

/* ── Monitor task ──────────────────────────────────────────────────────────── */

void profiler_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(PROFILER_DUMP_INTERVAL_MS));
        profiler_dump_all();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/* Xtensa hardware performance counters — compiled only when                  */
/* PROFILER_USE_PERFMON is defined and CONFIG_PERFMON=y is in sdkconfig       */
/* ═══════════════════════════════════════════════════════════════════════════ */
#ifdef PROFILER_USE_PERFMON

#include "xtensa_perfmon_apis.h"
#include "xtensa/xt_perf_consts.h"
#include "esp_err.h"

/* Six (select, mask) pairs — three hardware passes of two counters each.
 * The LX7 has two physical perfmon counters, so xtensa_perfmon_exec() makes
 * three passes, calling the work function once per pass (when repeat_count=1).
 *
 * Two pairs share XTPERF_CNT_INSN (select=2) but use different masks, so the
 * callback can distinguish them; they land in different passes anyway. */
static const uint32_t k_slam_perfmon_counters[] = {
    /* Pass 1 */
    XTPERF_CNT_CYCLES,          XTPERF_MASK_CYCLES,
    XTPERF_CNT_INSN,            XTPERF_MASK_INSN_ALL,
    /* Pass 2 */
    XTPERF_CNT_D_STALL,         XTPERF_MASK_D_STALL_ALL,
    XTPERF_CNT_I_STALL,         XTPERF_MASK_I_STALL_ALL,
    /* Pass 3 */
    XTPERF_CNT_INSN,            (XTPERF_MASK_INSN_BRANCH_TAKEN |
                                  XTPERF_MASK_INSN_BRANCH_NOT_TAKEN),
    XTPERF_CNT_BRANCH_PENALTY,  XTPERF_MASK_BRANCH_PENALTY,
};
#define PERFMON_NUM_PAIRS \
    (sizeof(k_slam_perfmon_counters) / (2u * sizeof(k_slam_perfmon_counters[0])))

/* Combined branch mask — constant so the callback switch can compare it */
#define PERFMON_BRANCH_MASK \
    (XTPERF_MASK_INSN_BRANCH_TAKEN | XTPERF_MASK_INSN_BRANCH_NOT_TAKEN)

void task_perfmon_profile_init(task_perfmon_profile_t *pm, const char *task_name,
                                int tracelevel, uint32_t repeat_count)
{
    memset(pm, 0, sizeof(*pm));
    pm->task_name    = task_name;
    pm->tracelevel   = tracelevel;
    pm->repeat_count = repeat_count ? repeat_count : 1u;
}

void perfmon_to_telemetry_cb(void *params,
                              uint32_t select, uint32_t mask, uint32_t value)
{
    task_perfmon_profile_t *pm = (task_perfmon_profile_t *)params;
    if (!pm) return;

    if (select == (uint32_t)XTPERF_CNT_CYCLES &&
        mask   == (uint32_t)XTPERF_MASK_CYCLES) {
        pm->cycles += value;
    } else if (select == (uint32_t)XTPERF_CNT_INSN &&
               mask   == (uint32_t)XTPERF_MASK_INSN_ALL) {
        pm->instructions += value;
    } else if (select == (uint32_t)XTPERF_CNT_D_STALL &&
               mask   == (uint32_t)XTPERF_MASK_D_STALL_ALL) {
        pm->dcache_stall_cycles += value;
    } else if (select == (uint32_t)XTPERF_CNT_I_STALL &&
               mask   == (uint32_t)XTPERF_MASK_I_STALL_ALL) {
        pm->icache_stall_cycles += value;
    } else if (select == (uint32_t)XTPERF_CNT_INSN &&
               mask   == (uint32_t)PERFMON_BRANCH_MASK) {
        pm->branch_count += value;
    } else if (select == (uint32_t)XTPERF_CNT_BRANCH_PENALTY &&
               mask   == (uint32_t)XTPERF_MASK_BRANCH_PENALTY) {
        pm->branch_penalty_cycles += value;
    }
    /* Unknown (select, mask) pairs are silently ignored — forward compatibility */
}

esp_err_t task_perfmon_collect(task_perfmon_profile_t *pm,
                                void (*work)(void *), void *work_arg,
                                int tracelevel, uint32_t repeat_count)
{
    if (!pm || !work) return ESP_ERR_INVALID_ARG;

    /* Preserve the task_name across the clear */
    const char *saved_name = pm->task_name;
    memset(pm, 0, sizeof(*pm));
    pm->task_name    = saved_name;
    pm->tracelevel   = tracelevel;
    pm->repeat_count = repeat_count ? repeat_count : 1u;

    xtensa_perfmon_config_t cfg = {
        .repeat_count    = (int)pm->repeat_count,
        .max_deviation   = 0.1f,   /* tolerate up to 10% variance across repeats */
        .call_params     = work_arg,
        .call_function   = work,
        .callback        = perfmon_to_telemetry_cb,
        .callback_params = pm,
        .tracelevel      = tracelevel,
        .counters_size   = (uint32_t)PERFMON_NUM_PAIRS,
        .select_mask     = k_slam_perfmon_counters,
    };

    esp_err_t err = xtensa_perfmon_exec(&cfg);
    if (err != ESP_OK) return err;

    /* Compute derived metrics from raw accumulators */
    if (pm->cycles > 0u) {
        pm->ipc = (float)pm->instructions / (float)pm->cycles;
        pm->dcache_stall_pct =
            (float)pm->dcache_stall_cycles / (float)pm->cycles * 100.0f;
        pm->icache_stall_pct =
            (float)pm->icache_stall_cycles / (float)pm->cycles * 100.0f;
    }
    if (pm->branch_count > 0u) {
        uint64_t est_mispreds = pm->branch_penalty_cycles / LX7_MISPRED_PENALTY_CYCLES;
        pm->mispred_rate_pct =
            (float)est_mispreds / (float)pm->branch_count * 100.0f;
    }

    return ESP_OK;
}

int task_perfmon_to_json(const task_perfmon_profile_t *pm, char *buf, size_t buf_size)
{
    if (!pm || !buf || buf_size < 2u) return -1;

    int n = snprintf(buf, buf_size,
        "{\"t\":\"%s\","
        "\"cyc\":%" PRIu64 ","
        "\"ins\":%" PRIu64 ","
        "\"ipc\":%.2f,"
        "\"dcs\":%.1f,"
        "\"ics\":%.1f,"
        "\"br\":%" PRIu64 ","
        "\"mpr\":%.1f}",
        pm->task_name ? pm->task_name : "?",
        pm->cycles,
        pm->instructions,
        (double)pm->ipc,
        (double)pm->dcache_stall_pct,
        (double)pm->icache_stall_pct,
        pm->branch_count,
        (double)pm->mispred_rate_pct);

    return (n > 0 && (size_t)n < buf_size) ? n : -1;
}

void task_perfmon_profile_dump(const task_perfmon_profile_t *pm)
{
    if (!pm) return;
    ESP_LOGI(TAG, "+-- Perfmon [%s] tracelevel=%d repeat=%" PRIu32 " --|",
             pm->task_name ? pm->task_name : "?",
             pm->tracelevel, pm->repeat_count);
    ESP_LOGI(TAG, "| cycles=%" PRIu64 "  insns=%" PRIu64 "  IPC=%.2f",
             pm->cycles, pm->instructions, (double)pm->ipc);
    ESP_LOGI(TAG, "| D-cache stall: %" PRIu64 " cyc  (%.1f%%)",
             pm->dcache_stall_cycles, (double)pm->dcache_stall_pct);
    ESP_LOGI(TAG, "| I-cache stall: %" PRIu64 " cyc  (%.1f%%)",
             pm->icache_stall_cycles, (double)pm->icache_stall_pct);
    ESP_LOGI(TAG, "| branches: %" PRIu64
             "  penalty_cyc: %" PRIu64
             "  est_mispred_rate: %.1f%%",
             pm->branch_count,
             pm->branch_penalty_cycles,
             (double)pm->mispred_rate_pct);
    ESP_LOGI(TAG, "+--------------------------------------------------+");
}

#endif /* PROFILER_USE_PERFMON */

#endif /* PROFILER_ENABLED */
