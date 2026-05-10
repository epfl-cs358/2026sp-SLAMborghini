/**
 * profiler.h — ESP32 / FreeRTOS task profiling framework
 *              SLAMborghini project — ESP-IDF v5.x, dual-core aware
 *
 * STATIC MEMORY ANALYSIS — run after every build:
 *   idf.py size              total binary: .text / .data / .bss / .iram0
 *   idf.py size-components   per-component breakdown
 *   idf.py size-files        per-.o file breakdown
 *
 *   .data   initialized globals — copied flash→DRAM at boot (costs flash + RAM)
 *   .bss    zero-init globals   — zeroed in DRAM at boot  (costs RAM only)
 *   .text   code executed from flash via XIP              (costs flash only)
 *   .iram0  code/data pinned in IRAM for ISR / time-critical paths
 *           (expensive: IRAM shared with Wi-Fi/BT stacks — budget carefully)
 *
 * COMPILE-TIME TOGGLE:
 *   #define PROFILER_ENABLED 0   strips everything — zero overhead in production
 *   #define PROFILER_ENABLED 1   full profiling active (default when not defined)
 *
 * THREAD SAFETY:
 *   Each task_profile_t is written only from its own task (in cycle_begin/end).
 *   The monitor task reads but never writes — no mutex required under this model.
 *   queue_profile_t is written by the sending task only; safe for single-producer queues.
 */

#ifndef PROFILER_H
#define PROFILER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifndef PROFILER_ENABLED
#define PROFILER_ENABLED 1
#endif

/* ── Tunables ──────────────────────────────────────────────────────────────── */
#define PROFILER_STACK_WARN_BYTES      512u    /* warn when this many bytes remain on stack */
#define PROFILER_HEAP_FRAG_WARN_PCT    30u     /* warn above this DRAM fragmentation % */
#define PROFILER_MOTOR_CYCLE_WARN_US   500u    /* motor task cycle time budget */
#define PROFILER_SLAM_CYCLE_WARN_US    20000u  /* SLAM task cycle time budget */
#define PROFILER_CPU_FREQ_MHZ          240u    /* clock assumed for cycle-count → µs conversion */
#define PROFILER_MAX_TASKS             16u     /* maximum tasks in global registry */
#define PROFILER_MAX_QUEUES            16u     /* maximum queues in global registry */
#define PROFILER_DATA_EMA_ALPHA        0.1f    /* EMA smoothing factor for item-count average */
#define PROFILER_DUMP_INTERVAL_MS      5000u   /* monitor task dump period */

/* ── Data output profile ───────────────────────────────────────────────────── */
/* Describes what a task produces each cycle and its lifetime statistics. */
typedef struct {
    const char  *output_type_name;       /* e.g. "lidar_point_t" */
    size_t       item_size;              /* sizeof one output item */
    bool         heap_allocated;         /* output buffer is heap-allocated */
    bool         dma_memory;             /* output buffer is DMA-capable */

    /* per-cycle shape */
    uint32_t     items_this_cycle;
    uint32_t     bytes_this_cycle;
    char         layout_desc[64];        /* e.g. "360 x 12-byte structs" */

    /* lifetime range */
    uint32_t     items_min;              /* min items ever produced in one cycle */
    uint32_t     items_max;             /* max items ever produced in one cycle */
    float        items_avg_ema;          /* exponential moving average, alpha=PROFILER_DATA_EMA_ALPHA */

    /* allocation sizing */
    uint32_t     peak_allocation_bytes;  /* largest single cycle allocation */

    /* ownership tracking */
    bool         passes_ownership;       /* true → producer does not free the buffer */
    const char  *consumer_task_name;     /* name of task that frees the buffer */

    /* cumulative */
    uint64_t     total_bytes_produced;   /* running total since init */
} task_data_profile_t;

/* ── Queue profile ─────────────────────────────────────────────────────────── */
typedef struct {
    const char   *queue_name;
    size_t        item_size;
    uint32_t      depth;
    QueueHandle_t handle;

    uint32_t      peak_fill;              /* max messages waiting at any moment */
    uint32_t      total_sends;
    uint32_t      total_receives;
    uint32_t      total_drops;            /* send-failed-because-full count */
    uint64_t      total_bytes_transferred;/* bytes successfully sent into queue */
} queue_profile_t;

/* ── Per-task profile ──────────────────────────────────────────────────────── */
typedef struct {
    /* identity */
    const char  *task_name;
    uint32_t     stack_size;             /* bytes, as passed to xTaskCreatePinnedToCore */
    UBaseType_t  priority;
    BaseType_t   core_id;

    /* CPU timing (raw cycle counts; divide by PROFILER_CPU_FREQ_MHZ for µs) */
    uint32_t     cpu_cycles_min;
    uint32_t     cpu_cycles_max;
    uint32_t     cpu_cycles_last;
    float        cpu_cycles_avg;         /* incremental mean — no overflow risk */
    uint32_t     cycle_count;

    /* stack (bytes — IDF-FreeRTOS returns bytes from uxTaskGetStackHighWaterMark) */
    uint32_t     stack_hwm_current;      /* bytes free at end of last cycle */
    uint32_t     stack_hwm_lowest;       /* minimum bytes free ever seen */
    uint32_t     stack_bytes_used;       /* stack_size - stack_hwm_current */
    float        stack_pct_used;

    /* heap — internal DRAM, measured at cycle boundaries */
    int32_t      heap_delta_last;        /* free_end - free_begin this cycle; negative = net alloc */
    int32_t      heap_delta_max;         /* worst (most negative) single-cycle delta ever seen */
    uint32_t     heap_free_min;          /* lowest free DRAM ever seen since init */
    uint32_t     heap_frag_pct;          /* fragmentation % at last cycle end */

    /* data output characterisation */
    task_data_profile_t data_profile;

    /* private — written by cycle_begin, consumed by cycle_end */
    uint32_t     _cycle_start_cycles;
    uint32_t     _heap_free_before;
} task_profile_t;


/* ═══════════════════════════════════════════════════════════════════════════ */
#if PROFILER_ENABLED
/* ═══════════════════════════════════════════════════════════════════════════ */

#include "esp_cpu.h"

/**
 * PROFILE_CPU_BEGIN(label) / PROFILE_CPU_END(label, us_out)
 *
 * Wrap any sub-block inside a task cycle to measure it independently.
 * 'label' must be a valid C identifier — it is used to create a unique variable name.
 * 'us_out' is a uint32_t* that receives the elapsed microseconds (may be NULL).
 *
 * Example:
 *   PROFILE_CPU_BEGIN(scan_match);
 *   icp_run(&scan, &map);
 *   uint32_t icp_us;
 *   PROFILE_CPU_END(scan_match, &icp_us);
 */
#define PROFILE_CPU_BEGIN(label) \
    uint32_t _prof_start_##label = esp_cpu_get_cycle_count()

#define PROFILE_CPU_END(label, us_out) \
    do { \
        uint32_t _elapsed_##label = esp_cpu_get_cycle_count() - _prof_start_##label; \
        if ((us_out) != NULL) { \
            *(us_out) = _elapsed_##label / PROFILER_CPU_FREQ_MHZ; \
        } \
    } while (0)

/* ── Heap leak tracing ─────────────────────────────────────────────────────
 * Requires CONFIG_HEAP_TRACING=y and CONFIG_HEAP_TRACING_STANDALONE=y.
 * When enabled: HEAP_TRACE_START() arms the tracer, HEAP_TRACE_STOP_DUMP()
 * stops it and prints every allocation that was not freed.
 *
 * Warning: tracing instruments every malloc/free — significant overhead.
 * Use only in targeted debug sessions, not in continuous profiling.
 */
#ifdef CONFIG_HEAP_TRACING_STANDALONE
#include "esp_heap_trace.h"
#define PROFILER_HEAP_TRACE_RECORDS 64
extern heap_trace_record_t _profiler_heap_trace_buf[PROFILER_HEAP_TRACE_RECORDS];

#define HEAP_TRACE_START() \
    do { \
        heap_trace_init_standalone(_profiler_heap_trace_buf, PROFILER_HEAP_TRACE_RECORDS); \
        heap_trace_start(HEAP_TRACE_LEAKS); \
    } while (0)

#define HEAP_TRACE_STOP_DUMP() \
    do { \
        heap_trace_stop(); \
        heap_trace_dump(); \
    } while (0)
#else
#define HEAP_TRACE_START()      do {} while (0)
#define HEAP_TRACE_STOP_DUMP()  do {} while (0)
#endif /* CONFIG_HEAP_TRACING_STANDALONE */

/* ── task_profile API ──────────────────────────────────────────────────────── */

/** One-time init — call before creating the task or at the top of the task function. */
void task_profile_init(task_profile_t *p, const char *name,
                       uint32_t stack_size, UBaseType_t priority,
                       BaseType_t core_id);

/** Call at the very top of each task loop iteration (snapshots heap + cycle counter). */
void task_profile_cycle_begin(task_profile_t *p);

/**
 * Call at the very bottom of each task loop iteration, BEFORE vTaskDelay.
 * Updates CPU, stack, and heap metrics.
 */
void task_profile_cycle_end(task_profile_t *p);

/** Pretty-print the full profile via ESP_LOGI. */
void task_profile_dump(const task_profile_t *p);

/* ── task_data_profile API ─────────────────────────────────────────────────── */

/** One-time init for the data output sub-profile. */
void task_data_profile_init(task_data_profile_t *dp,
                             const char *type_name, size_t item_size,
                             bool heap_allocated, bool dma_memory,
                             bool passes_ownership, const char *consumer);

/** Call once per cycle after producing output. n_items = number of items produced. */
void task_data_profile_update(task_data_profile_t *dp, uint32_t n_items);

/** Pretty-print the data sub-profile (called automatically by task_profile_dump). */
void task_data_profile_dump(const task_data_profile_t *dp);

/* ── queue_profile API ─────────────────────────────────────────────────────── */

/** One-time init — call after xQueueCreate. */
void queue_profile_init(queue_profile_t *qp, const char *name,
                        QueueHandle_t handle, size_t item_size, uint32_t depth);

/**
 * Drop-in replacements for xQueueSend / xQueueReceive.
 * Update peak fill, drop count, and byte counters automatically.
 */
BaseType_t profiled_queue_send(queue_profile_t *qp, const void *item,
                               TickType_t timeout);
BaseType_t profiled_queue_receive(queue_profile_t *qp, void *item,
                                  TickType_t timeout);

/** Pretty-print the queue profile. */
void queue_profile_dump(const queue_profile_t *qp);

/* ── Global registry ───────────────────────────────────────────────────────── */

/** Register a task profile so profiler_dump_all() includes it. */
void profiler_register_task(task_profile_t *p);

/** Register a queue profile so profiler_dump_all() includes it. */
void profiler_register_queue(queue_profile_t *qp);

/**
 * Dump everything:
 *   1. System-wide heap (DRAM, DMA, PSRAM)
 *   2. vTaskGetRunTimeStats() per-task CPU % table
 *   3. uxTaskGetSystemState() full task table
 *   4. per-task task_profile_dump() for every registered task
 *   5. per-queue queue_profile_dump() for every registered queue
 */
void profiler_dump_all(void);

/**
 * FreeRTOS task entry point for the monitor task.
 * Create with lowest priority (1). Calls profiler_dump_all() every
 * PROFILER_DUMP_INTERVAL_MS milliseconds.
 *
 * Example:
 *   xTaskCreatePinnedToCore(profiler_monitor_task, "prof_mon",
 *                           4096, NULL, 1, NULL, 0);
 */
void profiler_monitor_task(void *arg);


/* ═══════════════════════════════════════════════════════════════════════════ */
/* Hardware performance counters (Xtensa perfmon) — optional layer            */
/* Requires:  -DPROFILER_USE_PERFMON  and  CONFIG_PERFMON=y in sdkconfig      */
/* ═══════════════════════════════════════════════════════════════════════════ */
#ifdef PROFILER_USE_PERFMON

#include "xtensa_perfmon_apis.h"
#include "esp_err.h"

/**
 * Approximate branch misprediction penalty on Xtensa LX7 (ESP32-S3).
 * Hardware reports total penalty cycles; dividing by this constant gives an
 * estimated misprediction count.  True penalty varies (4-10 cycles depending
 * on branch distance) -- treat mispred_rate_pct as an approximation.
 */
#define LX7_MISPRED_PENALTY_CYCLES 6u

/**
 * Results of one xtensa_perfmon_exec() measurement pass.
 *
 * Raw 64-bit accumulators survive repeat_count > 1 without overflow.
 * Derived float metrics are computed by task_perfmon_collect() after capture.
 *
 * Thread safety: owned by one task; monitor reads without a lock (safe because
 * monitor only reads after collection completes and all fields are stable).
 */
typedef struct {
    const char *task_name;

    /* Raw hardware counter values */
    uint64_t cycles;
    uint64_t instructions;
    uint64_t dcache_stall_cycles;
    uint64_t icache_stall_cycles;
    uint64_t branch_count;
    uint64_t branch_penalty_cycles;

    /* Derived metrics set by task_perfmon_collect() */
    float    ipc;               /* instructions / cycles */
    float    dcache_stall_pct;  /* dcache_stall_cycles / cycles x 100 */
    float    icache_stall_pct;  /* icache_stall_cycles / cycles x 100 */
    float    mispred_rate_pct;  /* (branch_penalty_cycles / LX7_MISPRED_PENALTY_CYCLES)
                                 * / branch_count x 100 -- estimated, not exact */

    /* Configuration snapshot */
    int      tracelevel;
    uint32_t repeat_count;
} task_perfmon_profile_t;

/** Zero struct and set identity fields.  Call once before task_perfmon_collect(). */
void task_perfmon_profile_init(task_perfmon_profile_t *pm, const char *task_name,
                                int tracelevel, uint32_t repeat_count);

/**
 * Callback for xtensa_perfmon_exec() that stores results into a
 * task_perfmon_profile_t instead of printing to stdout.
 *
 * Set config.callback = perfmon_to_telemetry_cb and
 *     config.callback_params = &your_pm_struct.
 * Exposed publicly so callers can build custom xtensa_perfmon_config_t when
 * the six default counters are insufficient.
 */
void perfmon_to_telemetry_cb(void *params,
                              uint32_t select, uint32_t mask, uint32_t value);

/**
 * Run xtensa_perfmon_exec() over a work function and populate pm.
 *
 * Measures six events in three hardware passes (LX7 has two physical counters):
 *   Pass 1: cycles, total instructions
 *   Pass 2: D-cache stall cycles, I-cache stall cycles
 *   Pass 3: branch count (taken + not-taken), branch penalty cycles
 *
 * work() is called once per pass, so it runs exactly three times total.
 *
 * @param pm           output struct (cleared then populated by this call)
 * @param work         function to profile
 * @param work_arg     opaque pointer forwarded to work()
 * @param tracelevel   -1  = count in all contexts (normal task profiling)
 *                     >=0 = count ONLY when interrupt_level > tracelevel
 *                     WARNING: tracelevel=0 counts exclusively during ISR
 *                     execution -- it does NOT exclude ISR overhead.
 * @param repeat_count times work() is called per pass (use 1 for real tasks)
 * @return             ESP_OK, ESP_ERR_INVALID_ARG, or ESP_FAIL on overflow
 */
esp_err_t task_perfmon_collect(task_perfmon_profile_t *pm,
                                void (*work)(void *), void *work_arg,
                                int tracelevel, uint32_t repeat_count);

/**
 * Serialise pm to compact single-line JSON for UART transmission.
 *
 * Example output (one line, no trailing newline):
 *   {"t":"full_slam","cyc":12345678,"ins":9876543,"ipc":0.80,
 *    "dcs":5.2,"ics":1.3,"br":12345,"mpr":2.1}
 *
 * Abbreviated keys minimise wire bytes:
 *   t=task_name, cyc=cycles, ins=instructions, ipc=IPC,
 *   dcs=D-cache stall%, ics=I-cache stall%, br=branch count, mpr=mispred%
 *
 * @return bytes written (excluding NUL), or -1 if buf_size insufficient
 */
int task_perfmon_to_json(const task_perfmon_profile_t *pm,
                          char *buf, size_t buf_size);

/** Pretty-print all perfmon metrics via ESP_LOGI. */
void task_perfmon_profile_dump(const task_perfmon_profile_t *pm);

#else  /* PROFILER_USE_PERFMON not defined -- stub the API to nothing */

typedef struct {
    const char *task_name;
    int         tracelevel;
    uint32_t    repeat_count;
} task_perfmon_profile_t;

static inline void task_perfmon_profile_init(task_perfmon_profile_t *pm,
    const char *n, int tl, uint32_t rc)
    { if (pm) { pm->task_name = n; pm->tracelevel = tl; pm->repeat_count = rc; } }
static inline void perfmon_to_telemetry_cb(void *p,
    uint32_t s, uint32_t m, uint32_t v)
    { (void)p; (void)s; (void)m; (void)v; }
static inline int task_perfmon_collect(task_perfmon_profile_t *pm,
    void (*work)(void *), void *arg, int tl, uint32_t rc)
    { (void)pm; (void)work; (void)arg; (void)tl; (void)rc; return 0; }
static inline int task_perfmon_to_json(const task_perfmon_profile_t *pm,
    char *buf, size_t sz)
    { (void)pm; if (buf && sz) buf[0] = '\0'; return 0; }
static inline void task_perfmon_profile_dump(const task_perfmon_profile_t *pm)
    { (void)pm; }

#endif /* PROFILER_USE_PERFMON */


/* ═══════════════════════════════════════════════════════════════════════════ */
#else  /* PROFILER_ENABLED == 0 -- compile everything away */
/* ═══════════════════════════════════════════════════════════════════════════ */

#define PROFILE_CPU_BEGIN(label)          do {} while (0)
#define PROFILE_CPU_END(label, us_out)    do {} while (0)
#define HEAP_TRACE_START()                do {} while (0)
#define HEAP_TRACE_STOP_DUMP()            do {} while (0)

static inline void task_profile_init(task_profile_t *p, const char *n,
    uint32_t s, UBaseType_t pr, BaseType_t c)
    { (void)p; (void)n; (void)s; (void)pr; (void)c; }
static inline void task_profile_cycle_begin(task_profile_t *p) { (void)p; }
static inline void task_profile_cycle_end(task_profile_t *p)   { (void)p; }
static inline void task_profile_dump(const task_profile_t *p)  { (void)p; }

static inline void task_data_profile_init(task_data_profile_t *dp,
    const char *t, size_t s, bool h, bool d, bool o, const char *c)
    { (void)dp; (void)t; (void)s; (void)h; (void)d; (void)o; (void)c; }
static inline void task_data_profile_update(task_data_profile_t *dp, uint32_t n)
    { (void)dp; (void)n; }
static inline void task_data_profile_dump(const task_data_profile_t *dp)
    { (void)dp; }

static inline void queue_profile_init(queue_profile_t *qp, const char *n,
    QueueHandle_t h, size_t s, uint32_t d)
    { (void)qp; (void)n; (void)h; (void)s; (void)d; }
static inline BaseType_t profiled_queue_send(queue_profile_t *qp,
    const void *item, TickType_t t)
    { return xQueueSend(qp->handle, item, t); }
static inline BaseType_t profiled_queue_receive(queue_profile_t *qp,
    void *item, TickType_t t)
    { return xQueueReceive(qp->handle, item, t); }
static inline void queue_profile_dump(const queue_profile_t *qp) { (void)qp; }

static inline void profiler_register_task(task_profile_t *p)    { (void)p; }
static inline void profiler_register_queue(queue_profile_t *qp) { (void)qp; }
static inline void profiler_dump_all(void)                       {}
static inline void profiler_monitor_task(void *arg)
    { (void)arg; vTaskDelete(NULL); }

/* Perfmon stubs when PROFILER_ENABLED=0 */
typedef struct {
    const char *task_name;
    int         tracelevel;
    uint32_t    repeat_count;
} task_perfmon_profile_t;
static inline void task_perfmon_profile_init(task_perfmon_profile_t *pm,
    const char *n, int tl, uint32_t rc)
    { if (pm) { pm->task_name = n; pm->tracelevel = tl; pm->repeat_count = rc; } }
static inline void perfmon_to_telemetry_cb(void *p,
    uint32_t s, uint32_t m, uint32_t v)
    { (void)p; (void)s; (void)m; (void)v; }
static inline int task_perfmon_collect(task_perfmon_profile_t *pm,
    void (*work)(void *), void *arg, int tl, uint32_t rc)
    { (void)pm; (void)work; (void)arg; (void)tl; (void)rc; return 0; }
static inline int task_perfmon_to_json(const task_perfmon_profile_t *pm,
    char *buf, size_t sz)
    { (void)pm; if (buf && sz) buf[0] = '\0'; return 0; }
static inline void task_perfmon_profile_dump(const task_perfmon_profile_t *pm)
    { (void)pm; }

#endif /* PROFILER_ENABLED */
#endif /* PROFILER_H */
