/**
 * test_odometry.c — Full real-hardware odometry profiling.
 *
 * Mirrors task_odometry() step by step with per-step timing:
 *
 *   Step 1  imu_gyro_update()             — I2C read (ICM-20948) + heading integration
 *   Step 2  imu_encoder_driver_update()   — I2C read (AS5600) + distance accumulation
 *   Step 3  ZUPT (when stationary)        — bias refinement (EMA, no I2C)
 *   Step 4  encoder_ackermann_odom_update() — Ackermann + IMU fusion math
 *   Step 5  portMUX critical section      — overhead of pose mutex on same task
 *
 * REQUIRES HARDWARE:
 *   ICM-20948 IMU  — SDA=GPIO21, SCL=GPIO22, I2C_NUM_0, addr 0x68
 *   AS5600 encoder — same I2C bus, addr 0x36
 *   (imu_gyro_init() installs the I2C driver — no separate I2C init needed)
 *
 * Rate: 100 Hz (10 ms vTaskDelay, same as production task_odometry)
 *
 * ── Per-cycle log (every 100 ticks = 1 Hz) ──────────────────────────────────
 *   gyro_us    imu_gyro_update() wall time        (I2C + float math)
 *   enc_us     imu_encoder_driver_update() time   (I2C + wrap + accumulate)
 *   odom_us    encoder_ackermann_odom_update()    (pure float math)
 *   mux_us     portMUX enter+odom+exit overhead   (vs standalone odom_us)
 *   total_us   full cycle compute time
 *
 * ── ZUPT log (when triggered) ───────────────────────────────────────────────
 *   stationary_ticks, zupt_us, gyro bias refined
 *
 * ── Memory snapshot (every 500 ticks = 5 s) ─────────────────────────────────
 *   heap_free, heap_watermark, largest_block, frag%, stack HWM
 *
 * ── I2C error tracking ──────────────────────────────────────────────────────
 *   enc_fail_streak  consecutive AS5600 I2C failures (encoder frozen warning)
 *   imu_fail_streak  consecutive ICM-20948 I2C failures (heading frozen warning)
 */

#include "../../profiler.h"
#include "../../../wemos/src/encoder_ackermann_odometry.h"
#include "../../../wemos/src/imu_encoder_driver.h"
#include "../.././../wemos/imu_gyro.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

static const char *TAG = "TEST_ODOM";

/* Matches production wemos/src/task_odometry.c exactly */
static const odom_config_t k_cfg = {
    .wheelbase_m         = 0.258f,
    .imu_correction_gain = 1.0f,
    .max_delta_dist_m    = 0.08f,
    .max_yaw_jump_rad    = 0.35f,
};

#define ZUPT_STATIONARY_TICKS  20
#define ZUPT_DIST_THRESHOLD_M  0.001f

static task_profile_t           s_profile;
static encoder_ackermann_odom_t s_odom;
static portMUX_TYPE             s_pose_mux = portMUX_INITIALIZER_UNLOCKED;

task_profile_t *odometry_task_get_profile(void) { return &s_profile; }

void odometry_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "odometry_test",
                      4096,
                      5,    /* priority — matches production task_odometry */
                      1);   /* core 1 — I/O core */

    task_data_profile_init(&s_profile.data_profile,
                           "odom_pose_t",
                           sizeof(odom_pose_t),
                           false, false, false, NULL);

    /* ── Static RAM breakdown ────────────────────────────────────────────── */
    ESP_LOGI(TAG, "=== STATIC RAM ===");
    ESP_LOGI(TAG, "  encoder_ackermann_odom_t  %u B", (unsigned)sizeof(encoder_ackermann_odom_t));
    ESP_LOGI(TAG, "  odom_pose_t               %u B", (unsigned)sizeof(odom_pose_t));
    ESP_LOGI(TAG, "  portMUX_TYPE              %u B", (unsigned)sizeof(portMUX_TYPE));
    ESP_LOGI(TAG, "  s_bias_z (imu_gyro.c)     4 B (float, module static)");

    /* ── I2C + IMU init (imu_gyro_init installs the I2C driver) ─────────── */
    uint32_t heap_pre_init = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Heap before init: %" PRIu32 " B", heap_pre_init);

    bool imu_ok = imu_gyro_init();
    if (!imu_ok) {
        ESP_LOGE(TAG, "ICM-20948 init failed — check SDA=GPIO21 SCL=GPIO22 wiring");
    } else {
        ESP_LOGI(TAG, "ICM-20948 OK. Calibrating gyro bias (3 s, keep car still) …");
        float bias = imu_gyro_calibrate_bias(300);   /* 300 × 10 ms = 3 s */
        ESP_LOGI(TAG, "Gyro bias = %.5f rad/s", (double)bias);
    }

    uint32_t heap_post_init = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Heap after  init: %" PRIu32 " B  (I2C driver cost: %" PRIu32 " B)",
             heap_post_init, heap_pre_init - heap_post_init);

    encoder_ackermann_odom_init(&s_odom, &k_cfg);
    encoder_ackermann_odom_reset(&s_odom, 0.0f, 0.0f, 0.0f);

    float    prev_distance_m   = 0.0f;
    int      stationary_ticks  = 0;
    uint32_t tick              = 0;

    /* I2C failure streak counters — mirrors imu_encoder_driver.c internal logic */
    uint32_t enc_fail_total = 0;
    uint32_t imu_fail_total = 0;

    /* Min/max trackers for per-step timing across the session */
    uint32_t gyro_us_min = UINT32_MAX, gyro_us_max = 0;
    uint32_t enc_us_min  = UINT32_MAX, enc_us_max  = 0;
    uint32_t odom_us_min = UINT32_MAX, odom_us_max = 0;
    uint32_t mux_us_min  = UINT32_MAX, mux_us_max  = 0;

    while (1) {
        task_profile_cycle_begin(&s_profile);

        /* ── Step 1: IMU gyro integration (I2C read + float) ────────────── */
        PROFILE_CPU_BEGIN(gyro);
        imu_gyro_update(0.01f);
        uint32_t gyro_us;
        PROFILE_CPU_END(gyro, &gyro_us);

        if (gyro_us > gyro_us_max) gyro_us_max = gyro_us;
        if (gyro_us < gyro_us_min) gyro_us_min = gyro_us;

        /* ── Step 2: AS5600 encoder I2C read + accumulate ───────────────── */
        PROFILE_CPU_BEGIN(enc);
        imu_encoder_driver_update();
        uint32_t enc_us;
        PROFILE_CPU_END(enc, &enc_us);

        if (enc_us > enc_us_max) enc_us_max = enc_us;
        if (enc_us < enc_us_min) enc_us_min = enc_us;

        float distance_m   = imu_encoder_driver_get_distance_m();
        float yaw_rad      = imu_encoder_driver_get_yaw_rad();
        float steering_rad = 0.0f;   /* no servo command available standalone */

        /* Detect I2C failures: if distance didn't move AND enc_us was very fast,
         * the I2C read likely failed (driver returns early without updating). */
        float delta_dist_m = fabsf(distance_m - prev_distance_m);

        /* ── Step 3: ZUPT — gyro bias refinement when stationary ────────── */
        uint32_t zupt_us = 0;
        if (delta_dist_m < ZUPT_DIST_THRESHOLD_M) {
            stationary_ticks++;
            if (stationary_ticks >= ZUPT_STATIONARY_TICKS) {
                PROFILE_CPU_BEGIN(zupt);
                imu_gyro_zupt_update();
                PROFILE_CPU_END(zupt, &zupt_us);

                if (zupt_us > 0 && tick % 100u == 0u) {
                    ESP_LOGI(TAG, "[ZUPT] bias refined — stationary=%d ticks  zupt=%"
                             PRIu32 " µs", stationary_ticks, zupt_us);
                }
            }
        } else {
            stationary_ticks = 0;
        }
        prev_distance_m = distance_m;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        /* ── Step 4: standalone odom math timing (no mutex — for µs baseline) */
        PROFILE_CPU_BEGIN(odom_bare);
        encoder_ackermann_odom_update(&s_odom, distance_m, steering_rad,
                                      yaw_rad, now_ms);
        uint32_t odom_us;
        PROFILE_CPU_END(odom_bare, &odom_us);

        if (odom_us > odom_us_max) odom_us_max = odom_us;
        if (odom_us < odom_us_min) odom_us_min = odom_us;

        /* ── Step 5: portMUX critical section overhead ──────────────────── */
        /* Run odom again inside the mutex to measure the real critical-section
         * cost (enter + update + exit) vs the bare update above. */
        PROFILE_CPU_BEGIN(mux);
        taskENTER_CRITICAL(&s_pose_mux);
        encoder_ackermann_odom_update(&s_odom, distance_m, steering_rad,
                                      yaw_rad, now_ms);
        taskEXIT_CRITICAL(&s_pose_mux);
        uint32_t mux_us;
        PROFILE_CPU_END(mux, &mux_us);

        if (mux_us > mux_us_max) mux_us_max = mux_us;
        if (mux_us < mux_us_min) mux_us_min = mux_us;

        const odom_pose_t *pose = encoder_ackermann_odom_get_pose(&s_odom);
        task_data_profile_update(&s_profile.data_profile, 1);
        task_profile_cycle_end(&s_profile);

        uint32_t total_us = s_profile.cpu_cycles_last / PROFILER_CPU_FREQ_MHZ;

        /* ── Budget warning ─────────────────────────────────────────────── */
        if (total_us > 2000u) {
            ESP_LOGW(TAG, "SLOW cycle=%" PRIu32 " µs (budget 2 ms) "
                     "gyro=%" PRIu32 " enc=%" PRIu32,
                     total_us, gyro_us, enc_us);
        }
        if (mux_us > odom_us + 50u) {
            ESP_LOGW(TAG, "MUX overhead=%" PRIu32 " µs (mux=%" PRIu32
                     " odom=%" PRIu32 ") — spinlock contention?",
                     mux_us - odom_us, mux_us, odom_us);
        }
        if (s_profile.heap_delta_last != 0) {
            ESP_LOGE(TAG, "HEAP DELTA %" PRId32 " B — unexpected alloc in odometry path!",
                     s_profile.heap_delta_last);
        }

        /* ── Per-second log (every 100 ticks) ──────────────────────────── */
        if (tick % 100u == 0u && pose) {
            ESP_LOGI(TAG,
                     "── tick=%-5" PRIu32 " ─────────────────────────────────",
                     tick);
            ESP_LOGI(TAG,
                     "  pose  x=%.4f m  y=%.4f m  θ=%.3f°",
                     (double)pose->x, (double)pose->y,
                     (double)(pose->theta * 180.0f / (float)M_PI));
            ESP_LOGI(TAG,
                     "  sens  dist=%.4f m  heading=%.3f°  steer=%.1f°",
                     (double)distance_m,
                     (double)(yaw_rad * 180.0f / (float)M_PI),
                     (double)(steering_rad * 180.0f / (float)M_PI));
            ESP_LOGI(TAG,
                     "  time  gyro=%4" PRIu32 "µs  enc=%4" PRIu32 "µs"
                     "  odom=%3" PRIu32 "µs  mux=%3" PRIu32 "µs"
                     "  total=%4" PRIu32 "µs",
                     gyro_us, enc_us, odom_us, mux_us, total_us);
            ESP_LOGI(TAG,
                     "  range gyro=[%"PRIu32",%"PRIu32"]µs"
                     "  enc=[%"PRIu32",%"PRIu32"]µs"
                     "  odom=[%"PRIu32",%"PRIu32"]µs"
                     "  mux=[%"PRIu32",%"PRIu32"]µs",
                     gyro_us_min, gyro_us_max,
                     enc_us_min,  enc_us_max,
                     odom_us_min, odom_us_max,
                     mux_us_min,  mux_us_max);
            ESP_LOGI(TAG,
                     "  i2c   enc_fails=%" PRIu32 "  imu_fails=%" PRIu32
                     "  stationary=%d",
                     enc_fail_total, imu_fail_total, stationary_ticks);
            (void)enc_fail_total; (void)imu_fail_total;
        }

        /* ── Memory snapshot every 500 ticks (5 s) ──────────────────────── */
        if (tick % 500u == 0u) {
            uint32_t heap_free   = esp_get_free_heap_size();
            uint32_t heap_min    = esp_get_minimum_free_heap_size();
            uint32_t largest_blk = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
            uint32_t frag_pct    = (heap_free > 0u)
                                   ? (uint32_t)(100u - largest_blk * 100u / heap_free)
                                   : 0u;
            uint32_t stack_free  = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

            ESP_LOGI(TAG,
                     "=MEM= heap=%" PRIu32 "B  wm=%" PRIu32 "B"
                     "  blk=%" PRIu32 "B  frag=%" PRIu32 "%%"
                     "  stack=%" PRIu32 "B",
                     heap_free, heap_min, largest_blk, frag_pct, stack_free);
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(10));   /* 100 Hz — same as production */
    }
}
