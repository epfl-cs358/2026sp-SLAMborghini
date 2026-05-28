/**
 * frontier_detector.c
 * Module: Frontier-based exploration detector.
 * Board: ESP32-S3
 *
 * Strategy: Wavefront Frontier Detection (WFD)
 *
 *   1. BFS outward from the robot's cell through free cells only.
 *      (Free cells are the only cells that can contain frontiers, so we
 *       never waste time scanning unknown or occupied regions.)
 *
 *   2. Each free cell popped from the queue is tested: if it has at least
 *      one 4-connected unknown neighbour it is a frontier seed.
 *
 *   3. Each new seed triggers a 4-connected flood-fill (cluster_frontier)
 *      that collects the full connected frontier region into s_cbuf[].
 *
 *   4. Target point: compute the cluster centroid, then find the cluster
 *      cell closest to it. This is always an actual free cell — unlike a
 *      raw centroid that can land inside an obstacle.
 *      (Topiwala et al. WFD — use median/nearest-to-centroid, not raw centroid.)
 *
 *   5. Safety spiral: if any 8-connected neighbour of the target cell is
 *      not free, step outward ring by ring until a fully-safe cell is found.
 *      (Adapted from SLAMaleykoum mission_planner.cpp — proven on hardware.)
 *
 *   6. Clusters smaller than MIN_CLUSTER_SIZE are discarded (scan noise).
 *
 * Complexity: O(F) where F = reachable free cells — far cheaper than the
 * O(R^2) bounding-box scan used in SLAMaleykoum, especially at startup
 * when the free bubble is small.
 *
 * Memory: all buffers are static — no heap allocation, safe on ESP32-S3.
 *
 * Reference: Topiwala, Maini, Bhatt — "Frontier Based Exploration for
 *            Autonomous Robot" (WFD variant).
 */

#include "frontier_detector.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ── Occupancy thresholds (matching quadtree_map conventions) ─────────────
 * quadtree_map_query maps int8 raw log-odds to uint8: query = raw + 128.
 *
 *   query ≤ 115 : free      (raw ≤ -13, ~7 MISS sweeps at MISS_DEC=-2)
 *   120 – 178   : unknown   (raw -8…+50; never-observed centre = 128)
 *   179 – 255   : occupied  (raw > 50;   2 HITs at HIT_INC=30 → raw=60)
 *   gap 116-119 : hysteresis — neither free nor unknown; BFS won't expand
 *
 * OCC_FREE_MAX raised from 50→115 so that cells swept ~7 times in one
 * scan register as free immediately instead of after 39 sweeps.  This
 * eliminates the "unknown cell in front of every wall" phantom-frontier
 * artifact that came from the old threshold requiring raw ≤ -78.
 * ──────────────────────────────────────────────────────────────────────── */
#define OCC_FREE_MAX    115u
#define OCC_UNK_MIN     120u
#define OCC_UNK_MAX     178u

/* ── Robot footprint ─────────────────────────────────────────────────────────
 * Body: 394 mm long × 295 mm wide.  Half-diagonal = sqrt(197²+147.5²) ≈ 246 mm.
 * Grid resolution ≈ 156 mm per leaf cell.  246/156 = 1.58 → ceil = 2 cells.
 * A 2-cell Chebyshev radius (5×5 box) guarantees clearance on all sides.
 * SPIRAL_STEPS must exceed ROBOT_CLEAR_CELLS so the spiral has enough room to
 * find a passing candidate in narrow corridors. ──────────────────────────── */
#define ROBOT_CLEAR_CELLS  2    /* Chebyshev clearance radius in grid cells  */

/* ── Tuning constants ────────────────────────────────────────────────────── */
#define MIN_CLUSTER_SIZE    5    /* discard clusters with fewer frontier free cells */
#define MIN_ADJACENT_UNK    6    /* min unknown-cell touches on the cluster —
                                  * wall-shadow phantoms: 1 unknown/cell → ~5 total
                                  * real frontiers:      2-3 unknown/cell → 10-20
                                  * threshold 6 rejects phantoms, keeps real walls  */
#define CLEARANCE_RADIUS    3    /* Chebyshev radius (cells) for open-space check */
#define MIN_CLEARANCE_CELLS 12   /* min free cells in the (2R+1)²=49 box around
                                  * the target — 12/49≈25% keeps 2-cell corridors
                                  * (≥14 free) while rejecting 1-cell tunnels (7) */
#define BFS_QUEUE_CAP   2048    /* WFD BFS ring-buffer capacity              */
#define CLUSTER_CAP      128    /* max cells collected per frontier cluster  */
#define SPIRAL_STEPS       8    /* safety spiral max search radius — must be */
                                /* > ROBOT_CLEAR_CELLS so narrow corridors   */
                                /* can still find a safe cell                */

/* ── Grid dimension limits  (10 000 mm / 50 mm = 200 cells per axis) ─────── */
#define MAX_GRID_W  200
#define MAX_GRID_H  200
#define VISITED_BYTES  ((MAX_GRID_W * MAX_GRID_H + 7) / 8)   /* 5000 bytes */

/* ── Internal cell coordinate (int16 keeps the queue at 4 bytes/entry) ────── */
typedef struct { int16_t ix; int16_t iy; } cell_t;

/* ── Static buffers — allocated once in BSS, cleared per call ──────────────
 *   s_vis  : WFD BFS visited bits     (~5 KB)
 *   s_clu  : cluster BFS visited bits (~5 KB)
 *   s_bfs  : WFD BFS ring buffer      (~8 KB)
 *   s_cbuf : current cluster cells    (~0.5 KB)
 *   Total  : ~18.5 KB — well within ESP32-S3's 512 KB SRAM.
 * ──────────────────────────────────────────────────────────────────────── */
static uint8_t  s_vis[VISITED_BYTES];
static uint8_t  s_clu[VISITED_BYTES];
static cell_t   s_bfs[BFS_QUEUE_CAP];
static cell_t   s_cbuf[CLUSTER_CAP];

/* ── 4-connected neighbour offsets ──────────────────────────────────────── */
static const int8_t K4X[4] = {  1, -1,  0,  0 };
static const int8_t K4Y[4] = {  0,  0,  1, -1 };


/* ═══════════════════════ BIT-ARRAY HELPERS ════════════════════════════════ */

static inline bool bit_get(const uint8_t *a, int ix, int iy)
{
    unsigned idx = (unsigned)(iy * MAX_GRID_W + ix);
    return (a[idx >> 3] >> (idx & 7u)) & 1u;
}

static inline void bit_set(uint8_t *a, int ix, int iy)
{
    unsigned idx = (unsigned)(iy * MAX_GRID_W + ix);
    a[idx >> 3] |= (uint8_t)(1u << (idx & 7u));
}

/* ═══════════════════════ GRID HELPERS ═════════════════════════════════════ */

/* Bounds check using unsigned cast — avoids two comparisons */
static inline bool in_bounds(int ix, int iy, int mw, int mh)
{
    return (unsigned)ix < (unsigned)mw && (unsigned)iy < (unsigned)mh;
}

/* Cell centre position in mm (query at cell centre, not corner) */
static inline float cx_mm(int ix, float res) { return (ix + 0.5f) * res; }
static inline float cy_mm(int iy, float res) { return (iy + 0.5f) * res; }

/* ═══════════════════════ OCCUPANCY CATEGORY HELPERS ══════════════════════ */

static inline bool cell_is_free(const quadtree_map_t *m, float x, float y)
{
    return quadtree_map_query(m, x, y) <= OCC_FREE_MAX;
}

static inline bool cell_is_unknown(const quadtree_map_t *m, float x, float y)
{
    uint8_t v = quadtree_map_query(m, x, y);
    return v >= OCC_UNK_MIN && v <= OCC_UNK_MAX;
}

static inline bool cell_is_occupied(const quadtree_map_t *m, float x, float y)
{
    return quadtree_map_query(m, x, y) > OCC_UNK_MAX;
}

/* ═══════════════════════ is_frontier ══════════════════════════════════════
 * Returns true if cell (ix, iy) is free AND has at least one 4-connected
 * neighbour that is unknown. This is the canonical WFD frontier definition.
 * ══════════════════════════════════════════════════════════════════════════ */
static bool is_frontier(const quadtree_map_t *m, int ix, int iy,
                         float res, int mw, int mh)
{
    if (!cell_is_free(m, cx_mm(ix, res), cy_mm(iy, res))) return false;

    for (int k = 0; k < 4; k++) {
        int nx = ix + K4X[k];
        int ny = iy + K4Y[k];
        if (!in_bounds(nx, ny, mw, mh)) continue;
        if (cell_is_unknown(m, cx_mm(nx, res), cy_mm(ny, res))) return true;
    }
    return false;
}

/* ═══════════════════════ is_safe_cell ═════════════════════════════════════
 * Returns true if every cell within ROBOT_CLEAR_CELLS Chebyshev radius of
 * (ix, iy) is free — i.e., the full (2R+1)² box is obstacle-free.
 * This guarantees the robot body (394×295 mm, half-diagonal ≈ 246 mm) clears
 * walls when the navigation target is placed at this cell.
 * ══════════════════════════════════════════════════════════════════════════ */
static bool is_safe_cell(const quadtree_map_t *m, int ix, int iy,
                          float res, int mw, int mh)
{
    for (int dy = -ROBOT_CLEAR_CELLS; dy <= ROBOT_CLEAR_CELLS; dy++) {
        for (int dx = -ROBOT_CLEAR_CELLS; dx <= ROBOT_CLEAR_CELLS; dx++) {
            int nx = ix + dx;
            int ny = iy + dy;
            if (!in_bounds(nx, ny, mw, mh)) continue;
            /* Reject if any cell in the box is not confirmed free (unknown or occupied).
             * This pushes the target 2 cells away from walls AND away from wall-shadow
             * unknown cells, so the robot never drives right up to a wall. */
            if (!cell_is_free(m, cx_mm(nx, res), cy_mm(ny, res))) return false;
        }
    }
    return true;
}

/* ═══════════════════════ safety_spiral ════════════════════════════════════
 * If the chosen cell is unsafe (any 8-neighbour not free), expand outward
 * ring by ring up to SPIRAL_STEPS cells and adopt the first safe cell found.
 * Updates *ix, *iy in place. Keeps original if nothing better is found.
 *
 * Adapted from SLAMaleykoum get_safe_neighbor() — proven on hardware.
 * ══════════════════════════════════════════════════════════════════════════ */
/* Returns true if a safe cell was found (ix/iy updated), false if the
 * spiral exhausted all candidates — caller must skip this frontier entirely. */
static bool safety_spiral(const quadtree_map_t *m, int *ix, int *iy,
                           float res, int mw, int mh)
{
    if (is_safe_cell(m, *ix, *iy, res, mw, mh)) return true;

    for (int r = 1; r <= SPIRAL_STEPS; r++) {
        for (int dy = -r; dy <= r; dy++) {
            for (int dx = -r; dx <= r; dx++) {
                /* Visit only the outermost ring at radius r */
                if (abs(dx) != r && abs(dy) != r) continue;
                int nx = *ix + dx;
                int ny = *iy + dy;
                if (!in_bounds(nx, ny, mw, mh)) continue;
                if (!cell_is_free(m, cx_mm(nx, res), cy_mm(ny, res))) continue;
                if (is_safe_cell(m, nx, ny, res, mw, mh)) {
                    *ix = nx;
                    *iy = ny;
                    return true;
                }
            }
        }
    }
    return false;   /* no safe cell found — drop this frontier */
}

/* ═══════════════════════ count_adjacent_unknown ════════════════════════════
 * Count unknown cells that are 4-connected neighbours of the cluster in s_cbuf.
 * Used to reject map-artifact holes: a real frontier wall has many adjacent
 * unknown cells; an isolated hole artifact has only 1-2.
 * May double-count cells shared by adjacent cluster members — that's fine,
 * since we only need the count to be above a threshold, not exact.
 * ══════════════════════════════════════════════════════════════════════════ */
static int count_adjacent_unknown(const quadtree_map_t *m, int cnt,
                                   float res, int mw, int mh)
{
    int total = 0;
    for (int i = 0; i < cnt; i++) {
        for (int k = 0; k < 4; k++) {
            int nx = s_cbuf[i].ix + K4X[k];
            int ny = s_cbuf[i].iy + K4Y[k];
            if (!in_bounds(nx, ny, mw, mh)) continue;
            if (cell_is_unknown(m, cx_mm(nx, res), cy_mm(ny, res))) total++;
        }
    }
    return total;
}

/* ═══════════════════════ count_free_in_radius ══════════════════════════════
 * Count free cells within a Chebyshev radius of (ix, iy).
 * Used as an open-space quality score: high count = large navigable area
 * around the target; low count = tight corner or narrow dead-end.
 * ══════════════════════════════════════════════════════════════════════════ */
static int count_free_in_radius(const quadtree_map_t *m, int ix, int iy,
                                 int radius, float res, int mw, int mh)
{
    int count = 0;
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            int nx = ix + dx;
            int ny = iy + dy;
            if (!in_bounds(nx, ny, mw, mh)) continue;
            if (cell_is_free(m, cx_mm(nx, res), cy_mm(ny, res))) count++;
        }
    }
    return count;
}

/* ═══════════════════════ cluster_frontier ═════════════════════════════════
 * 4-connected flood-fill from seed (sx, sy) over cells that pass is_frontier.
 * Stores visited cells in s_cbuf[] and marks them in s_clu[].
 * Returns the number of cells found (capped at CLUSTER_CAP).
 *
 * Uses a local 512-byte stack queue — small enough for ESP32-S3 tasks.
 * ══════════════════════════════════════════════════════════════════════════ */
static int cluster_frontier(const quadtree_map_t *m, int sx, int sy,
                             float res, int mw, int mh)
{
    cell_t lq[CLUSTER_CAP];    /* local queue — 512 bytes on stack */
    int head = 0, tail = 0, count = 0;

    lq[tail++] = (cell_t){ (int16_t)sx, (int16_t)sy };
    bit_set(s_clu, sx, sy);

    while (head != tail && count < CLUSTER_CAP) {
        cell_t c = lq[head++];
        s_cbuf[count++] = c;

        for (int k = 0; k < 4; k++) {
            int nx = c.ix + K4X[k];
            int ny = c.iy + K4Y[k];
            if (!in_bounds(nx, ny, mw, mh))        continue;
            if (bit_get(s_clu, nx, ny))             continue;
            if (!is_frontier(m, nx, ny, res, mw, mh)) continue;
            if (tail >= CLUSTER_CAP)                continue; /* queue full */
            lq[tail++] = (cell_t){ (int16_t)nx, (int16_t)ny };
            bit_set(s_clu, nx, ny);
        }
    }
    return count;
}

/* ═══════════════════════ nearest_to_centroid ══════════════════════════════
 * Finds the cell in s_cbuf[0..count) closest to the cluster centroid.
 * Returns its map coordinates in mm via *out_ix / *out_iy.
 *
 * Why not use the raw centroid?
 *   The centroid is the arithmetic mean of cell positions. It can fall
 *   inside an obstacle or unknown cell (e.g., a concave frontier).
 *   The cell nearest to the centroid is always an actual free cell,
 *   so it is always reachable. (Topiwala et al. WFD recommendation.)
 * ══════════════════════════════════════════════════════════════════════════ */
static void nearest_to_centroid(int count, int *out_ix, int *out_iy)
{
    /* Compute centroid in cell-index space */
    float sum_x = 0.0f, sum_y = 0.0f;
    for (int i = 0; i < count; i++) {
        sum_x += s_cbuf[i].ix;
        sum_y += s_cbuf[i].iy;
    }
    float cen_x = sum_x / (float)count;
    float cen_y = sum_y / (float)count;

    /* Find closest cluster cell to centroid */
    float best_d = 1e9f;
    int   best_i = 0;
    for (int i = 0; i < count; i++) {
        float dx = (float)s_cbuf[i].ix - cen_x;
        float dy = (float)s_cbuf[i].iy - cen_y;
        float d  = dx * dx + dy * dy;
        if (d < best_d) { best_d = d; best_i = i; }
    }
    *out_ix = s_cbuf[best_i].ix;
    *out_iy = s_cbuf[best_i].iy;
}

/* ══════════════════════════════════════════════════════════════════════════
 * frontier_detector_detect
 *
 * WFD BFS from robot pose outward through free cells.
 * Returns up to 32 valid frontier clusters, each with a safe target point.
 * ══════════════════════════════════════════════════════════════════════════ */
frontier_list_t frontier_detector_detect(const quadtree_map_t *map,
                                          const pose_t *robot_pose)
{
    frontier_list_t result = {0};

    if (!map || !robot_pose) return result;

    /* Derive resolution from bounds (quadtree_map_init step_mm = width/MAX_GRID_W).
     * The new QuadTreeMap stores x_min/x_max/y_min/y_max instead of resolution_mm. */
    float map_w = map->x_max - map->x_min;
    float map_h = map->y_max - map->y_min;
    float res   = map_w / (float)MAX_GRID_W;
    if (res <= 0.0f) return result;

    /* Grid dimensions — clamped to static buffer limits */
    int mw = MAX_GRID_W;
    int mh = (int)(map_h / res);
    if (mh > MAX_GRID_H) mh = MAX_GRID_H;

    /* Clear visited arrays for this call */
    memset(s_vis, 0, sizeof(s_vis));
    memset(s_clu, 0, sizeof(s_clu));

    /* Convert robot pose (mm) to grid cell index — offset by map origin */
    int rx = (int)((robot_pose->x - map->x_min) / res);
    int ry = (int)((robot_pose->y - map->y_min) / res);
    if (!in_bounds(rx, ry, mw, mh)) return result;

    /* Robot must be in a free cell — if not, the map isn't ready yet */
    if (!cell_is_free(map, cx_mm(rx, res), cy_mm(ry, res))) return result;

    /* ── WFD BFS ──────────────────────────────────────────────────────── */
    int head = 0, tail = 0;
    s_bfs[tail++] = (cell_t){ (int16_t)rx, (int16_t)ry };
    bit_set(s_vis, rx, ry);

    /* "Behind" fallback: used only when no forward-facing frontier passes all
     * filters (e.g. robot pushed into a corner with only unknown behind it). */
    int fallback_tix = -1, fallback_tiy = -1, fallback_clr = 0;

    while (head != tail) {
        cell_t c = s_bfs[head];
        head = (head + 1) % BFS_QUEUE_CAP;

        /* ── Frontier check ─────────────────────────────────────────── */
        if (is_frontier(map, c.ix, c.iy, res, mw, mh) &&
            !bit_get(s_clu, c.ix, c.iy))
        {
            int cnt = cluster_frontier(map, c.ix, c.iy, res, mw, mh);

            /* Filter 1: cluster size — discard scan noise */
            if (cnt < MIN_CLUSTER_SIZE) goto next_bfs;

            /* Filter 2: hole rejection — real frontiers border many unknown
             * cells; isolated map artifacts (flickering holes) border only 1-2. */
            if (count_adjacent_unknown(map, cnt, res, mw, mh) < MIN_ADJACENT_UNK)
                goto next_bfs;

            int tix, tiy;
            nearest_to_centroid(cnt, &tix, &tiy);
            if (!safety_spiral(map, &tix, &tiy, res, mw, mh)) goto next_bfs;

            /* Filter 3: open-space clearance — reject tight spots the robot
             * cannot comfortably navigate (1-cell tunnels, blind corners).
             * The clearance count also serves as the ranking score in
             * frontier_detector_best: more free space = higher priority. */
            int clr = count_free_in_radius(map, tix, tiy,
                                            CLEARANCE_RADIUS, res, mw, mh);
            if (clr < MIN_CLEARANCE_CELLS) goto next_bfs;

            /* Filter 4: forward half-plane — prefer no reversing. */
            float fdx = cx_mm(tix, res) - cx_mm(rx, res);
            float fdy = cy_mm(tiy, res) - cy_mm(ry, res);
            bool ahead = (fdx * cosf(robot_pose->theta) +
                          fdy * sinf(robot_pose->theta) >= 0.0f);

            if (ahead) {
                if (result.count < 32) {
                    result.items[result.count].cx   = cx_mm(tix, res);
                    result.items[result.count].cy   = cy_mm(tiy, res);
                    result.items[result.count].size = (uint8_t)(clr > 255 ? 255 : clr);
                    result.count++;
                }
                /* 8 forward frontiers is plenty for the selector — stop BFS
                 * early to bound the time the map mutex is held. */
                if (result.count >= 8) goto bfs_done;
            } else if (fallback_tix < 0) {
                fallback_tix = tix;
                fallback_tiy = tiy;
                fallback_clr = clr;
            }
        }
        next_bfs:;

        /* ── BFS expansion ──────────────────────────────────────────── */
        /* Expand only through free cells (frontier cells are free too,
         * so they are naturally included and will be processed above). */
        for (int k = 0; k < 4; k++) {
            int nx = c.ix + K4X[k];
            int ny = c.iy + K4Y[k];
            if (!in_bounds(nx, ny, mw, mh))                   continue;
            if (bit_get(s_vis, nx, ny))                        continue;
            if (!cell_is_free(map, cx_mm(nx, res), cy_mm(ny, res))) continue;

            int next_tail = (tail + 1) % BFS_QUEUE_CAP;
            if (next_tail == head) continue;   /* queue full — skip, not crash */
            s_bfs[tail] = (cell_t){ (int16_t)nx, (int16_t)ny };
            tail = next_tail;
            bit_set(s_vis, nx, ny);
        }
    }
    bfs_done:;

    /* No "ahead" frontier passed all filters — fall back to the nearest
     * behind-robot frontier so exploration doesn't deadlock. */
    if (result.count == 0 && fallback_tix >= 0) {
        result.items[0].cx   = cx_mm(fallback_tix, res);
        result.items[0].cy   = cy_mm(fallback_tiy, res);
        result.items[0].size = (uint8_t)(fallback_clr > 255 ? 255 : fallback_clr);
        result.count = 1;
    }
    return result;
}

/* ══════════════════════════════════════════════════════════════════════════
 * frontier_detector_best
 *
 * Score: U(f) = clearance / distance_to_robot
 *   - clearance  free cells within CLEARANCE_RADIUS of the target — higher
 *                means more open navigable space (stored in frontier_t.size)
 *   - distance   travel cost approximation
 *
 * Prefers frontiers with large open space nearby that are not too far away.
 * Holes and tight spots are already filtered in frontier_detector_detect,
 * so every candidate here is a legitimate, safely reachable target.
 * ══════════════════════════════════════════════════════════════════════════ */
frontier_t frontier_detector_best(const frontier_list_t *list,
                                   const pose_t *robot_pose)
{
    frontier_t best = {0};
    if (!list || !robot_pose || list->count == 0) return best;

    float best_score = -1.0f;

    for (uint8_t i = 0; i < list->count; i++) {
        const frontier_t *f = &list->items[i];
        float dx   = f->cx - robot_pose->x;
        float dy   = f->cy - robot_pose->y;
        float dist = sqrtf(dx * dx + dy * dy);

        /* Clamp minimum distance to avoid division by zero when the robot
         * is already sitting on a frontier cell (startup edge case). */
        if (dist < 1.0f) dist = 1.0f;

        float score = (float)f->size / dist;
        if (score > best_score) {
            best_score = score;
            best       = *f;
        }
    }
    return best;
}


/* ══════════════════════════════════════════════════════════════════════════
 * frontier_selector_pick  —  Option C: heading-aligned tiered selection
 *
 * Four tiers of angular tolerance (widening cone) with distance cap:
 *   Tier 0: ±30°,  max 3000 mm — prefer nearby, aligned frontiers
 *   Tier 1: ±60°,  max 4000 mm — widen if tier 0 empty
 *   Tier 2: ±120°, max 6000 mm — widen further
 *   Tier 3: ±180°, no cap      — guaranteed fallback (no feasibility check)
 *
 * Tiers 0-2 apply a three-part curvature feasibility pre-filter:
 *   1. Steering reach  — bicycle model: dist >= 2*R_min*|sin(heading_err)|
 *   2. Corridor width  — lateral clearance at 200 mm ahead >= 300 mm total
 *   3. Rollout         — < 2 of 4 cells along robot→frontier are occupied
 *
 * Tier 3 skips feasibility to prevent permanent deadlock.
 * Within a passing tier the nearest candidate is returned.
 * ══════════════════════════════════════════════════════════════════════════ */

/* Bicycle model parameters */
#define _FS_WHEELBASE_MM    258.0f
#define _FS_MAX_STEER_RAD   1.134f   /* ~65° measured from hardware */

/* Corridor width check */
#define _FS_WIDTH_MIN_MM    300.0f   /* minimum total corridor width */
#define _FS_LATERAL_FWD_MM  200.0f   /* look-ahead distance along heading */
#define _FS_LATERAL_STEP_MM  50.0f   /* lateral probe step size */
#define _FS_LATERAL_STEPS      4     /* steps each side = 200 mm max */

/* Mini rollout */
#define _FS_ROLLOUT_STEPS      4     /* probe points */
#define _FS_ROLLOUT_STEP_MM  200.0f  /* spacing of probe points */
#define _FS_ROLLOUT_MAX_OCC    2     /* reject if 2+ occupied cells on direct path */

static float _fs_wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* Bicycle model: can the robot reach the frontier without exceeding max
 * steering angle?  Required turning arc: dist >= 2*R_min*|sin(delta)|. */
static bool _fs_check_steer(const pose_t *robot, const frontier_t *f)
{
    float dx = f->cx - robot->x;
    float dy = f->cy - robot->y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 1.0f) return true;
    float goal_hdg = atan2f(dy, dx);
    float herr = fabsf(_fs_wrap_pi(goal_hdg - robot->theta));
    /* R_min = L / tan(max_steer); precomputed at compile time */
    float r_min = _FS_WHEELBASE_MM / tanf(_FS_MAX_STEER_RAD);
    return dist >= 2.0f * r_min * sinf(herr);
}

/* Corridor width: probe laterally at _FS_LATERAL_FWD_MM ahead and count
 * free steps on each side before hitting an obstacle. */
static bool _fs_check_width(const pose_t *robot, const frontier_t *f,
                              const quadtree_map_t *map)
{
    float dx = f->cx - robot->x;
    float dy = f->cy - robot->y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 1.0f) return true;

    float ux = dx / dist, uy = dy / dist;   /* unit vec toward frontier */
    float lx = -uy,       ly =  ux;         /* left perpendicular (+90°) */

    float px = robot->x + ux * _FS_LATERAL_FWD_MM;
    float py = robot->y + uy * _FS_LATERAL_FWD_MM;

    int lw = 0, rw = 0;
    for (int s = 1; s <= _FS_LATERAL_STEPS; s++) {
        float step = (float)s * _FS_LATERAL_STEP_MM;
        if (qt_query_const(map, px + lx * step, py + ly * step) >= QT_OCC_CAUTION) break;
        lw++;
    }
    for (int s = 1; s <= _FS_LATERAL_STEPS; s++) {
        float step = (float)s * _FS_LATERAL_STEP_MM;
        if (qt_query_const(map, px - lx * step, py - ly * step) >= QT_OCC_CAUTION) break;
        rw++;
    }
    return ((float)(lw + rw) * _FS_LATERAL_STEP_MM) >= _FS_WIDTH_MIN_MM;
}

/* Rollout: sample 4 cells along the straight line robot→frontier.
 * Reject if 2 or more are occupied. */
static bool _fs_check_rollout(const pose_t *robot, const frontier_t *f,
                               const quadtree_map_t *map)
{
    float dx = f->cx - robot->x;
    float dy = f->cy - robot->y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 1.0f) return true;

    float ux = dx / dist, uy = dy / dist;
    int occ = 0;
    for (int s = 1; s <= _FS_ROLLOUT_STEPS; s++) {
        float step = (float)s * _FS_ROLLOUT_STEP_MM;
        if (step >= dist) break;
        if (qt_query_const(map, robot->x + ux * step, robot->y + uy * step) >= QT_OCC_CAUTION)
            occ++;
    }
    return occ < _FS_ROLLOUT_MAX_OCC;
}

static bool _fs_feasible(const pose_t *robot, const frontier_t *f,
                          const quadtree_map_t *map)
{
    return _fs_check_steer(robot, f) &&
           _fs_check_width(robot, f, map) &&
           _fs_check_rollout(robot, f, map);
}

typedef struct { float half_angle_rad; float max_dist_mm; } _fs_tier_t;

frontier_t frontier_selector_pick(const frontier_list_t *list,
                                   const pose_t *robot,
                                   const quadtree_map_t *map)
{
    if (!list || !robot || list->count == 0) {
        frontier_t z = {0}; return z;
    }

    static const _fs_tier_t tiers[] = {
        {  30.0f * (float)M_PI / 180.0f, 3000.0f },
        {  60.0f * (float)M_PI / 180.0f, 4000.0f },
        { 120.0f * (float)M_PI / 180.0f, 6000.0f },
        { (float)M_PI,                   1e9f     },  /* fallback — no feasibility */
    };
    const int N_TIERS = (int)(sizeof(tiers) / sizeof(tiers[0]));

    for (int t = 0; t < N_TIERS; t++) {
        float best_dist = 1e9f;
        int   best_idx  = -1;
        bool  do_feasibility = (t < N_TIERS - 1);

        for (uint8_t i = 0; i < list->count; i++) {
            const frontier_t *f = &list->items[i];
            float dx = f->cx - robot->x;
            float dy = f->cy - robot->y;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist < 1.0f) dist = 1.0f;

            if (dist > tiers[t].max_dist_mm) continue;

            float goal_hdg = atan2f(dy, dx);
            float herr = fabsf(_fs_wrap_pi(goal_hdg - robot->theta));
            if (herr > tiers[t].half_angle_rad) continue;

            if (do_feasibility && !_fs_feasible(robot, f, map)) continue;

            if (dist < best_dist) {
                best_dist = dist;
                best_idx  = i;
            }
        }

        if (best_idx >= 0) return list->items[best_idx];
    }

    return list->items[0];   /* should never reach: tier 3 = ±180° with no cap */
}
