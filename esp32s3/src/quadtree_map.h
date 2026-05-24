/**
 * quadtree_map.h
 * Module: Quadtree occupancy map.
 * Board: ESP32-S3
 * Provides a hierarchical spatial map structure for storing occupancy and semantic
 * class information across the robot's operational environment.
 */

#ifndef QUADTREE_MAP_H
#define QUADTREE_MAP_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>



#define QT_MAX_DEPTH 7   /* depth 7 → 2^6=64 splits/axis → 156 mm leaf cells on 10 m map */

// Node pool size (each QTNode = 12 bytes after alignment).
// Depth 7: worst-case full tree needs ~5461 nodes (4096 leaves + internal).
//
// ESP32-S3 D-cache = 16 KB = 1365 nodes max before cache pressure.
// Phase 1 (fixed pose, 1500 mm active radius) uses ~400 nodes = 4.8 KB.
// 1200 nodes = 14.4 KB — fits in D-cache, 3× headroom over Phase 1.
// If the robot moves extensively and the pool fills, qt_is_pool_full()
// returns true — caller must reset with qt_free() + qt_init().
//
// Wemos (ESP32): no WiFi on the map path, cache pressure is lower;
//   6000 nodes kept for full-room SLAM headroom.
#ifndef QT_POOL_SIZE
#  ifdef CONFIG_IDF_TARGET_ESP32S3
#    define QT_POOL_SIZE 4000   /* 4000 × 12 B = 48 KB; supports compaction cycle */
#  else
#    define QT_POOL_SIZE 6000
#  endif
#endif
#define QT_NULL 0 // no child index 

// Log-odds increments
// HIT:MISS ratio of 15:1 — walls resist erosion from stray free-space sweeps.
// One HIT (+15) needs 15 MISS passes to erase.  Walls still saturate quickly
// (each scan delivers dozens of HIT reinforcements along any visible surface).
// Halving from 30→15 lets ghost obstacles from rare bad scans clear in ~1 scan
// rather than accumulating until the robot revisits the same exact beam angle.
#define QT_HIT_INC 15   // obstacle confirmed → value rises
#define QT_MISS_DEC (-2)  // ray passed through → value drops
#define QT_VALUE_MAX 40
#define QT_VALUE_MIN (-40)

// Node 
typedef struct {
    uint16_t children[4]; // pool indices; QT_NULL = absent 
    int8_t value; // log-odds occupancy (meaningful at leaves) 
    uint8_t depth; // depth in tree (root = 1) 
} QTNode;


typedef struct {
    QTNode *pool; /* static BSS on device; heap on host (tests only) */
    uint16_t count; /* next free slot */
    float x_min, x_max;
    float y_min, y_max;
} QuadTreeMap;


// Initialise a map covering [x_min,x_max] × [y_min,y_max] and 
// allocates the node pool with malloc + call qt_free() when done 
void qt_init(QuadTreeMap *map,
             float x_min, float x_max,
             float y_min, float y_max);

// free the node pool                                             
void qt_free(QuadTreeMap *map);

// update the log-odds value at world position (x, y).
// Pass QT_HIT_INC for an obstacle hit/ QT_MISS_DEC for a free ray 
void qt_update(QuadTreeMap *map, float x, float y, int8_t delta);

//Return the log-odds value at (x, y).
//Returns 0 if never observed/out of bounds

int8_t qt_query(QuadTreeMap *map, float x, float y);

// qt_iterate_occupied: Initiates traversal of occupied cells cad value > 0).
//cb receives the cell centre (cx, cy)+ its value+ userdata 
void qt_iterate_occupied(QuadTreeMap *map,
                         void (*cb)(float cx, float cy,
                                    int8_t value, void *userdata),
                         void *userdata);

// qt_memory_bytes: returns memory usage of allocated nodes.
size_t qt_memory_bytes(const QuadTreeMap *map);

// qt_is_pool_full: true once no more nodes can be allocated (map is frozen).
static inline bool qt_is_pool_full(const QuadTreeMap *map) {
    return map && map->pool && map->count >= QT_POOL_SIZE;
}

/* qt_compact — snapshot confident walls, wipe the pool, re-insert.
 *
 * Collects every leaf with value >= min_value via qt_iterate_occupied,
 * resets the pool in-place (no malloc/free), then re-inserts the saved
 * cells so the quadtree retains all confident wall knowledge while
 * freeing all uncertain/noise nodes.
 *
 * Call when map->count > QT_POOL_SIZE * 85 / 100 to prevent freezing.
 * Safe to call from the same task that writes the map (no mutex needed). */
void qt_compact(QuadTreeMap *map, int8_t min_value);

// qt_query const variant — does not modify the map.
int8_t qt_query_const(const QuadTreeMap *map, float x, float y);


/* ════════════════════════════════════════════════════════════════════════════
 * Compatibility layer — keeps frontier_detector, test_room, wifi_dashboard,
 * and map_updater compiling without renaming every call site.
 *
 * Legacy API:
 *   quadtree_map_t                           → typedef for QuadTreeMap
 *   quadtree_map_init(map, w_mm, h_mm, step) → qt_init(0, w, 0, h)
 *   quadtree_map_insert(map, x, y, cls)      → qt_update with correct delta
 *   quadtree_map_query(map, x, y) → uint8_t  → mapped from log-odds int8_t
 *
 * Log-odds → uint8_t thresholds (match frontier_detector.c constants):
 *   log-odds < 0  (free)    → 20   ≤ OCC_FREE_MAX  (50)
 *   log-odds == 0 (unknown) → 128  ∈ OCC_UNK range (77-178)
 *   log-odds > 0  (wall)    → 230  > OCC_UNK_MAX   (178)
 * ════════════════════════════════════════════════════════════════════════════ */
typedef QuadTreeMap quadtree_map_t;

#include "../../types.h"   /* semantic_class_t */

void     quadtree_map_init  (quadtree_map_t *map,
                              float width_mm, float height_mm, float step_mm);
void     quadtree_map_insert(quadtree_map_t *map,
                              float x, float y, semantic_class_t cls);
uint8_t  quadtree_map_query (const quadtree_map_t *map, float x, float y);

#endif