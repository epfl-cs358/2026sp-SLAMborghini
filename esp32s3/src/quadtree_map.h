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



#define QT_MAX_DEPTH 6

// complete quadtree to depth 6 needs at most 5 461 nodes -> 6 000 gives a comfortable safety marginƒ
#define QT_POOL_SIZE 6000
#define QT_NULL 0 // no child index 

// Log-odds increments 
#define QT_HIT_INC 15  // obstacle confirmed → value rises
#define QT_MISS_DEC (-6)  // ray passed through → value drops 
#define QT_VALUE_MAX 40
#define QT_VALUE_MIN (-40)

// Node 
typedef struct {
    uint16_t children[4]; // pool indices; QT_NULL = absent 
    int8_t value; // log-odds occupancy (meaningful at leaves) 
    uint8_t depth; // depth in tree (root = 1) 
} QTNode;


typedef struct {
    QTNode *pool; // heap-allocated array (from 0 to QT_POOL_SIZE)
    uint16_t count; // next free slot 
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