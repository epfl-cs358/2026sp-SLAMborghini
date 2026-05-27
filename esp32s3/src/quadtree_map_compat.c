/**
 * quadtree_map_compat.c
 * Implements the legacy quadtree_map_* API declared in quadtree_map.h using
 * the underlying qt_* functions.  No files from teammates are modified.
 */

#include "quadtree_map.h"
#include <stdint.h>

/* ── Init: map covers [0, width_mm] × [0, height_mm]; step_mm unused by qt ── */
void quadtree_map_init(quadtree_map_t *map,
                       float width_mm, float height_mm, float step_mm)
{
    (void)step_mm;
    qt_init(map, 0.0f, width_mm, 0.0f, height_mm);
}

/* ── Insert: one log-odds update per call ──────────────────────────────────── */
void quadtree_map_insert(quadtree_map_t *map,
                         float x, float y, semantic_class_t cls)
{
    if (cls == CLASS_WALL) {
        qt_update(map, x, y, QT_HIT_INC);
    } else {
        /* CLASS_FREE, CLASS_UNKNOWN, etc. — mark traversed */
        qt_update(map, x, y, QT_MISS_DEC);
    }
}

/* ── Query: int8 log-odds → uint8 occupancy ────────────────────────────────── */
uint8_t quadtree_map_query(const quadtree_map_t *map, float x, float y)
{
    int8_t v = qt_query_const(map, x, y);
    if (v <= QT_FREE_CONFIRMED) return 20u;    /* confirmed free */
    if (v >= QT_OCC_CONFIRMED)  return 230u;   /* confirmed wall */
    return 128u;               /* unknown */
}
