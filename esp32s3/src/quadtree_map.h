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
#include "../../types.h"

/** Forward declaration for internal quadtree node (opaque to callers). */
typedef struct quadtree_node_t quadtree_node_t;

/** Top-level quadtree occupancy map descriptor. */
typedef struct {
    void  *root;          /**< Pointer to root quadtree_node_t (opaque) */
    float  resolution_mm; /**< Leaf cell size in millimetres */
    float  width_mm;      /**< Total map width in millimetres */
    float  height_mm;     /**< Total map height in millimetres */
} quadtree_map_t;

/**
 * Initialize a quadtree map covering [0, width_mm] x [0, height_mm].
 * @param map           Pointer to the quadtree_map_t to initialize.
 * @param width_mm      Map width in millimetres.
 * @param height_mm     Map height in millimetres.
 * @param resolution_mm Minimum leaf cell size in millimetres.
 */
void quadtree_map_init(quadtree_map_t *map, float width_mm, float height_mm, float resolution_mm);

/**
 * Insert a classified observation at the given map coordinates.
 * @param map Pointer to the map.
 * @param x   X coordinate of the observation in mm.
 * @param y   Y coordinate of the observation in mm.
 * @param cls Semantic class of the observation.
 */
void quadtree_map_insert(quadtree_map_t *map, float x, float y, semantic_class_t cls);

/**
 * Query occupancy at a given map coordinate.
 * @param map Pointer to the map (const, no modification).
 * @param x   X coordinate to query in mm.
 * @param y   Y coordinate to query in mm.
 * @return Occupancy value 0 (free) to 255 (fully occupied).
 */
uint8_t quadtree_map_query(const quadtree_map_t *map, float x, float y);

/**
 * Free all dynamically allocated nodes in the quadtree map.
 * @param map Pointer to the map to free.
 */
void quadtree_map_free(quadtree_map_t *map);

#endif /* QUADTREE_MAP_H */
