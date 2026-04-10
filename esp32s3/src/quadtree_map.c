/**
 * quadtree_map.c
 * Module: Quadtree occupancy map.
 * Board: ESP32-S3
 * Implementation phase: stub (tree allocation and traversal not yet implemented)
 */

#include "quadtree_map.h"

void quadtree_map_init(quadtree_map_t *map, float width_mm, float height_mm, float resolution_mm)
{
    // TODO: implement
    // Allocate root node, set map->width_mm, map->height_mm, map->resolution_mm.
    // Initialize all cells to unknown (occupancy = 128).
    (void)map;
    (void)width_mm;
    (void)height_mm;
    (void)resolution_mm;
}

void quadtree_map_insert(quadtree_map_t *map, float x, float y, semantic_class_t cls)
{
    // TODO: implement
    // Traverse/allocate quadtree nodes to reach the leaf at (x, y).
    // Update occupancy and semantic class at that leaf.
    (void)map;
    (void)x;
    (void)y;
    (void)cls;
}

uint8_t quadtree_map_query(const quadtree_map_t *map, float x, float y)
{
    // TODO: implement
    // Traverse the quadtree to find the leaf at (x, y).
    // Return its occupancy value, or 128 (unknown) if not yet observed.
    (void)map;
    (void)x;
    (void)y;
    return 0;
}

void quadtree_map_free(quadtree_map_t *map)
{
    // TODO: implement
    // Recursively free all quadtree_node_t allocations starting from map->root.
    // Set map->root = NULL.
    (void)map;
}
