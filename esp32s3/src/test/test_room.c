/**
 * test_room.c
 * Module: Pre-loaded test room for hardware validation.
 * Board: ESP32-S3
 *
 * Implements build_test_room().  Only calls the existing public quadtree API:
 *   quadtree_map_init(), quadtree_map_insert().
 * Does NOT touch frontier_detector.c, command_gen.c, or uart_bridge.c.
 */

#include "test_room.h"
#include "room_data.h"
#include <math.h>

/* ════════════════════════════════════════════════════════════════════════════
 * build_free_disk
 * Stamps a circular region of CLASS_FREE cells (radius = r_mm) centred on
 * (cx_mm, cy_mm) into the map.  This gives frontier_detector_detect() a
 * traversable seed region so BFS can start from the robot's pose.
 *
 * Cell step = 50 mm (matching quadtree resolution_mm).  Only cells whose
 * centre falls within the circle are inserted.  ~11 000 insertions for
 * r = 4 000 mm.
 * ════════════════════════════════════════════════════════════════════════════ */
#define FREE_STEP_MM  50.0f   /* must match quadtree resolution_mm */

static void build_free_disk(quadtree_map_t *map,
                             float cx_mm, float cy_mm, float r_mm)
{
    float r2   = r_mm * r_mm;
    float step = FREE_STEP_MM;

    for (float dy = -r_mm; dy <= r_mm; dy += step) {
        for (float dx = -r_mm; dx <= r_mm; dx += step) {
            if (dx * dx + dy * dy <= r2) {
                quadtree_map_insert(map, cx_mm + dx, cy_mm + dy, CLASS_FREE);
            }
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * build_test_room
 * ════════════════════════════════════════════════════════════════════════════ */
void build_test_room(quadtree_map_t *map, pose_t *start_pose)
{
    /* 1. Initialise the map with the dataset's bounding box. */
    quadtree_map_init(map, ROOM_WIDTH_MM, ROOM_HEIGHT_MM, FREE_STEP_MM);

    /* 2. Insert all wall cells from the pre-parsed dataset. */
    for (uint32_t i = 0; i < ROOM_DATA_COUNT; i++) {
        quadtree_map_insert(map,
                            ROOM_DATA[i].x_mm,
                            ROOM_DATA[i].y_mm,
                            ROOM_DATA[i].cls);
    }

    /* 3. Set the robot's starting pose to the first GFS pose. */
    *start_pose = ROOM_START_POSE;

    /* 4. Stamp a free disk around the start pose so BFS can traverse.
     *
     *    IMPORTANT: quadtree_map_insert(CLASS_FREE) must set leaf occupancy
     *    to ~20 (≤ OCC_FREE_MAX = 50).  The wall cells inserted above already
     *    mark the room boundaries, so the disk's free cells stop at the walls.
     *
     *    If quadtree_map_query() still returns 0 for unvisited cells, ALL
     *    cells will appear free to frontier_detector — fix that first. */
    build_free_disk(map,
                    start_pose->x,
                    start_pose->y,
                    BUILD_FREE_DISK_MM);
}
