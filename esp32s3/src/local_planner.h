/**
 * local_planner.h
 * Module: Local planner — 100 ms reactive navigation on top of Hybrid A* global path.
 * Board: ESP32-S3
 *
 * Navigation stack:
 *   Hybrid A*  → global path (unchanged, external)
 *   Pure Pursuit (STUB) → default path tracking
 *   Reactive   → local obstacle avoidance
 *   ESCAPE     → reverse + rotate recovery
 *   RECOVER    → reduced-speed mode under high SLAM uncertainty
 */

#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <stdbool.h>
#include "../../types.h"
#include "quadtree_map.h"
#include "hybrid_astar.h"

/** Operating mode of the local planner state machine. */
typedef enum {
    LP_MODE_PURE_PURSUIT = 0, /**< Default: tracking global Hybrid A* path (stub) */
    LP_MODE_REACTIVE,         /**< Local obstacle cluster detected — candidate steering */
    LP_MODE_ESCAPE,           /**< All reactive candidates blocked — reverse+rotate sequence */
    LP_MODE_RECOVER,          /**< High SLAM uncertainty — reduced speed, wider heading filter */
    LP_MODE_STOPPED           /**< Footprint cell occupied — immediate stop */
} lp_mode_t;

/**
 * Initialise the local planner static state.
 * Must be called once before local_planner_update().
 * @param robot_radius_mm Physical robot radius in mm (used for footprint inflation).
 */
void local_planner_init(float robot_radius_mm);

/**
 * Run one 100 ms local planner cycle.
 *
 * @param map           Current quadtree map (read-only).
 * @param raw_pose      Raw SLAM pose for this cycle.
 * @param global_path   Hybrid A* path to follow; may be NULL or empty.
 * @param override_flag If true, an emergency layer has control — skip this cycle.
 * @param out_cmd       Output control frame; only valid when function returns true.
 * @return true  — out_cmd is valid, caller should transmit it.
 *         false — cycle skipped (override active); do NOT transmit.
 */
bool local_planner_update(const quadtree_map_t *map,
                          const pose_t         *raw_pose,
                          const path_t         *global_path,
                          bool                  override_flag,
                          control_frame_t      *out_cmd);

/** Return the current operating mode. */
lp_mode_t local_planner_get_mode(void);

/**
 * Return true if the local planner requests a global replan.
 * Caller should invoke hybrid_astar_plan() with the current pose and target,
 * then pass the fresh path back to local_planner_update().
 */
bool local_planner_replan_needed(void);

/** Clear the replan request flag after replanning. */
void local_planner_clear_replan(void);

/**
 * Reset the waypoint cursor to 0.
 * Must be called whenever a new global path is supplied.
 */
void local_planner_reset_waypoint(void);

#endif /* LOCAL_PLANNER_H */
