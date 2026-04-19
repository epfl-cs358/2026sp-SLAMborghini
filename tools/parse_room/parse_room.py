#!/usr/bin/env python3
"""
parse_room.py — SLAMborghini hardware-test room pre-loader
=============================================================
Reads mexico.gfs.log (Carmen GFS / FLASER format), rasterises wall endpoints
onto a 50 mm grid, and emits:

  1.  ../../esp32s3/src/room_data.c   — const C array of CLASS_WALL room_point_t
                                        structs + start pose (walls only; free space
                                        is seeded at runtime by build_test_room())
  2.  ../../esp32s3/src/room_data.h   — extern declarations
  3.  room_preview.png                — static visual proof (requires matplotlib)
  4.  ../dashboard/room_map_data.js   — JS data for live_dashboard.html
                                        (walls at 50 mm + free cells at 200 mm
                                        for the browser — not sent to firmware)

Memory budget for the C array:
  ~30 000 wall cells × 12 bytes = ~360 KB in DROM (flash read-only).
  ESP32-S3 with 4 MB+ flash handles this fine as a const array.

  Free cells are NOT in the C array.  build_test_room() calls a small helper
  that stamps a free disk (radius = FREE_DISK_MM) around the start pose at
  runtime so BFS has a traversable seed region.

FLASER format (Carmen GFS, SICK LMS 180°/1°/181 rays):
  FLASER  num  r[0]..r[N-1]  lx ly ltheta  ox oy otheta  ts host ts2
Beam i:  global_angle = ltheta + (-π/2 + i·π/180)
Ranges in metres.

Usage:
    cd tools/parse_room
    python3 parse_room.py
    python3 parse_room.py --no-png   # skip matplotlib
"""

import math, sys, os
from pathlib import Path

# ── Tunables ────────────────────────────────────────────────────────────────
CELL_MM        = 50        # wall grid resolution; matches quadtree resolution_mm
JS_CELL_MM     = 200       # display resolution for the JS dashboard map
MAX_RANGE_M    = 30.0      # ignore beams ≥ this range (corridors, open doors)
MIN_RANGE_M    = 0.10      # ignore beams ≤ this (sensor noise / self-hits)
MARGIN_MM      = 2000      # padding added around the bounding box on each side
SCAN_STRIDE    = 1         # use every Nth scan (1 = all)
FREE_DISK_MM   = 4000      # radius of the free disk seeded at start by firmware
JS_FREE_STRIDE = 4         # sample free cells every Nth beam cell for JS display
# ────────────────────────────────────────────────────────────────────────────

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
LOG_PATH     = PROJECT_ROOT / "mexico.gfs.log"
OUT_C        = PROJECT_ROOT / "esp32s3" / "src" / "room_data.c"
OUT_H        = PROJECT_ROOT / "esp32s3" / "src" / "room_data.h"
OUT_PNG      = Path(__file__).parent / "room_preview.png"
OUT_JS       = PROJECT_ROOT / "tools" / "dashboard" / "room_map_data.js"


# ════════════════════════════════════════════════════════════════════════════
# 1.  Parse FLASER records
# ════════════════════════════════════════════════════════════════════════════

def parse_flaser(log_path):
    records = []
    with open(log_path) as f:
        for line in f:
            if not line.startswith("FLASER"):
                continue
            parts = line.split()
            n     = int(parts[1])
            ranges = [float(parts[2 + i]) for i in range(n)]
            base   = 2 + n
            x, y, theta = float(parts[base]), float(parts[base+1]), float(parts[base+2])
            records.append((x, y, theta, ranges))
    return records


# ════════════════════════════════════════════════════════════════════════════
# 2.  Rasterise wall endpoints onto a 50 mm grid
# ════════════════════════════════════════════════════════════════════════════

def rasterise(records, stride, max_range_m, min_range_m, cell_mm, margin_mm):
    """
    Returns
    -------
    wall50   : set of (gx, gy) at CELL_MM resolution
    offset   : (ox_mm, oy_mm) world → grid offset
    grid_w_mm, grid_h_mm
    start_pose : (x_mm, y_mm, theta)
    """
    xs = [r[0] for r in records]
    ys = [r[1] for r in records]
    min_x, max_x = min(xs) - max_range_m, max(xs) + max_range_m
    min_y, max_y = min(ys) - max_range_m, max(ys) + max_range_m

    ox = -min_x * 1000.0 + margin_mm
    oy = -min_y * 1000.0 + margin_mm
    gw = (max_x - min_x) * 1000.0 + 2 * margin_mm
    gh = (max_y - min_y) * 1000.0 + 2 * margin_mm

    def to50(xm, ym):
        return (int((xm * 1000.0 + ox) / cell_mm),
                int((ym * 1000.0 + oy) / cell_mm))

    wall50 = {}

    for idx, (rx, ry, theta, ranges) in enumerate(records):
        if idx % stride != 0:
            continue
        for i, r in enumerate(ranges):
            if r < min_range_m or r >= max_range_m:
                continue
            angle = theta + (-math.pi / 2.0 + i * math.pi / 180.0)
            wx = rx + r * math.cos(angle)
            wy = ry + r * math.sin(angle)
            wall50[to50(wx, wy)] = True

    start = records[0]
    sp = (start[0] * 1000.0 + ox,
          start[1] * 1000.0 + oy,
          start[2])

    return set(wall50.keys()), (ox, oy), gw, gh, sp


# ════════════════════════════════════════════════════════════════════════════
# 2b.  Build a separate 200 mm free-cell set for the JS dashboard only
# ════════════════════════════════════════════════════════════════════════════

def build_js_free_cells(records, stride, max_range_m, min_range_m,
                        free_stride, ox, oy, js_cell):
    """
    Returns set of (gx, gy) at JS_CELL_MM resolution that are free along beams.
    Used only by the JS dashboard — NOT written into the C firmware array.
    """
    free = {}
    def to_js(xm, ym):
        return (int((xm * 1000.0 + ox) / js_cell),
                int((ym * 1000.0 + oy) / js_cell))

    for idx, (rx, ry, theta, ranges) in enumerate(records):
        if idx % stride != 0:
            continue
        for i, r in enumerate(ranges):
            if r < min_range_m:
                continue
            valid = min(r, max_range_m)
            angle = theta + (-math.pi / 2.0 + i * math.pi / 180.0)
            cos_a, sin_a = math.cos(angle), math.sin(angle)
            steps = int(valid * 1000.0 / js_cell)
            for s in range(1, steps, free_stride):
                d = s * js_cell / 1000.0
                free[to_js(rx + d * cos_a, ry + d * sin_a)] = True

    return set(free.keys())


# ════════════════════════════════════════════════════════════════════════════
# 3.  Write room_data.h
# ════════════════════════════════════════════════════════════════════════════

HEADER_H = """\
/**
 * room_data.h
 * Auto-generated by tools/parse_room/parse_room.py — do not edit by hand.
 * Declares the pre-loaded room wall data used by build_test_room().
 * Source dataset: mexico.gfs.log (Carmen GFS / SICK LMS)
 */

#ifndef ROOM_DATA_H
#define ROOM_DATA_H

#include <stdint.h>
#include "../../types.h"

/**
 * One pre-classified map point inserted by build_test_room().
 * All points in ROOM_DATA[] carry CLASS_WALL.
 * Free space around the start pose is seeded at runtime inside build_test_room()
 * (see test_room.c: build_free_disk()), so this array contains walls only.
 */
typedef struct {
    float            x_mm; /**< X coordinate in mm (offset world frame) */
    float            y_mm; /**< Y coordinate in mm (offset world frame) */
    semantic_class_t cls;  /**< Always CLASS_WALL here */
} room_point_t;

/** Wall points extracted from the GFS dataset. Stored in DROM (flash). */
extern const room_point_t ROOM_DATA[];

/** Number of entries in ROOM_DATA. */
extern const uint32_t ROOM_DATA_COUNT;

/** Map bounding-box dimensions — pass directly to quadtree_map_init(). */
extern const float ROOM_WIDTH_MM;
extern const float ROOM_HEIGHT_MM;

/**
 * First robot pose from the GFS dataset.
 * build_test_room() copies this into the caller's pose_t so the robot
 * starts at a location that has been mapped.
 */
extern const pose_t ROOM_START_POSE;

#endif /* ROOM_DATA_H */
"""


def write_header(path):
    with open(path, "w") as f:
        f.write(HEADER_H)
    print(f"  Wrote {path}")


# ════════════════════════════════════════════════════════════════════════════
# 4.  Write room_data.c
# ════════════════════════════════════════════════════════════════════════════

def write_c(path, wall_cells, gw, gh, start_pose, cell_mm, free_disk_mm):
    sx, sy, stheta = start_pose

    rows = sorted(wall_cells)
    lines = []
    for gx, gy in rows:
        x_mm = (gx + 0.5) * cell_mm
        y_mm = (gy + 0.5) * cell_mm
        lines.append(f"    {{ {x_mm:.1f}f, {y_mm:.1f}f, CLASS_WALL }},")

    total = len(rows)
    body  = "\n".join(lines)

    text = f"""\
/**
 * room_data.c
 * Auto-generated by tools/parse_room/parse_room.py — do not edit by hand.
 * Source dataset: mexico.gfs.log (Carmen GFS, SICK LMS 181-ray FLASER)
 *
 * Grid resolution   : {cell_mm} mm/cell
 * Map dimensions    : {gw:.0f} mm x {gh:.0f} mm
 * Wall cells        : {total}
 * Free cells        : seeded at runtime as a {free_disk_mm} mm-radius disk
 *                     around ROOM_START_POSE inside build_test_room().
 *
 * Stored in flash (DROM) — approximately {total * 12 // 1024} KB.
 *
 * ── quadtree_map_insert() semantic contract ─────────────────────────────
 * CLASS_WALL  →  leaf occupancy = 255  (OCC_OCCUPIED)
 * CLASS_FREE  →  leaf occupancy =  20  (OCC_FREE)    ← used by free disk
 *
 * ⚠  CRITICAL BUG in quadtree_map.c (stub):
 *    quadtree_map_query() currently returns 0 instead of 128 for
 *    unvisited cells.  Must be fixed to:
 *        return 128;   // OCC_UNKNOWN
 *    before frontier_detector_detect() will produce valid results.
 * ─────────────────────────────────────────────────────────────────────────
 */

#include "room_data.h"

const room_point_t ROOM_DATA[] = {{
{body}
}};

const uint32_t ROOM_DATA_COUNT = {total}u;

const float ROOM_WIDTH_MM  = {gw:.1f}f;
const float ROOM_HEIGHT_MM = {gh:.1f}f;

const pose_t ROOM_START_POSE = {{
    .x     = {sx:.1f}f,
    .y     = {sy:.1f}f,
    .theta = {stheta:.6f}f,
    .cov   = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
}};
"""
    with open(path, "w") as f:
        f.write(text)
    print(f"  Wrote {path}  ({total} wall cells, ~{total * 12 // 1024} KB in flash)")


# ════════════════════════════════════════════════════════════════════════════
# 5.  Write room_map_data.js (dashboard only — walls at 50 mm + free at 200 mm)
# ════════════════════════════════════════════════════════════════════════════

def write_js(path, wall50, js_free, gw, gh, start_pose, cell_mm, js_cell):
    sx, sy, stheta = start_pose

    # Build JS-resolution wall cells from the 50 mm set
    js_wall = {}
    scale = js_cell // cell_mm       # 200/50 = 4
    for (gx, gy) in wall50:
        js_wall[(gx // scale, gy // scale)] = True

    # Remove js_free cells that overlap walls
    for k in js_wall:
        js_free.discard(k)

    grid_cols = int(math.ceil(gw / js_cell))
    grid_rows = int(math.ceil(gh / js_cell))

    wall_arr = ",".join(f"[{gx},{gy}]" for gx, gy in sorted(js_wall))
    free_arr = ",".join(f"[{gx},{gy}]" for gx, gy in sorted(js_free))

    text = f"""\
/* Auto-generated by tools/parse_room/parse_room.py — do not edit.
 * Source: mexico.gfs.log
 * Display grid: {grid_cols} x {grid_rows} cells at {js_cell} mm/cell
 * Wall cells  : {len(js_wall)}
 * Free cells  : {len(js_free)}
 *
 * This file is loaded by live_dashboard.html to render the background map.
 * It is NOT used by the firmware (firmware uses room_data.c).
 */
var ROOM_MAP_META = {{
  cellMm     : {js_cell},
  gridCols   : {grid_cols},
  gridRows   : {grid_rows},
  widthMm    : {gw:.1f},
  heightMm   : {gh:.1f},
  startX     : {sx:.1f},
  startY     : {sy:.1f},
  startTheta : {stheta:.6f}
}};
var ROOM_WALL_CELLS = [{wall_arr}];
var ROOM_FREE_CELLS = [{free_arr}];
"""
    with open(path, "w") as f:
        f.write(text)
    print(f"  Wrote {path}  ({len(js_wall)} wall + {len(js_free)} free display cells)")


# ════════════════════════════════════════════════════════════════════════════
# 6.  PNG preview
# ════════════════════════════════════════════════════════════════════════════

def write_png(path, wall50, js_free, gw, gh, start_pose, cell_mm, js_cell):
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("  [skip] matplotlib not available")
        return

    cols = int(math.ceil(gw / js_cell))
    rows = int(math.ceil(gh / js_cell))

    scale = js_cell // cell_mm
    img = {}

    for (gx, gy) in js_free:
        img[(gx, gy)] = (45, 100, 75)
    for (gx, gy) in wall50:
        img[(gx // scale, gy // scale)] = (192, 200, 208)

    arr = [[30, 30, 50]] * (rows * cols)
    for (gx, gy), color in img.items():
        if 0 <= gx < cols and 0 <= gy < rows:
            arr[gy * cols + gx] = list(color)

    import numpy as np
    rgb = np.array(arr, dtype=np.uint8).reshape(rows, cols, 3)

    sx, sy, _ = start_pose
    rx = int(sx / js_cell)
    ry = int(sy / js_cell)

    fig_w = max(8, cols / 150)
    fig_h = max(5, rows / 150)
    fig, ax = plt.subplots(figsize=(min(fig_w, 20), min(fig_h, 12)), dpi=150)
    ax.imshow(rgb, origin="lower", interpolation="nearest")
    ax.plot(rx, ry, "o", color="#4361ee", markersize=10, label="Start pose")
    ax.set_title(f"Mexico GFS — parsed room  ({cols}×{rows} cells @ {js_cell} mm)", fontsize=9)
    patches = [
        mpatches.Patch(color="#1e1e32", label="Unknown"),
        mpatches.Patch(color="#2d6448", label="Free"),
        mpatches.Patch(color="#c0c8d0", label="Wall"),
        mpatches.Patch(color="#4361ee", label="Start pose"),
    ]
    ax.legend(handles=patches, loc="lower right", fontsize=7)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"  Wrote {path}")


# ════════════════════════════════════════════════════════════════════════════
# main
# ════════════════════════════════════════════════════════════════════════════

def main():
    no_png = "--no-png" in sys.argv

    print("parse_room.py — SLAMborghini room pre-loader")
    print(f"  Input: {LOG_PATH}")

    print("  Parsing FLASER records …")
    records = parse_flaser(LOG_PATH)
    print(f"  Found {len(records)} records")

    print("  Rasterising wall cells at 50 mm …")
    wall50, (ox, oy), gw, gh, start_pose = rasterise(
        records, SCAN_STRIDE, MAX_RANGE_M, MIN_RANGE_M, CELL_MM, MARGIN_MM)
    print(f"  Grid: {gw:.0f} mm x {gh:.0f} mm  |  {len(wall50)} wall cells")

    print("  Building JS free-cell layer at 200 mm …")
    js_free = build_js_free_cells(
        records, SCAN_STRIDE, MAX_RANGE_M, MIN_RANGE_M, JS_FREE_STRIDE, ox, oy, JS_CELL_MM)
    # Remove js_free cells that overlap with walls (at JS resolution)
    scale = JS_CELL_MM // CELL_MM
    js_wall_keys = {(gx // scale, gy // scale) for (gx, gy) in wall50}
    js_free -= js_wall_keys
    print(f"  JS free cells: {len(js_free)}")

    sx, sy, st = start_pose
    print(f"  Start pose: x={sx:.0f} mm, y={sy:.0f} mm, theta={st:.4f} rad")

    write_header(OUT_H)
    write_c(OUT_C, wall50, gw, gh, start_pose, CELL_MM, FREE_DISK_MM)
    write_js(OUT_JS, wall50, js_free, gw, gh, start_pose, CELL_MM, JS_CELL_MM)

    if not no_png:
        write_png(OUT_PNG, wall50, js_free, gw, gh, start_pose, CELL_MM, JS_CELL_MM)

    print()
    print("Done.")
    print(f"  room_data.c   → esp32s3/src/room_data.c")
    print(f"  room_data.h   → esp32s3/src/room_data.h")
    print(f"  room_map_data.js → tools/dashboard/room_map_data.js")
    print()
    print("  ⚠  CRITICAL: quadtree_map_query() in esp32s3/src/quadtree_map.c")
    print("     returns 0 (free) for unknown cells — must be fixed to return 128")
    print("     (unknown) before frontier_detector_detect() will work:")
    print("        - return 0;   →   + return 128;")

if __name__ == "__main__":
    main()
