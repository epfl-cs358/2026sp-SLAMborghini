#!/usr/bin/env python3
"""
convert_quadtree.py — SLAMborghini Intel map converter
=======================================================
Reads intel_mock_map_quadtree.h and emits:

  1. ../../esp32s3/src/room_data.c   — CLASS_WALL points for build_test_room()
  2. ../../esp32s3/src/room_data.h   — extern declarations
  3. ../dashboard/room_map_data.js   — JS metadata + PNG pointer
  4. ../dashboard/room_map.png       — high-res occupancy grid (1 cm/pixel)

The quadtree is traversed recursively.  Each LEAF node covers a bounding box:
  value = 1  → WALL  (black  in PNG)
  value = 0  → FREE  (white  in PNG)
Internal nodes are sub-divided; cells not covered by any leaf stay UNKNOWN (grey).

Child ordering (standard NW/NE/SW/SE):
  child[0] = NW:  [x,   x+w/2) × [y+h/2, y+h)
  child[1] = NE:  [x+w/2, x+w) × [y+h/2, y+h)
  child[2] = SW:  [x,   x+w/2) × [y,   y+h/2)
  child[3] = SE:  [x+w/2, x+w) × [y,   y+h/2)

Usage:
    cd tools/parse_room
    python3 convert_quadtree.py
"""

import re, sys
from pathlib import Path

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ── Tunables ────────────────────────────────────────────────────────────────
ROOT_W_MM   = 10000.0    # physical width  of the room crop (10 m)
ROOT_H_MM   = 10000.0    # physical height of the room crop (10 m)

START_X_MM  = ROOT_W_MM / 2.0   # robot starts at room centre
START_Y_MM  = ROOT_H_MM / 2.0
START_THETA = 0.0

FREE_DISK_MM = 2000.0    # free-space disk seeded at runtime by firmware

PX_PER_MM   = 0.1        # PNG resolution: 1 px = 10 mm = 1 cm
                          # → 10 m × 10 m = 1000 × 1000 px
# ────────────────────────────────────────────────────────────────────────────

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
QT_H   = PROJECT_ROOT / "intel_mock_map_quadtree.h"
OUT_C  = PROJECT_ROOT / "esp32s3" / "src" / "room_data.c"
OUT_H  = PROJECT_ROOT / "esp32s3" / "src" / "room_data.h"
OUT_JS = PROJECT_ROOT / "tools" / "dashboard" / "room_map_data.js"
OUT_PNG = PROJECT_ROOT / "tools" / "dashboard" / "room_map.png"

CELL_MM = 1.0 / PX_PER_MM   # mm per pixel (10 mm)

# Grid cell states
UNKNOWN = 0
FREE    = 1
WALL    = 2

# PNG colours
COL = {
    UNKNOWN: (255, 140,   0),   # orange — unexplored
    FREE:    (255, 255, 255),   # white  — free space
    WALL:    (  0,   0,   0),   # black  — obstacle
}

# ════════════════════════════════════════════════════════════════════════════
# 1. Parse intel_mock_map_quadtree.h
# ════════════════════════════════════════════════════════════════════════════
text = QT_H.read_text()

pattern = (r'\{\s*(\d+)\s*,\s*(-?\d+)\s*,'
           r'\s*\{(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*\}')
nodes = []
for m in re.finditer(pattern, text):
    nodes.append((
        int(m.group(1)),
        int(m.group(2)),
        [int(m.group(i)) for i in range(3, 7)],
    ))

print(f"Parsed {len(nodes)} nodes from {QT_H.name}")

# ════════════════════════════════════════════════════════════════════════════
# 2. Traverse tree — collect wall centres (for room_data.c)
#                  — render full leaf bounding boxes (for PNG)
# ════════════════════════════════════════════════════════════════════════════
wall_mm = []

W = int(ROOT_W_MM * PX_PER_MM)
H = int(ROOT_H_MM * PX_PER_MM)

if HAS_NUMPY:
    grid = np.full((H, W), UNKNOWN, dtype=np.uint8)
else:
    grid = [[UNKNOWN] * W for _ in range(H)]

def fill_rect(x0_mm, y0_mm, w_mm, h_mm, state):
    """Fill pixels covering the bounding box [x0,x0+w) × [y0,y0+h) in mm."""
    c0 = max(0, int(x0_mm * PX_PER_MM))
    c1 = min(W,  int((x0_mm + w_mm) * PX_PER_MM))
    # Y flip: world y=0 is bottom, image row=0 is top
    r0 = max(0, H - int((y0_mm + h_mm) * PX_PER_MM))
    r1 = min(H, H - int(y0_mm * PX_PER_MM))
    if c1 <= c0 or r1 <= r0:
        return
    if HAS_NUMPY:
        grid[r0:r1, c0:c1] = state
    else:
        for r in range(r0, r1):
            for c in range(c0, c1):
                grid[r][c] = state

def traverse(idx, x0, y0, w, h):
    if idx < 0 or idx >= len(nodes):
        return
    is_leaf, value, children = nodes[idx]
    if is_leaf:
        if value == 1:
            wall_mm.append((x0 + w * 0.5, y0 + h * 0.5))
            fill_rect(x0, y0, w, h, WALL)
        elif value == 0:
            fill_rect(x0, y0, w, h, FREE)
        # value == -1 (mixed/unknown leaf): leave as UNKNOWN
    else:
        hw, hh = w * 0.5, h * 0.5
        traverse(children[0], x0,    y0 + hh, hw, hh)  # NW
        traverse(children[1], x0+hw, y0 + hh, hw, hh)  # NE
        traverse(children[2], x0,    y0,      hw, hh)  # SW
        traverse(children[3], x0+hw, y0,      hw, hh)  # SE

traverse(0, 0.0, 0.0, ROOT_W_MM, ROOT_H_MM)
print(f"Extracted {len(wall_mm)} wall points")

# ════════════════════════════════════════════════════════════════════════════
# 3. Save PNG
# ════════════════════════════════════════════════════════════════════════════
if HAS_NUMPY and HAS_PIL:
    rgb = np.empty((H, W, 3), dtype=np.uint8)
    for state, colour in COL.items():
        rgb[grid == state] = colour
    img = Image.fromarray(rgb, "RGB")
    img.save(OUT_PNG)
    print(f"Wrote {OUT_PNG}  ({W}×{H} px, {CELL_MM:.0f} mm/px)")
elif not HAS_PIL:
    print("WARNING: Pillow not installed — PNG skipped.  pip install Pillow")
else:
    print("WARNING: numpy not installed — PNG skipped.  pip install numpy")

# ════════════════════════════════════════════════════════════════════════════
# 4. Write room_data.c
# ════════════════════════════════════════════════════════════════════════════
with open(OUT_C, 'w') as f:
    f.write(f"""\
/**
 * room_data.c — Auto-generated by tools/parse_room/convert_quadtree.py
 * Source: intel_mock_map_quadtree.h
 */

#include "room_data.h"

const room_point_t ROOM_DATA[] = {{
""")
    for x, y in wall_mm:
        f.write(f"    {{ {x:.2f}f, {y:.2f}f, CLASS_WALL }},\n")
    f.write(f"""\
}};

const uint32_t ROOM_DATA_COUNT = {len(wall_mm)}u;

const float ROOM_WIDTH_MM  = {ROOT_W_MM:.1f}f;
const float ROOM_HEIGHT_MM = {ROOT_H_MM:.1f}f;

const pose_t ROOM_START_POSE = {{ {START_X_MM:.1f}f, {START_Y_MM:.1f}f, {START_THETA:.4f}f, {{0}} }};
""")
print(f"Wrote {OUT_C}")

# ════════════════════════════════════════════════════════════════════════════
# 5. Write room_data.h
# ════════════════════════════════════════════════════════════════════════════
with open(OUT_H, 'w') as f:
    f.write("""\
/**
 * room_data.h — Auto-generated by tools/parse_room/convert_quadtree.py
 */
#ifndef ROOM_DATA_H
#define ROOM_DATA_H

#include <stdint.h>
#include "../../types.h"

typedef struct {
    float            x_mm;
    float            y_mm;
    semantic_class_t cls;
} room_point_t;

extern const room_point_t ROOM_DATA[];
extern const uint32_t     ROOM_DATA_COUNT;
extern const float        ROOM_WIDTH_MM;
extern const float        ROOM_HEIGHT_MM;
extern const pose_t       ROOM_START_POSE;

#endif /* ROOM_DATA_H */
""")
print(f"Wrote {OUT_H}")

# ════════════════════════════════════════════════════════════════════════════
# 6. Write room_map_data.js
# ════════════════════════════════════════════════════════════════════════════
with open(OUT_JS, 'w') as f:
    f.write(f"""\
// Auto-generated by tools/parse_room/convert_quadtree.py — do not edit.
// Source: intel_mock_map_quadtree.h

const ROOM_MAP_META = {{
  imageFile:  'room_map.png',
  cellMm:     {CELL_MM},        // mm per pixel
  gridCols:   {W},
  gridRows:   {H},
  startX:     {START_X_MM},    // robot start in mm (matches ESP32 ROOM_START_POSE)
  startY:     {START_Y_MM},
  startTheta: {START_THETA},
  // No coordinate offset needed — ESP32 and PNG share the same origin.
  robotOffsetX: 0,
  robotOffsetY: 0,
}};

// No ROOM_WALL_CELLS / ROOM_FREE_CELLS — map is loaded from room_map.png.
const ROOM_WALL_CELLS = [];
const ROOM_FREE_CELLS = [];
""")
print(f"Wrote {OUT_JS}")

print(f"\nSummary:")
print(f"  Wall points : {len(wall_mm)}")
print(f"  PNG         : {W}×{H} px  @ {CELL_MM:.0f} mm/px")
print(f"  Room        : {ROOT_W_MM/1000:.0f} m × {ROOT_H_MM/1000:.0f} m")
print(f"  Start pose  : ({START_X_MM:.0f}, {START_Y_MM:.0f}) mm  θ={START_THETA}")
