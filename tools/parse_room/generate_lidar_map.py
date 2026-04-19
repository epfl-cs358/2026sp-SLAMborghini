#!/usr/bin/env python3
"""
generate_lidar_map.py
Parses room.log (CARMEN FLASER format, 180 beams) and produces a high-resolution
occupancy grid map for the SLAMborghini dashboard.

room.log field layout (191 fields per FLASER line):
  FLASER 180 r0..r179 x y theta odom_x odom_y odom_theta ts host ts
  No max_range field — unlike mexico.gfs.log which has 181 beams + max_range.

Algorithm:
  For every FLASER scan:
    • Ray-trace each beam from robot → hit point (mark cells FREE)
    • Mark hit cell as WALL
  Render at 5 cm/pixel.

Output:
    tools/dashboard/lidar_map.png     — occupancy grid image
    tools/dashboard/room_map_data.js  — metadata JS consumed by live_dashboard.html

Usage (run from repo root):
    python3 tools/parse_room/generate_lidar_map.py
"""

import math, os, sys, json
from pathlib import Path

try:
    import numpy as np
except ImportError:
    print("ERROR: numpy required.  Install with:  pip install numpy")
    sys.exit(1)

try:
    from PIL import Image
except ImportError:
    print("ERROR: Pillow required.  Install with:  pip install Pillow")
    sys.exit(1)

# ── Config ────────────────────────────────────────────────────────────────────
LOG_FILE      = "room.log"
OUT_PNG       = "tools/dashboard/lidar_map.png"
OUT_JS        = "tools/dashboard/room_map_data.js"
OUT_SCAN_JS   = "tools/dashboard/room_scan_data.js"
OUT_WP_C      = "esp32s3/src/room_waypoints.c"
OUT_WP_H      = "esp32s3/src/room_waypoints.h"

# Waypoint generation settings
WP_STRIDE     = 5        # sample every Nth scan (~250 mm spacing at 50mm/scan)
WP_SPEED      = 100.0    # mm/s target speed sent to Wemos
ESP_START_X   = 5000.0   # mm — ESP32 dead-reckon origin X (matches live_dashboard)
ESP_START_Y   = 5000.0   # mm — ESP32 dead-reckon origin Y

M_PER_PX   = 0.05          # 5 cm per pixel
PX_PER_M   = 1.0 / M_PER_PX   # 20 px per metre
MARGIN_M   = 1.5            # blank border around map

# FLASER geometry — 180 beams, -90° to +89°, 1° apart
N_BEAMS       = 180
BEAM_START_RAD = math.radians(-90.0)
BEAM_STEP_RAD  = math.radians(1.0)

MAX_RANGE = 8.1    # readings ≥ this are "no obstacle" (SICK LMS indoor typical max)

# Grid cell states
UNKNOWN = 0
FREE    = 1
WALL    = 2
PATH    = 3

# Render colours  (R, G, B)
COL = {
    UNKNOWN: (255, 140,   0),   # orange — unexplored
    FREE:    (255, 255, 255),   # white  — scanned empty space
    WALL:    (  0,   0,   0),   # black  — obstacle
    PATH:    (  0, 180, 220),   # cyan   — robot trajectory
}


# ── Parse log ─────────────────────────────────────────────────────────────────
def parse_flaser(path):
    """Return list of (x_m, y_m, theta_rad, ranges_m).

    room.log layout: FLASER n r0…rN-1 x y theta odom_x odom_y odom_theta ts host ts
    No max_range field (unlike mexico.gfs.log which has one after the ranges).
    """
    scans = []
    with open(path) as f:
        for line in f:
            if not line.startswith("FLASER"):
                continue
            p   = line.split()
            n   = int(p[1])
            rng = [float(p[2 + i]) for i in range(n)]
            # base = 2 + n  (immediately after the n range values)
            base = 2 + n
            x, y, th = float(p[base]), float(p[base+1]), float(p[base+2])
            scans.append((x, y, th, rng))
    return scans


# ── World ↔ pixel helpers ─────────────────────────────────────────────────────
def to_px(x_m, y_m, ox, oy, H):
    """World metres → image pixel (column, row).  Y is flipped (world-up → image-down)."""
    col = int((x_m - ox) * PX_PER_M)
    row = H - 1 - int((y_m - oy) * PX_PER_M)
    return col, row


# ── Render ────────────────────────────────────────────────────────────────────
def render(scans):
    print(f"  {len(scans)} scans loaded.")

    # ── Compute world bounds over all hit points ──────────────────────────────
    print("  Computing bounds …")
    xs, ys = [], []
    for (rx, ry, th, rng) in scans:
        xs.append(rx); ys.append(ry)
        for i, r in enumerate(rng):
            if r >= MAX_RANGE:
                continue
            a  = th + BEAM_START_RAD + i * BEAM_STEP_RAD
            xs.append(rx + r * math.cos(a))
            ys.append(ry + r * math.sin(a))

    ox = min(xs) - MARGIN_M
    oy = min(ys) - MARGIN_M
    W  = int((max(xs) + MARGIN_M - ox) * PX_PER_M) + 1
    H  = int((max(ys) + MARGIN_M - oy) * PX_PER_M) + 1
    print(f"  Grid: {W} × {H} px  "
          f"({(W*M_PER_PX):.1f} m × {(H*M_PER_PX):.1f} m)")

    grid = np.zeros((H, W), dtype=np.uint8)   # UNKNOWN = 0

    # ── Ray-cast every scan ───────────────────────────────────────────────────
    print("  Ray-casting …")
    for idx, (rx, ry, th, rng) in enumerate(scans):
        if idx % 200 == 0:
            print(f"    scan {idx}/{len(scans)}")

        rc, rr = to_px(rx, ry, ox, oy, H)

        # Robot path dot
        if 0 <= rc < W and 0 <= rr < H:
            grid[rr, rc] = PATH

        for i, r in enumerate(rng):
            a   = th + BEAM_START_RAD + i * BEAM_STEP_RAD
            eff = min(r, MAX_RANGE - 0.05)   # ray length (cap at just under max)
            hx  = rx + eff * math.cos(a)
            hy  = ry + eff * math.sin(a)
            hc, hr = to_px(hx, hy, ox, oy, H)

            # Bresenham line from robot to hit — mark FREE
            dc = hc - rc; dr = hr - rr
            steps = max(abs(dc), abs(dr), 1)
            for s in range(steps):
                t  = s / steps
                pc = int(rc + t * dc)
                pr = int(rr + t * dr)
                if 0 <= pc < W and 0 <= pr < H:
                    if grid[pr, pc] == UNKNOWN:
                        grid[pr, pc] = FREE

            # Mark wall hit (only for real obstacles, not max-range misses)
            if r < MAX_RANGE and 0 <= hc < W and 0 <= hr < H:
                grid[hr, hc] = WALL

    # ── Render to PNG ─────────────────────────────────────────────────────────
    print("  Rendering PNG …")
    rgb = np.empty((H, W, 3), dtype=np.uint8)
    for state, colour in COL.items():
        mask = (grid == state)
        rgb[mask] = colour

    img = Image.fromarray(rgb, "RGB")

    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    img.save(OUT_PNG)
    print(f"  Saved: {OUT_PNG}")

    # ── Write JS metadata ─────────────────────────────────────────────────────
    sx, sy, sth = scans[0][0], scans[0][1], scans[0][2]
    js = f"""// Auto-generated by tools/parse_room/generate_lidar_map.py
// Source: {LOG_FILE}  —  do not edit manually.

const ROOM_MAP_META = {{
  imageFile:  'lidar_map.png',
  pxPerMeter: {PX_PER_M},
  cellMm:     {M_PER_PX * 1000},   // mm per pixel (50 mm = 5 cm)
  originX:    {ox},     // world x at image left edge   (metres)
  originY:    {oy},     // world y at image bottom edge (metres)
  widthPx:    {W},
  heightPx:   {H},
  gridCols:   {W},      // alias used by coordinate helpers
  gridRows:   {H},
  startX:     {sx * 1000},   // world position of robot start (mm)
  startY:     {sy * 1000},
  startTheta: {sth},
  // ESP32 dead-reckoning starts at (espStartX, espStartY) mm = (startX, startY) world mm.
  espStartX:  5000.0,
  espStartY:  5000.0,
}};

// No ROOM_WALL_CELLS / ROOM_FREE_CELLS — map loaded from lidar_map.png.
const ROOM_WALL_CELLS = [];
const ROOM_FREE_CELLS = [];
"""
    with open(OUT_JS, "w") as f:
        f.write(js)
    print(f"  Saved: {OUT_JS}")

    # ── Export scan data for animated replay ─────────────────────────────────
    print("  Exporting scan data for lidar_replay.html …")
    export_scan_data(scans, ox, oy, W, H, OUT_SCAN_JS)

    print("  Exporting C waypoints for ESP32 playback …")
    export_waypoints(scans, OUT_WP_C, OUT_WP_H)

    print(f"\nDone.  Map: {W}×{H} px, {len(scans)} scans, "
          f"start pose ({sx:.2f}, {sy:.2f}) m")


# ── Export waypoints as C for ESP32 playback ─────────────────────────────────
def export_waypoints(scans, out_c, out_h,
                     stride=WP_STRIDE,
                     esp_start_x=ESP_START_X, esp_start_y=ESP_START_Y,
                     speed_mm_s=WP_SPEED):
    """
    Derive room_waypoints.c / .h from room.log scan poses.

    Coordinate mapping (must mirror live_dashboard.html mmToCell):
      ESP32 starts at (esp_start_x, esp_start_y) mm = scan[0] world position.
      waypoint[i].x = esp_start_x + (scan[i*stride].x_m - scan[0].x_m) * 1000
      waypoint[i].y = esp_start_y + (scan[i*stride].y_m - scan[0].y_m) * 1000

    ROOM_WAYPOINT_SCAN_IDX[i] stores the original scan index so the browser
    (playback_dashboard.html) knows how far to advance the map render.
    """
    x0, y0 = scans[0][0], scans[0][1]

    wps   = []    # (esp_x, esp_y, theta, scan_orig_idx)
    for i, (x, y, th, _) in enumerate(scans):
        if i % stride == 0:
            wps.append((
                esp_start_x + (x - x0) * 1000.0,
                esp_start_y + (y - y0) * 1000.0,
                th,
                i,
            ))

    # ── Write header ──────────────────────────────────────────────────────────
    h_text = f"""\
/**
 * room_waypoints.h
 * Auto-generated by tools/parse_room/generate_lidar_map.py — do not edit.
 * Waypoint sequence derived from room.log (Carmen GFS / FLASER, stride={stride}).
 *
 * Coordinate frame: ESP32 dead-reckoning starts at
 *   ({esp_start_x:.0f}, {esp_start_y:.0f}) mm  ≡  first room.log pose ({x0:.3f}, {y0:.3f}) m
 * Matches the espStartX / espStartY fields in tools/dashboard/room_map_data.js.
 */

#ifndef ROOM_WAYPOINTS_H
#define ROOM_WAYPOINTS_H

#include <stdint.h>
#include "../../types.h"

/** Total number of waypoints. */
extern const uint16_t ROOM_WAYPOINT_COUNT;

/** room.log scan index each waypoint was sampled from.
 *  Broadcast as "si" over WebSocket so the browser advances the map render. */
extern const uint16_t ROOM_WAYPOINT_SCAN_IDX[{len(wps)}];

/** Waypoint sequence in ESP32 mm dead-reckoning frame. */
extern const waypoint_t ROOM_WAYPOINTS[{len(wps)}];

/** Starting pose — initialise pose_t from this in app_main. */
extern const pose_t ROOM_WP_START_POSE;

#endif /* ROOM_WAYPOINTS_H */
"""
    with open(out_h, 'w') as f:
        f.write(h_text)

    # ── Write C implementation ─────────────────────────────────────────────────
    wp_lines  = [f"    {{ {ex:.1f}f, {ey:.1f}f, {th:.5f}f, {speed_mm_s:.1f}f }},"
                 for ex, ey, th, _ in wps]
    idx_lines = [str(si) for _, _, _, si in wps]

    c_text = f"""\
/**
 * room_waypoints.c
 * Auto-generated by tools/parse_room/generate_lidar_map.py — do not edit.
 * Source: room.log  |  {len(wps)} waypoints from {len(scans)} scans (stride={stride})
 *
 * Each waypoint is spaced ~{stride * 60:.0f} mm apart along the recorded trajectory.
 * flash footprint: {len(wps) * 16 // 1024} KB (waypoints) + {len(wps) * 2 // 1024 + 1} KB (scan indices)
 */

#include "room_waypoints.h"

const waypoint_t ROOM_WAYPOINTS[{len(wps)}] = {{
{chr(10).join(wp_lines)}
}};

const uint16_t ROOM_WAYPOINT_SCAN_IDX[{len(wps)}] = {{
    {", ".join(idx_lines)}
}};

const uint16_t ROOM_WAYPOINT_COUNT = {len(wps)}u;

const pose_t ROOM_WP_START_POSE = {{
    .x     = {esp_start_x:.1f}f,
    .y     = {esp_start_y:.1f}f,
    .theta = {scans[0][2]:.5f}f,
    .cov   = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
}};
"""
    with open(out_c, 'w') as f:
        f.write(c_text)

    print(f"  Wrote {out_h}")
    print(f"  Wrote {out_c}  ({len(wps)} waypoints, stride={stride})")


# ── Export scan data for browser replay ───────────────────────────────────────
def export_scan_data(scans, ox, oy, W, H, out_path):
    """
    Write room_scan_data.js consumed by lidar_replay.html.
    Stores all poses and ranges so the browser can re-run Bresenham ray-casting
    scan-by-scan, building the occupancy grid incrementally.
    """
    poses  = [[round(x, 4), round(y, 4), round(th, 5)] for x, y, th, _ in scans]
    ranges = [[round(r, 2) for r in rng] for _, _, _, rng in scans]

    meta = {
        "originX":     round(ox, 6),
        "originY":     round(oy, 6),
        "pxPerMeter":  PX_PER_M,
        "width":       W,
        "height":      H,
        "maxRange":    MAX_RANGE,
        "nBeams":      N_BEAMS,
        "beamStartRad": round(BEAM_START_RAD, 6),
        "beamStepRad":  round(BEAM_STEP_RAD, 6),
        "nScans":      len(scans),
    }

    with open(out_path, "w") as f:
        f.write("// Auto-generated by tools/parse_room/generate_lidar_map.py\n")
        f.write("// Source: room.log — do not edit manually.\n")
        f.write("// Loaded by lidar_replay.html for animated scan-by-scan replay.\n\n")
        f.write(f"const SCAN_META   = {json.dumps(meta, indent=2)};\n\n")
        f.write(f"const SCAN_POSES  = {json.dumps(poses)};\n\n")
        f.write(f"const SCAN_RANGES = {json.dumps(ranges)};\n")
    print(f"  Saved: {out_path}  ({len(scans)} scans)")


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    repo = Path(__file__).resolve().parent.parent.parent
    os.chdir(repo)

    if not Path(LOG_FILE).exists():
        print(f"ERROR: {LOG_FILE} not found in {repo}")
        sys.exit(1)

    print(f"Parsing {LOG_FILE} …")
    scans = parse_flaser(LOG_FILE)
    if not scans:
        print("ERROR: no FLASER lines found.")
        sys.exit(1)

    render(scans)
