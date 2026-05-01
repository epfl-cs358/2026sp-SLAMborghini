#!/usr/bin/env python3
"""
slam_sim.py — SLAMborghini dashboard preview simulation  (v2)

Models the exact firmware behaviour introduced in the latest changes:
  • Two-phase drive  (arc-turn → straight)  —  mirrors drive_for_cmd()
  • s_drv live-pose interpolation           —  mirrors scan_task() s_drv logic
  • ROBOT_CLEAR_CELLS = 2 clearance box     —  mirrors frontier_detector.c

Visual markers (visible while driving):
  RED circle    — stale pose: where OLD firmware placed all scans during a drive
  GREEN dot     — live pose:  where NEW firmware integrates each scan (s_drv interp)
  CYAN cross    — current navigation target
  GREEN box     — ROBOT_CLEAR_CELLS=2 safety zone (5×5 cells ≈ 780 mm per side)

Dashed red line connecting the two pose markers shows the improvement magnitude.
The panel displays "stale error" vs "interp error" live — stale error climbs during
a drive while interp error stays near zero for straight phases and is small for arcs.

Controls:
  SPACE   pause / resume
  G       toggle ghost (stale vs live comparison)
  R       reset
  +/-     2× faster / 2× slower  (×0.25 … ×8)
  Q/ESC   quit
"""

import sys
import math
import pygame

# ─── world / simulation parameters ───────────────────────────────────────────
MAP_W = MAP_H   = 10000.0     # world size mm
LIDAR_RAYS      = 360
LIDAR_RANGE     = 5500        # mm
ROBOT_SPEED     = 500.0       # mm/s (real firmware: 346 mm/s)
QT_MARCH_STEP   = 150.0

# ─── quadtree constants (mirror quadtree_map.h) ───────────────────────────────
QT_MAX_DEPTH    = 7
QT_HIT          = 15
QT_MISS         = -6
QT_VAL_MAX      = 40
QT_VAL_MIN      = -40
CELL_MM         = MAP_W / (2 ** (QT_MAX_DEPTH - 1))   # ≈ 156.25 mm per leaf cell

# ─── drive constants (mirror wemos/main.c + frontier_detector.c) ─────────────
ROBOT_CLEAR_CELLS = 2             # frontier_detector.c ROBOT_CLEAR_CELLS
ARC_THRESH_RAD    = math.pi / 4   # main.c TURN_ARC_THRESH_RAD  (π/4 ≈ 45°)
ARC_DURATION      = 0.8           # s — main.c TURN_ARC_MS = 800 ms
PLAN_DELAY        = 0.12          # s — brief pause for frontier detection sim
TURN_RATE         = 2.4           # rad/s angular velocity during arc phase

# ─── drive phase constants ───────────────────────────────────────────────────
PHASE_PLAN  = 0
PHASE_TURN  = 1
PHASE_DRIVE = 2
PHASE_LABEL = {PHASE_PLAN: "PLANNING", PHASE_TURN: "ARC TURN", PHASE_DRIVE: "STRAIGHT"}
PHASE_COLOR = {PHASE_PLAN: (140, 160, 220), PHASE_TURN: (255, 170, 50), PHASE_DRIVE: (80, 230, 120)}

# ─── window layout ────────────────────────────────────────────────────────────
WIN_W, WIN_H    = 1420, 820
MAP_PX          = 720         # square canvas side in pixels
MAP_OX, MAP_OY  = 20, 48      # canvas top-left offset

# ─── palette ─────────────────────────────────────────────────────────────────
C_BG        = ( 18,  18,  25)
C_MAP_BG    = ( 38,  38,  50)
C_PANEL     = ( 28,  28,  38)
C_UNK       = (244, 162,  97)    # orange  — unknown / unvisited
C_FREE      = (255, 255, 255)    # white   — free
C_ROBOT     = ( 67,  97, 238)    # blue    — robot body
C_HEAD      = (  0, 212, 255)    # cyan    — heading stripe
C_SCAN      = (251, 191,  36)    # amber   — lidar hit dot
C_TRAIL     = (  0, 255, 255)    # cyan    — path trail
C_TEXT      = (200, 200, 200)
C_TITLE     = (140, 160, 220)
C_DIM       = ( 90,  90, 110)
C_GREEN     = ( 80, 230, 120)
C_ORANGE    = (255, 190,  60)
C_STALE     = (255,  70,  70)    # red     — stale pose (old firmware)
C_LIVE      = ( 60, 230,  80)    # green   — interpolated pose (new firmware)
C_TARGET    = (  0, 220, 255)    # cyan    — navigation target
C_CLEAR_BOX = ( 90, 190,  70)    # green   — safety clearance box

# Depth spectrum: one strong colour per quadtree level
DEPTH_COLORS = [
    (255,  50,  50),   # 1 — red      10 000 mm
    (255, 145,   0),   # 2 — orange    5 000 mm
    (230, 225,   0),   # 3 — yellow    2 500 mm
    ( 50, 230,  50),   # 4 — green     1 250 mm
    (  0, 210, 255),   # 5 — cyan        625 mm
    ( 80,  80, 255),   # 6 — blue        312 mm
    (210,   0, 255),   # 7 — purple      156 mm  ← leaf
]
DEPTH_LW = [3, 3, 2, 2, 1, 1, 1]   # border line widths per depth

# ─── room geometry ───────────────────────────────────────────────────────────
#
#  10 m × 10 m world. Rooms occupy [1000,9000]×[2000,8000].
#  Central wall at x=5000 with doorway at y=4300–5700.
#  One obstacle box per room.
#
WALLS = [
    # outer boundary
    ((1000, 2000), (9000, 2000)),
    ((9000, 2000), (9000, 8000)),
    ((9000, 8000), (1000, 8000)),
    ((1000, 8000), (1000, 2000)),
    # central dividing wall — south half (y=2000..4300)
    ((5000, 2000), (5000, 4300)),
    # central dividing wall — north half (y=5700..8000)
    ((5000, 5700), (5000, 8000)),
    # obstacle box — east room
    ((6500, 2800), (7800, 2800)),
    ((7800, 2800), (7800, 4200)),
    ((7800, 4200), (6500, 4200)),
    ((6500, 4200), (6500, 2800)),
    # obstacle box — west room
    ((2000, 5800), (3200, 5800)),
    ((3200, 5800), (3200, 7100)),
    ((3200, 7100), (2000, 7100)),
    ((2000, 7100), (2000, 5800)),
]

# Exploration waypoints — tours both rooms, loops
WAYPOINTS = [
    (3000, 5000),
    (2000, 3000),
    (4500, 3000),
    (4500, 7500),
    (2000, 7500),
    (3000, 5000),
    (5200, 5000),
    (8500, 5000),
    (8500, 3000),
    (5500, 3000),
    (5500, 7500),
    (8500, 7500),
    (5200, 5000),
]


# ─── utilities ───────────────────────────────────────────────────────────────

def normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a <= -math.pi: a += 2.0 * math.pi
    return a


# ─── quadtree ─────────────────────────────────────────────────────────────────

class _Node:
    __slots__ = ('ch', 'val', 'dep')
    def __init__(self, dep: int):
        self.ch  = [None, None, None, None]   # 0=NW 1=NE 2=SW 3=SE
        self.val = 0
        self.dep = dep


class QuadTree:
    """
    Log-odds quadtree — mirrors esp32s3/src/quadtree_map.c exactly.

    Quadrant convention (same as C code):
        0 = NW (x < cx, y >= cy)
        1 = NE (x >= cx, y >= cy)
        2 = SW (x < cx, y < cy)
        3 = SE (x >= cx, y < cy)

    pack() returns DFS pre-order so parent fills are always drawn before
    child fills — each child correctly overwrites its parent's sub-area.
    """

    def __init__(self):
        self.root   = _Node(1)
        self._cache = None

    def update(self, x: float, y: float, delta: int):
        if not (0 <= x < MAP_W and 0 <= y < MAP_H):
            return
        self._upd(self.root, 0.0, MAP_W, 0.0, MAP_H, x, y, delta)
        self._cache = None

    def pack(self):
        """DFS pre-order → [(xmin, ymin, size, depth, value), ...]"""
        if self._cache is None:
            self._cache = []
            self._pk(self.root, 0.0, MAP_W, 0.0, MAP_H, self._cache)
        return self._cache

    def node_count(self) -> int:
        return self._nc(self.root)

    @staticmethod
    def _quad(xmn, xmx, ymn, ymx, x, y) -> int:
        cx = (xmn + xmx) * 0.5
        cy = (ymn + ymx) * 0.5
        e = 1 if x >= cx else 0
        n = 1 if y >= cy else 0
        return e if n else (2 + e)

    @staticmethod
    def _child_bounds(xmn, xmx, ymn, ymx, q):
        cx = (xmn + xmx) * 0.5
        cy = (ymn + ymx) * 0.5
        if q == 0: return xmn, cx,  cy,  ymx
        if q == 1: return cx,  xmx, cy,  ymx
        if q == 2: return xmn, cx,  ymn, cy
        return           cx,  xmx, ymn, cy

    def _upd(self, nd, xmn, xmx, ymn, ymx, x, y, d):
        if nd.dep >= QT_MAX_DEPTH:
            nd.val = max(QT_VAL_MIN, min(QT_VAL_MAX, nd.val + d))
            return
        q = self._quad(xmn, xmx, ymn, ymx, x, y)
        if nd.ch[q] is None:
            nd.ch[q] = _Node(nd.dep + 1)
        self._upd(nd.ch[q], *self._child_bounds(xmn, xmx, ymn, ymx, q), x, y, d)

    def _pk(self, nd, xmn, xmx, ymn, ymx, out):
        out.append((xmn, ymn, xmx - xmn, nd.dep, nd.val))
        cx = (xmn + xmx) * 0.5
        cy = (ymn + ymx) * 0.5
        if nd.ch[0]: self._pk(nd.ch[0], xmn, cx,  cy,  ymx, out)
        if nd.ch[1]: self._pk(nd.ch[1], cx,  xmx, cy,  ymx, out)
        if nd.ch[2]: self._pk(nd.ch[2], xmn, cx,  ymn, cy,  out)
        if nd.ch[3]: self._pk(nd.ch[3], cx,  xmx, ymn, cy,  out)

    def _nc(self, nd) -> int:
        return 1 + sum(self._nc(c) for c in nd.ch if c)


# ─── LiDAR ────────────────────────────────────────────────────────────────────

def _ray_seg_dist(ox, oy, dx, dy, x1, y1, x2, y2) -> float:
    bx, by = x2 - x1, y2 - y1
    den = dx * by - dy * bx
    if abs(den) < 1e-9:
        return math.inf
    t = ((x1 - ox) * by - (y1 - oy) * bx) / den
    s = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    if t > 1e-4 and 0.0 <= s <= 1.0:
        return t
    return math.inf


def lidar_scan(rx: float, ry: float):
    """Cast LIDAR_RAYS beams. Returns [(theta_rad, range_mm), ...]."""
    results = []
    step = math.tau / LIDAR_RAYS
    for i in range(LIDAR_RAYS):
        th = i * step
        dx, dy = math.cos(th), math.sin(th)
        r = float(LIDAR_RANGE)
        for (x1, y1), (x2, y2) in WALLS:
            d = _ray_seg_dist(rx, ry, dx, dy, x1, y1, x2, y2)
            if d < r:
                r = d
        results.append((th, r))
    return results


def integrate_scan(qt: QuadTree, scan, rx: float, ry: float):
    """
    Ray-march QT_MISS then QT_HIT at endpoint.
    Mirrors lidar_to_map.c — scan angles are world-frame.
    rx, ry: the pose used for integration (may differ from true robot pos
    during drives when using live interpolation).
    """
    qt.update(rx, ry, QT_MISS)
    for th, r in scan:
        if r < 100 or r >= LIDAR_RANGE:
            continue
        dx = math.cos(th)
        dy = math.sin(th)
        t = QT_MARCH_STEP
        while t < r - QT_MARCH_STEP:
            qt.update(rx + dx * t, ry + dy * t, QT_MISS)
            t += QT_MARCH_STEP
        qt.update(rx + dx * r, ry + dy * r, QT_HIT)


# ─── coordinate conversion ────────────────────────────────────────────────────

def w2s(wx: float, wy: float):
    """World mm → screen pixel. Y flipped (world Y-up, screen Y-down)."""
    sx = int(MAP_OX + (wx / MAP_W) * MAP_PX)
    sy = int(MAP_OY + (1.0 - wy / MAP_H) * MAP_PX)
    return sx, sy


# ─── drawing helpers ──────────────────────────────────────────────────────────

def draw_dashed_line(surf, color, p1, p2, dash=6, width=1):
    """Draw a dashed line from p1 to p2."""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    length = math.hypot(dx, dy)
    if length < 1:
        return
    segs = max(1, int(length / dash))
    for s in range(0, segs, 2):
        ax = int(p1[0] + dx * s / segs)
        ay = int(p1[1] + dy * s / segs)
        bx = int(p1[0] + dx * min(s + 1, segs) / segs)
        by = int(p1[1] + dy * min(s + 1, segs) / segs)
        pygame.draw.line(surf, color, (ax, ay), (bx, by), width)


# ─── map rendering ────────────────────────────────────────────────────────────

def draw_map(surf, qt: QuadTree):
    """
    Two-pass quadtree rendering.

    Pass 1 — fills (DFS pre-order):
        Parent before children → child fills overwrite parent's sub-area.
        Colour = occupancy: white=free, orange=unknown, dark gray=occupied.

    Pass 2 — depth-coloured borders (same order, on top of all fills):
        Every structural boundary is visible regardless of map state.
        One spectrum colour per depth level, thicker lines for coarser levels.
    """
    nodes = qt.pack()
    scale = MAP_PX / MAP_W

    for xmn, ymn, sz, dep, val in nodes:
        spx = sz * scale
        if spx < 0.5:
            continue
        px  = int(MAP_OX + xmn * scale)
        py  = int(MAP_OY + (1.0 - (ymn + sz) / MAP_H) * MAP_PX)
        ipx = max(1, int(spx))
        if val == 0:
            fill = C_UNK
        elif val < 0:
            fill = C_FREE
        else:
            t    = min(1.0, val / QT_VAL_MAX)
            g    = int(200 * (1.0 - t))
            fill = (g, g, g)
        pygame.draw.rect(surf, fill, (px, py, ipx, ipx))

    for xmn, ymn, sz, dep, val in nodes:
        spx = sz * scale
        if spx < 1.5:
            continue
        px  = int(MAP_OX + xmn * scale)
        py  = int(MAP_OY + (1.0 - (ymn + sz) / MAP_H) * MAP_PX)
        ipx = max(1, int(spx))
        pygame.draw.rect(surf, DEPTH_COLORS[dep - 1], (px, py, ipx, ipx), DEPTH_LW[dep - 1])


# ─── target + clearance box ───────────────────────────────────────────────────

def draw_target(surf, tx: float, ty: float):
    """
    Draw the current navigation target with its ROBOT_CLEAR_CELLS=2 safety zone.

    The safety zone is a (2R+1)² = 5×5 cell box centred on the target.
    Each cell is CELL_MM ≈ 156 mm, so the box is 5×156 = 780 mm per side.
    frontier_detector.c checks that every cell in this box is free before
    accepting the target — is_safe_cell() with ROBOT_CLEAR_CELLS=2 Chebyshev.
    """
    scale    = MAP_PX / MAP_W
    half     = (2 * ROBOT_CLEAR_CELLS + 1) * CELL_MM / 2.0   # 390.625 mm

    # Screen corners of the clearance box
    sx1, sy_top = w2s(tx - half, ty + half)
    sx2, sy_bot = w2s(tx + half, ty - half)
    bw = max(2, sx2 - sx1)
    bh = max(2, sy_bot - sy_top)

    # Semi-transparent fill
    fill_s = pygame.Surface((bw, bh), pygame.SRCALPHA)
    fill_s.fill((*C_CLEAR_BOX, 22))
    surf.blit(fill_s, (sx1, sy_top))

    # Box border
    pygame.draw.rect(surf, C_CLEAR_BOX, (sx1, sy_top, bw, bh), 1)

    # Internal cell grid lines (5×5)
    cell_px = int(CELL_MM * scale)
    for k in range(1, 5):
        gx = sx1 + k * cell_px
        gy = sy_top + k * cell_px
        if sx1 < gx < sx1 + bw:
            pygame.draw.line(surf, (*C_CLEAR_BOX, 60), (gx, sy_top), (gx, sy_bot))
        if sy_top < gy < sy_top + bh:
            pygame.draw.line(surf, (*C_CLEAR_BOX, 60), (sx1, gy), (sx1 + bw, gy))

    # Crosshair
    sx, sy = w2s(tx, ty)
    arm = 12
    pygame.draw.circle(surf, C_TARGET, (sx, sy), 8, 2)
    pygame.draw.line(surf, C_TARGET, (sx - arm, sy), (sx + arm, sy), 2)
    pygame.draw.line(surf, C_TARGET, (sx, sy - arm), (sx, sy + arm), 2)


# ─── drive state markers ─────────────────────────────────────────────────────

def draw_drive_markers(surf, drv_x0: float, drv_y0: float,
                       interp_rx: float, interp_ry: float,
                       show_ghost: bool):
    """
    Visualise the live-vs-stale pose comparison during a drive.

    GREEN dot  = scan_task integration origin (new firmware — s_drv interpolation)
    RED circle = stale pose (old firmware — all scans during this drive went here)
    Dashed line between them shows the magnitude of the improvement.

    The dashed line is the "error saved" by the live interpolation fix:
      stale_err  = |actual_pos - drv_x0|  (climbs linearly during drive)
      interp_err = |actual_pos - interp|  (near-zero for straight, small for arc)
    """
    ix, iy = w2s(interp_rx, interp_ry)

    if show_ghost:
        gx, gy = w2s(drv_x0, drv_y0)
        # Dashed line from stale to live
        draw_dashed_line(surf, C_STALE, (gx, gy), (ix, iy), dash=5, width=1)
        # Stale circle (old code)
        pygame.draw.circle(surf, C_STALE, (gx, gy), 9, 2)

    # Live dot (new code) — always visible
    pygame.draw.circle(surf, C_LIVE,           (ix, iy), 6)
    pygame.draw.circle(surf, (255, 255, 255),  (ix, iy), 6, 1)


# ─── robot sprite ─────────────────────────────────────────────────────────────

def draw_robot(surf, rx: float, ry: float, theta: float):
    scale = MAP_PX / MAP_W
    bl = max(10, int(300 * scale))
    bw = max( 6, int(180 * scale))

    body = pygame.Surface((bl, bw), pygame.SRCALPHA)
    body.fill((*C_ROBOT, 255))
    sw = max(2, bl // 5)
    pygame.draw.rect(body, (*C_HEAD, 255), (bl - sw, 0, sw, bw))

    rotated = pygame.transform.rotate(body, math.degrees(theta))
    cx, cy  = w2s(rx, ry)
    rect    = rotated.get_rect(center=(cx, cy))
    surf.blit(rotated, rect)


# ─── side panel ───────────────────────────────────────────────────────────────

def draw_panel(surf, fonts, rx, ry, rtheta,
               phase, phase_t,
               drv_active, drv_x0, drv_y0,
               interp_rx, interp_ry,
               stale_err, interp_err,
               show_ghost,
               target_x, target_y,
               node_count, speed_mult, paused, fps):

    font_sm, font_md, font_lg = fonts
    px  = MAP_OX + MAP_PX + 20
    pw  = WIN_W - px - 6
    pygame.draw.rect(surf, C_PANEL, (px, 0, pw, WIN_H))

    y = 14

    def txt(s, f=None, c=C_TEXT):
        nonlocal y
        f = f or font_sm
        surf.blit(f.render(s, True, c), (px + 12, y))
        y += f.get_height() + 3

    def sep():
        nonlocal y
        y += 4
        pygame.draw.line(surf, (55, 55, 72), (px + 8, y), (px + pw - 12, y))
        y += 7

    def swatch(label, color, circle=False):
        nonlocal y
        lx = px + 12
        if circle:
            pygame.draw.circle(surf, color, (lx + 6, y + 6), 5)
        else:
            pygame.draw.rect(surf, color, (lx, y + 1, 12, 12))
            pygame.draw.rect(surf, (80, 80, 100), (lx, y + 1, 12, 12), 1)
        surf.blit(font_sm.render(label, True, C_TEXT), (lx + 18, y))
        y += font_sm.get_height() + 3

    # ── header ────────────────────────────────────────────────────────────
    txt("SLAMborghini",      font_lg, C_TITLE)
    txt("Dashboard Sim  v2", font_sm, C_DIM)
    sep()

    # ── telemetry ─────────────────────────────────────────────────────────
    txt("TELEMETRY", font_md, C_TITLE)
    txt(f"X      {rx:8.0f} mm")
    txt(f"Y      {ry:8.0f} mm")
    txt(f"θ      {math.degrees(rtheta):8.1f} °")
    txt(f"Target ({target_x:.0f}, {target_y:.0f}) mm", c=C_TARGET)
    sep()

    # ── drive phase ───────────────────────────────────────────────────────
    txt("DRIVE PHASE", font_md, C_TITLE)
    p_label = PHASE_LABEL[phase]
    p_color = PHASE_COLOR[phase]
    txt(p_label, font_md, p_color)
    txt(f"Phase time  {phase_t:5.2f} s")
    if phase == PHASE_TURN:
        tx_wp = WAYPOINTS[0][0]   # just for display
        hdg_err = normalize_angle(
            math.atan2(target_y - ry, target_x - rx) - rtheta)
        txt(f"Heading err {math.degrees(hdg_err):+6.1f} °")
    elif phase == PHASE_PLAN:
        txt("Computing next target")
    else:
        dist_rem = math.hypot(target_x - rx, target_y - ry)
        txt(f"Dist remain {dist_rem:6.0f} mm")
    sep()

    # ── pose sync ─────────────────────────────────────────────────────────
    txt("POSE SYNC  (s_drv)", font_md, C_TITLE)
    if drv_active:
        txt("INTERPOLATING", font_md, C_LIVE)
        txt(f"Start  ({drv_x0:.0f}, {drv_y0:.0f})")
        txt(f"Live   ({interp_rx:.0f}, {interp_ry:.0f})")
        txt(f"Stale err  {stale_err:6.0f} mm", c=C_STALE)
        txt(f"Interp err {interp_err:6.0f} mm", c=C_LIVE)
        saved = max(0.0, stale_err - interp_err)
        txt(f"Error saved  {saved:5.0f} mm", c=C_GREEN)
    else:
        txt("IDLE", font_md, C_DIM)
        txt("scan_task uses s_pose", c=C_DIM)
    sep()

    # ── map ───────────────────────────────────────────────────────────────
    txt("MAP", font_md, C_TITLE)
    txt(f"Nodes    {node_count}")
    txt(f"Depth    {QT_MAX_DEPTH}  (max)")
    txt(f"Leaf     ≈{CELL_MM:.0f} mm/cell")
    txt(f"Clear    {ROBOT_CLEAR_CELLS} cells  (5×5 box)")
    sep()

    # ── simulation ────────────────────────────────────────────────────────
    txt("SIMULATION", font_md, C_TITLE)
    txt(f"Speed    ×{speed_mult:.2g}")
    txt(f"FPS      {fps}")
    sc, cc = ("PAUSED", C_ORANGE) if paused else ("RUNNING", C_GREEN)
    txt(sc, font_md, cc)
    sep()

    # ── legend ────────────────────────────────────────────────────────────
    txt("LEGEND", font_md, C_TITLE)
    swatch("Unknown / unvisited",  C_UNK)
    swatch("Free space",           C_FREE)
    swatch("Wall / occupied",      (50, 50, 50))
    swatch("LiDAR hit",            C_SCAN,      circle=True)
    swatch("Path trail",           C_TRAIL,     circle=True)
    swatch("Robot",                C_ROBOT)
    swatch("Nav target",           C_TARGET,    circle=True)
    swatch("Clearance box (5×5)",  C_CLEAR_BOX)
    if show_ghost:
        swatch("Stale pose  (OLD)", C_STALE,    circle=True)
    swatch("Live pose   (NEW)",    C_LIVE,      circle=True)
    sep()

    # ── controls ─────────────────────────────────────────────────────────
    txt("CONTROLS", font_md, C_TITLE)
    for line in ("SPACE  pause/resume",
                 "G      toggle ghost",
                 "R      reset",
                 "+/-    speed",
                 "Q/ESC  quit"):
        txt(line, font_sm, C_DIM)
    sep()

    # ── depth colour key ──────────────────────────────────────────────────
    txt("DEPTH COLOURS", font_md, C_TITLE)
    bx = px + 12
    entries = [
        (1, "1  red     10 000 mm"),
        (2, "2  orange   5 000 mm"),
        (3, "3  yellow   2 500 mm"),
        (4, "4  green    1 250 mm"),
        (5, "5  cyan       625 mm"),
        (6, "6  blue       312 mm"),
        (7, "7  purple     156 mm  ← leaf"),
    ]
    for dep, label in entries:
        color = DEPTH_COLORS[dep - 1]
        lw    = DEPTH_LW[dep - 1]
        pygame.draw.rect(surf, C_UNK,  (bx, y + 1, 14, 12))
        pygame.draw.rect(surf, color,  (bx, y + 1, 14, 12), lw)
        surf.blit(font_sm.render(label, True, color), (bx + 18, y))
        y += font_sm.get_height() + 3


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    pygame.init()
    pygame.display.set_caption("SLAMborghini — SLAM Dashboard Simulation v2")
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    clock  = pygame.time.Clock()

    try:
        font_sm = pygame.font.SysFont("Courier New", 11)
        font_md = pygame.font.SysFont("Courier New", 13)
        font_lg = pygame.font.SysFont("Courier New", 16, bold=True)
    except Exception:
        font_sm = font_md = font_lg = pygame.font.SysFont(None, 14)
    fonts = (font_sm, font_md, font_lg)

    # ── simulation state ──────────────────────────────────────────────────
    qt: QuadTree
    rx = ry = rtheta = 0.0
    wp_i   = 0
    scan_acc: list
    trail:    list
    scan_t    = 0.0
    sim_time  = 0.0
    speed_mult = 1.0
    paused     = False
    show_ghost = True

    # Drive state machine — mirrors wemos/main.c plan_task + scan_task
    phase          = PHASE_PLAN
    phase_t        = 0.0
    drive_heading  = 0.0   # target heading for current drive
    drv_active     = False
    drv_x0 = drv_y0 = 0.0
    drv_t0         = 0.0
    drv_speed      = ROBOT_SPEED
    target_x = target_y = 0.0

    def reset():
        nonlocal qt, rx, ry, rtheta, wp_i, scan_acc, trail, scan_t, sim_time
        nonlocal phase, phase_t, drive_heading
        nonlocal drv_active, drv_x0, drv_y0, drv_t0, drv_speed
        nonlocal target_x, target_y
        qt        = QuadTree()
        rx, ry    = float(WAYPOINTS[0][0]), float(WAYPOINTS[0][1])
        rtheta    = 0.0
        wp_i      = 1
        scan_acc  = []
        trail     = [(rx, ry)]
        scan_t    = 0.0
        sim_time  = 0.0
        phase     = PHASE_PLAN
        phase_t   = 0.0
        drv_active = False
        drv_x0 = drv_y0 = 0.0
        drive_heading = 0.0
        target_x  = float(WAYPOINTS[1][0])
        target_y  = float(WAYPOINTS[1][1])

    reset()

    last_ms  = pygame.time.get_ticks()
    fps_cnt  = fps_val = 0
    fps_last = last_ms

    while True:
        now     = pygame.time.get_ticks()
        dt      = min((now - last_ms) / 1000.0, 0.05)
        last_ms = now

        fps_cnt += 1
        if now - fps_last >= 1000:
            fps_val = fps_cnt; fps_cnt = 0; fps_last = now

        # ── events ────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if ev.type == pygame.KEYDOWN:
                k = ev.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit(); sys.exit()
                elif k == pygame.K_SPACE:
                    paused = not paused
                elif k == pygame.K_r:
                    reset()
                elif k == pygame.K_g:
                    show_ghost = not show_ghost
                elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    speed_mult = min(8.0, speed_mult * 2)
                elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    speed_mult = max(0.25, speed_mult / 2)

        # ── simulation step ───────────────────────────────────────────────
        if not paused:
            step_s = dt * speed_mult
            sim_time += step_s

            # ── drive state machine ────────────────────────────────────────
            # Mirrors the plan_task / drive_for_cmd logic in wemos/main.c:
            #   PLAN  → compute next waypoint heading
            #   TURN  → arc phase (imu_drive_and_track for TURN_ARC_MS)
            #   DRIVE → straight phase (imu_drive_and_track for drive_ms)

            if phase == PHASE_PLAN:
                # ── PLANNING PHASE ─────────────────────────────────────────
                # Brief pause simulating frontier detection + command_gen
                phase_t += step_s
                if phase_t >= PLAN_DELAY:
                    # Compute desired heading to next waypoint
                    tx_wp = float(WAYPOINTS[wp_i % len(WAYPOINTS)][0])
                    ty_wp = float(WAYPOINTS[wp_i % len(WAYPOINTS)][1])
                    target_x, target_y = tx_wp, ty_wp

                    desired = math.atan2(ty_wp - ry, tx_wp - rx)
                    err     = normalize_angle(desired - rtheta)

                    # ARM s_drv  (mirrors plan_task before drive_for_cmd)
                    drv_x0    = rx
                    drv_y0    = ry
                    drv_t0    = sim_time
                    drv_speed = ROBOT_SPEED
                    drv_active = True
                    drive_heading = desired

                    if abs(err) > ARC_THRESH_RAD:
                        # Large heading error → arc turn first
                        phase   = PHASE_TURN
                        phase_t = 0.0
                    else:
                        # Small heading error → straight drive immediately
                        phase   = PHASE_DRIVE
                        phase_t = 0.0

            elif phase == PHASE_TURN:
                # ── ARC TURN PHASE ─────────────────────────────────────────
                # Mirrors imu_drive_and_track(heading, TURN_ARC_MS) in
                # drive_for_cmd — robot turns while moving slightly forward
                # (arc motion).  scan_task uses s_drv interpolation throughout.
                phase_t += step_s
                err = normalize_angle(drive_heading - rtheta)

                # Rotate toward target heading
                rot   = min(abs(err), TURN_RATE * step_s)
                rtheta = normalize_angle(rtheta + math.copysign(rot, err))

                # Small forward creep during arc (car drives forward while turning)
                creep_speed = ROBOT_SPEED * 0.35   # ~35% forward during turn
                rx += math.cos(rtheta) * creep_speed * step_s
                ry += math.sin(rtheta) * creep_speed * step_s

                # Transition to straight once arc is done or heading aligned
                if phase_t >= ARC_DURATION or abs(normalize_angle(drive_heading - rtheta)) < 0.05:
                    phase   = PHASE_DRIVE
                    phase_t = 0.0
                    # NOTE: s_drv is NOT re-armed here (mirrors firmware v1 behaviour).
                    # drv_x0/y0 still holds the pre-arc start position.
                    # This means the straight-phase interpolation has the chord
                    # approximation error from the arc displacement.

            elif phase == PHASE_DRIVE:
                # ── STRAIGHT DRIVE PHASE ──────────────────────────────────
                # Mirrors imu_drive_and_track(heading, drive_ms) — robot
                # moves forward while scan_task integrates at interpolated pose.
                phase_t += step_s
                tx_wp = float(WAYPOINTS[wp_i % len(WAYPOINTS)][0])
                ty_wp = float(WAYPOINTS[wp_i % len(WAYPOINTS)][1])
                dx_wp = tx_wp - rx
                dy_wp = ty_wp - ry
                dist  = math.hypot(dx_wp, dy_wp)

                if dist < 80:
                    # Reached waypoint → DISARM s_drv, go to planning
                    drv_active = False
                    wp_i      += 1
                    phase      = PHASE_PLAN
                    phase_t    = 0.0
                else:
                    # Small steering correction toward waypoint
                    desired_now = math.atan2(dy_wp, dx_wp)
                    steer_err   = normalize_angle(desired_now - rtheta)
                    steer_step  = min(abs(steer_err), 0.6 * step_s) * math.copysign(1, steer_err)
                    rtheta      = normalize_angle(rtheta + steer_step)

                    # Forward movement
                    rx += math.cos(rtheta) * ROBOT_SPEED * step_s
                    ry += math.sin(rtheta) * ROBOT_SPEED * step_s

            # ── path trail ────────────────────────────────────────────────
            if not trail or math.hypot(rx - trail[-1][0], ry - trail[-1][1]) > 60:
                trail.append((rx, ry))
                if len(trail) > 400:
                    trail.pop(0)

            # ── LiDAR scan at 10 Hz sim-time ─────────────────────────────
            scan_t += step_s
            if scan_t >= 0.1:
                scan_t -= 0.1

                # Compute pose used by scan_task:
                #   If driving: interpolated position via s_drv (new firmware)
                #   If idle:    use current robot position (= s_pose)
                if drv_active:
                    elapsed_s = sim_time - drv_t0
                    # s_drv formula from scan_task in wemos/main.c:
                    #   pos = start + speed * elapsed * dir(live_theta)
                    scan_rx = drv_x0 + drv_speed * elapsed_s * math.cos(rtheta)
                    scan_ry = drv_y0 + drv_speed * elapsed_s * math.sin(rtheta)
                else:
                    scan_rx, scan_ry = rx, ry

                scan = lidar_scan(rx, ry)   # cast from actual robot position
                integrate_scan(qt, scan, scan_rx, scan_ry)

                # Scan hit dot overlay (world-frame, using actual ray endpoints)
                for th, r in scan:
                    if 100 < r < LIDAR_RANGE:
                        scan_acc.append((rx + r * math.cos(th),
                                         ry + r * math.sin(th)))
                if len(scan_acc) > 6000:
                    scan_acc = scan_acc[-6000:]

        # ── derive display values ─────────────────────────────────────────
        if drv_active:
            elapsed_s = sim_time - drv_t0
            interp_rx = drv_x0 + drv_speed * elapsed_s * math.cos(rtheta)
            interp_ry = drv_y0 + drv_speed * elapsed_s * math.sin(rtheta)
        else:
            interp_rx, interp_ry = rx, ry

        stale_err  = math.hypot(rx - drv_x0, ry - drv_y0) if drv_active else 0.0
        interp_err = math.hypot(rx - interp_rx, ry - interp_ry) if drv_active else 0.0

        # ── render ────────────────────────────────────────────────────────
        screen.fill(C_BG)
        pygame.draw.rect(screen, C_MAP_BG, (MAP_OX, MAP_OY, MAP_PX, MAP_PX))

        # 1. Quadtree map (main visual)
        draw_map(screen, qt)

        scale = MAP_PX / MAP_W

        # 2. LiDAR hit dots
        for hx, hy in scan_acc[-4000:]:
            sx = int(MAP_OX + hx * scale)
            sy = int(MAP_OY + (1.0 - hy / MAP_H) * MAP_PX)
            if MAP_OX <= sx < MAP_OX + MAP_PX and MAP_OY <= sy < MAP_OY + MAP_PX:
                pygame.draw.circle(screen, C_SCAN, (sx, sy), 2)

        # 3. Path trail
        if len(trail) > 1:
            pts = [w2s(x, y) for x, y in trail]
            pygame.draw.lines(screen, C_TRAIL, False, pts, 2)

        # 4. Navigation target + clearance box
        draw_target(screen, target_x, target_y)

        # 5. Drive state markers (stale ghost vs live interpolated pose)
        if drv_active:
            draw_drive_markers(screen, drv_x0, drv_y0,
                               interp_rx, interp_ry, show_ghost)

        # 6. Robot
        draw_robot(screen, rx, ry, rtheta)

        # Map border + title
        pygame.draw.rect(screen, (65, 65, 88), (MAP_OX, MAP_OY, MAP_PX, MAP_PX), 2)
        label = font_sm.render(
            f"10 m × 10 m world  |  depth {QT_MAX_DEPTH}  |  leaf ≈{CELL_MM:.0f} mm"
            f"  |  clearance {ROBOT_CLEAR_CELLS}-cell ({(2*ROBOT_CLEAR_CELLS+1)}×{(2*ROBOT_CLEAR_CELLS+1)} box)",
            True, C_DIM)
        screen.blit(label, (MAP_OX, MAP_OY - 20))

        # 7. Side panel
        draw_panel(screen, fonts, rx, ry, rtheta,
                   phase, phase_t,
                   drv_active, drv_x0, drv_y0,
                   interp_rx, interp_ry,
                   stale_err, interp_err,
                   show_ghost, target_x, target_y,
                   qt.node_count(), speed_mult, paused, fps_val)

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()
