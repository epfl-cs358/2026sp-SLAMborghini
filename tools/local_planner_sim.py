#!/usr/bin/env python3
"""
tools/local_planner_sim.py
Local planner validation — baseline vs LP head-to-head.

Four progressively harder scenarios, both robots run simultaneously.
  Baseline  — direct waypoint following, no collision awareness.
  LP        — full reactive / ESCAPE / RECOVER planner (local_planner.c port).

Logged metrics per run
  collisions     physical contact events (robot centre within ROBOT_R of obstacle)
  avg_cte_mm     mean |cross-track error| from the global path
  reactive_acts  mode-REACTIVE activations
  escape_count   mode-ESCAPE entries
  replan_count   times LP set replan_requested
  path_mm        total distance travelled
  cycles         cycles to goal, or MAX_CYCLES on timeout

Controls:  SPACE pause   N next scenario   R reset   Q quit
"""

import sys, math
from dataclasses import dataclass, field
from typing import List, Tuple

try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False
    print("pygame not found — running headless")

# ─── world ────────────────────────────────────────────────────────────────────
SIM_W, SIM_H   = 6000.0, 4000.0
MAX_CYCLES      = 800
GOAL_REACH_MM   = 250.0

# ─── movement model (mirrors firmware: each LP tick ≈ one command execution) ──
MOVE_MM         = 100.0   # normal forward step per LP cycle (= LP_REACTIVE_FWD_MM)
ESC_REV_MM      = 60.0    # reverse step per cycle in ESCAPE phase-0

# ─── quadtree ─────────────────────────────────────────────────────────────────
QT_DEPTH = 7
QT_HIT   = 15
QT_MISS  = -6
QT_VMAX  = 40
QT_VMIN  = -40
CELL_MM  = SIM_W / (2 ** (QT_DEPTH - 1))   # ≈ 93.75 mm
MSTEP    = CELL_MM * 0.55                   # marking density

# ─── LP constants (mirror local_planner.c) ────────────────────────────────────
LP_ROBOT_R    = 120.0
LP_WHEEL      = 150.0
LP_MAX_STEER  = 0.524    # 30°
LP_SPEED      = 150.0
LP_REV_SPEED  = 80.0
LP_FWD_NEAR   = 150.0
LP_FWD_MID    = 300.0
LP_FWD_FAR    = 500.0
LP_CLUSTER    = 3
LP_NARROW_MRG = 50.0
LP_FBD_HMAX   = 0.611    # 35° cap
LP_FBD_BUF    = 0.087    # 5° buffer
LP_ROLL_TOTAL = 550.0
LP_ROLL_NEAR  = 200.0
LP_NEAR_STEP  = 20.0
LP_FAR_STEP   = 80.0
LP_ROLL_NMAX  = 20
LP_OCC_HARD   = 10
LP_DIFF_COST  = 0.45
LP_UNK_COST   = 0.15
LP_MAX_DIFF   = 1.50
LP_CTE_NORM   = 500.0
LP_HERR_NORM  = 1.571
LP_W_PATH     = 0.60
LP_W_JERK     = 0.40
LP_JERK_NORM  = 0.873
LP_HYSTER     = 0.10
LP_TIE        = 0.10
LP_COLL_SC    = 1e9
LP_REACT_FWD  = 100.0
LP_WP_REACH   = 200.0
LP_BLKD_THR   = 10
LP_ESC_TO     = 30
LP_ESC_RSTEP  = 0.262    # 15° rotation per cycle
LP_ESC_FULL   = 6.283    # 360°
LP_ESC_RCYC   = 8
LP_SC_UNK     = 0.60
LP_SC_NARR    = 0.70
LP_SC_RECV    = 0.50
LP_UNK_THR    = 5
LP_RECV_STAB  = 5
LP_SIG_CLAMP  = 0.05
LP_SIG_RECV   = 0.25
LP_TH_ALPHA   = 0.3
LP_HIST_MAX   = 10
LP_HIST_NORM  = 5

# ─── modes ────────────────────────────────────────────────────────────────────
PP=0; REACT=1; ESCAPE=2; RECV=3; STOP=4
MODE_NAME  = {PP:'PP', REACT:'REACT', ESCAPE:'ESC', RECV:'RECV', STOP:'STOP'}
MODE_CLR   = {PP:(80,210,80), REACT:(255,200,0), ESCAPE:(255,70,70),
              RECV:(80,180,255), STOP:(180,180,180)}

# ─── display ──────────────────────────────────────────────────────────────────
WIN_W, WIN_H = 1300, 820
MAP_PX_W = 620
MAP_PX_H = int(MAP_PX_W * SIM_H / SIM_W)   # 413 px
MAP_OX, MAP_OY = 18, 72
PNL_X  = MAP_OX + MAP_PX_W + 22
PNL_W  = WIN_W - PNL_X - 8

C_BG   = (18, 18, 25);  C_MPBG = (30, 30, 42)
C_TXT  = (200,200,200); C_DIM  = (80,80,100)
C_TTL  = (140,160,220); C_PNL  = (26,26,38)
C_BASE = (255,100,100); C_LP   = (80,220,255)
C_GOAL = (80,255,80);   C_PATH = (130,130,60)
C_OBS  = (255,200,50);  C_FREE = (220,220,220)
C_UNK  = (58,58,75);    C_GRN  = (80,230,120)
C_ORG  = (255,190,60)



class _N:
    __slots__ = ('ch', 'val', 'd')
    def __init__(self, d): self.ch = [None]*4; self.val = 0; self.d = d


class QT:
    def __init__(self, W=SIM_W, H=SIM_H):
        self.W = W; self.H = H
        self.root = _N(1)

    def query(self, x, y) -> int:
        if not (0 <= x < self.W and 0 <= y < self.H): return 0
        return self._q(self.root, 0.0, self.W, 0.0, self.H, x, y)

    def update(self, x, y, d):
        if not (0 <= x < self.W and 0 <= y < self.H): return
        self._u(self.root, 0.0, self.W, 0.0, self.H, x, y, d)

    def mark_rect(self, x1, y1, x2, y2, d=QT_VMAX):
        s = MSTEP
        x = x1
        while x <= x2 + s:
            cx = min(x, x2)
            y = y1
            while y <= y2 + s:
                self.update(cx, min(y, y2), d)
                y += s
            x += s

    def fill_free(self):
        """Pre-mark entire world as free (7× QT_MISS saturates to QT_VMIN)."""
        s = MSTEP * 1.5
        x = s * 0.5
        while x < self.W:
            y = s * 0.5
            while y < self.H:
                for _ in range(7):
                    self.update(x, y, QT_MISS)
                y += s
            x += s

    def pack(self):
        out = []; self._p(self.root, 0.0, self.W, 0.0, self.H, out); return out

    # ── internals ─────────────────────────────────────────────────────────────
    @staticmethod
    def _qd(xn, xx, yn, yx, x, y):
        cx = (xn+xx)*0.5; cy = (yn+yx)*0.5
        return (1 if x >= cx else 0) if y >= cy else (2+(1 if x >= cx else 0))

    @staticmethod
    def _cb(xn, xx, yn, yx, q):
        cx = (xn+xx)*0.5; cy = (yn+yx)*0.5
        if q == 0: return xn, cx, cy, yx
        if q == 1: return cx, xx, cy, yx
        if q == 2: return xn, cx, yn, cy
        return cx, xx, yn, cy

    def _u(self, nd, xn, xx, yn, yx, x, y, d):
        if nd.d >= QT_DEPTH:
            nd.val = max(QT_VMIN, min(QT_VMAX, nd.val + d)); return
        q = self._qd(xn, xx, yn, yx, x, y)
        if nd.ch[q] is None: nd.ch[q] = _N(nd.d + 1)
        self._u(nd.ch[q], *self._cb(xn, xx, yn, yx, q), x, y, d)

    def _q(self, nd, xn, xx, yn, yx, x, y):
        if nd is None: return 0
        if nd.d >= QT_DEPTH: return nd.val
        q = self._qd(xn, xx, yn, yx, x, y)
        if nd.ch[q] is None: return nd.val   # inherit parent
        return self._q(nd.ch[q], *self._cb(xn, xx, yn, yx, q), x, y)

    def _p(self, nd, xn, xx, yn, yx, out):
        out.append((xn, yn, xx-xn, yx-yn, nd.val))
        cx = (xn+xx)*0.5; cy = (yn+yx)*0.5
        for i, (ax,bx,ay,by) in enumerate([(xn,cx,cy,yx),(cx,xx,cy,yx),(xn,cx,yn,cy),(cx,xx,yn,cy)]):
            if nd.ch[i]: self._p(nd.ch[i], ax, bx, ay, by, out)


# ══════════════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════════════

def ang(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


def pt_seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2-x1, y2-y1
    if dx == 0 and dy == 0: return math.hypot(px-x1, py-y1)
    t = max(0.0, min(1.0, ((px-x1)*dx + (py-y1)*dy) / (dx*dx+dy*dy)))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))


def pt_rect_dist(px, py, x1, y1, x2, y2):
    cx = max(x1, min(px, x2)); cy = max(y1, min(py, y2))
    return math.hypot(px-cx, py-cy)


def path_cte(path, x, y, theta):
    if not path: return 0.0, 0.0
    best, bestd = 0, 1e9
    for i, (wx, wy) in enumerate(path):
        d = math.hypot(wx-x, wy-y)
        if d < bestd: bestd = d; best = i
    cte = bestd
    seg_th = math.atan2(path[best][1]-y, path[best][0]-x)
    if best+1 < len(path):
        ax, ay = path[best]; bx, by = path[best+1]
        sdx, sdy = bx-ax, by-ay; L = math.hypot(sdx, sdy)
        if L > 1.0:
            cte = ((x-ax)*sdy - (y-ay)*sdx) / L
            seg_th = math.atan2(sdy, sdx)
    return cte, ang(theta - seg_th)


# ══════════════════════════════════════════════════════════════════════════════
# Local Planner (port of local_planner.c)
# ══════════════════════════════════════════════════════════════════════════════

class LP:
    def __init__(self, robot_r=LP_ROBOT_R):
        self.robot_r = robot_r
        self.inflate_r = robot_r
        self.sigma = 0.0
        self.theta_f = 0.0
        self.th_hist = [0.0] * LP_HIST_MAX
        self.th_idx = 0; self.th_cnt = 0
        self.mode = PP
        self.wp_idx = 0
        self.prev_steer = 0.0; self.prev_score = LP_COLL_SC
        self.esc_phase = 0; self.esc_rev_cyc = 0
        self.esc_heading = 0.0; self.esc_rot_acc = 0.0
        self.esc_start = 0
        self.recv_stable = 0; self.blk_cyc = 0
        self.replan_req = False; self.cyc = 0

    def reset_wp(self):
        self.wp_idx = 0; self.prev_steer = 0.0; self.prev_score = LP_COLL_SC

    # ── Step 1: pose filter ───────────────────────────────────────────────────
    def _filt(self, raw):
        delta = ang(raw - self.theta_f)
        self.theta_f = ang(self.theta_f + LP_TH_ALPHA * delta)
        self.th_hist[self.th_idx] = raw
        self.th_idx = (self.th_idx + 1) % LP_HIST_MAX
        if self.th_cnt < LP_HIST_MAX: self.th_cnt += 1
        return self.theta_f

    def _sigma(self):
        n = LP_HIST_MAX if self.mode == RECV else LP_HIST_NORM
        n = min(n, self.th_cnt)
        if n < 2: return self.sigma
        ss = cs = 0.0
        for i in range(n):
            idx = (self.th_idx + LP_HIST_MAX - n + i) % LP_HIST_MAX
            ss += math.sin(self.th_hist[idx]); cs += math.cos(self.th_hist[idx])
        mean = math.atan2(ss/n, cs/n)
        var = sum(
            ang(self.th_hist[(self.th_idx+LP_HIST_MAX-n+i)%LP_HIST_MAX] - mean)**2
            for i in range(n)
        ) / n
        ns = math.sqrt(var)
        d = ns - self.sigma
        if d >  LP_SIG_CLAMP: ns = self.sigma + LP_SIG_CLAMP
        if d < -LP_SIG_CLAMP: ns = self.sigma - LP_SIG_CLAMP
        self.sigma = ns; return ns

    # ── Step 9: RECOVER transitions ───────────────────────────────────────────
    def _upd_recv(self, sigma):
        if sigma > LP_SIG_RECV:
            if self.mode != ESCAPE: self.mode = RECV
            self.recv_stable = 0
        elif self.mode == RECV:
            self.recv_stable += 1
            if self.recv_stable >= LP_RECV_STAB: self.mode = PP

    # ── Step 3: local window ──────────────────────────────────────────────────
    def _window(self, qt, rx, ry, rth, ir):
        fp_r = ir * 0.65
        for i in range(4):
            a = rth + i * math.pi/2
            if qt.query(rx + fp_r*math.cos(a), ry + fp_r*math.sin(a)) > 0:
                return True, 0, 0, [], 0.0
        angs  = [-0.7854, -0.3927, 0.0, 0.3927, 0.7854]
        dists = [LP_FWD_NEAR, LP_FWD_MID, LP_FWD_FAR]
        occ = unk = 0; bearings = []
        for a in angs:
            for d in dists:
                cx = rx + d*math.cos(rth+a); cy = ry + d*math.sin(rth+a)
                v = qt.query(cx, cy)
                if v > 0:
                    occ += 1
                    bearings.append(ang(math.atan2(cy-ry, cx-rx) - rth))
                elif v == 0:
                    unk += 1
        # lateral free_width at LP_FWD_NEAR ahead
        fx = rx + LP_FWD_NEAR*math.cos(rth); fy = ry + LP_FWD_NEAR*math.sin(rth)
        lx = -math.sin(rth); ly = math.cos(rth)
        lw = rw = 0.0
        for dd in range(50, 450, 50):
            if qt.query(fx+dd*lx, fy+dd*ly) > 0: break
            lw = float(dd)
        for dd in range(50, 450, 50):
            if qt.query(fx-dd*lx, fy-dd*ly) > 0: break
            rw = float(dd)
        return False, occ, unk, bearings, lw+rw

    # ── Step 5a: hard reject ──────────────────────────────────────────────────
    def _hard_rej(self, steer, bearings):
        if abs(steer) > LP_MAX_STEER: return True
        if bearings:
            mn = min(bearings); mx = max(bearings)
            ctr = 0.5*(mn+mx)
            half = min(0.5*(mx-mn) + LP_FBD_BUF, LP_FBD_HMAX)
            if abs(ang(steer - ctr)) < half: return True
        return False

    # ── Step 5b: rollout ──────────────────────────────────────────────────────
    def _rollout(self, qt, rx, ry, rth, steer, path):
        x, y, h = rx, ry, rth
        dist = diff_pen = 0.0; min_cl = 1e9; steps = 0
        while dist < LP_ROLL_TOTAL and steps < LP_ROLL_NMAX:
            step = LP_NEAR_STEP if dist < LP_ROLL_NEAR else LP_FAR_STEP
            h = ang(h + step * math.tan(steer) / LP_WHEEL)
            x += step*math.cos(h); y += step*math.sin(h)
            dist += step; steps += 1
            v = qt.query(x, y)
            if v > LP_OCC_HARD:   return LP_COLL_SC, 0.0, False
            if v > 0:             diff_pen += LP_DIFF_COST
            elif v == 0:          diff_pen += LP_UNK_COST
            if diff_pen > LP_MAX_DIFF: return LP_COLL_SC, 0.0, False
            cl = float(-v)
            if cl < min_cl: min_cl = cl
        cte, herr = path_cte(path, x, y, h)
        np_ = min(1.0, abs(cte)/LP_CTE_NORM*0.5 + abs(herr)/LP_HERR_NORM*0.5)
        jk  = min(1.0, abs(steer - self.prev_steer)/LP_JERK_NORM)
        sc  = LP_W_PATH*np_ + LP_W_JERK*jk
        return sc, min_cl, True

    # ── Step 5: REACTIVE ──────────────────────────────────────────────────────
    def _reactive(self, qt, rx, ry, rth, ir, bearings, fw, path):
        narrow = fw < 2.0*ir + LP_NARROW_MRG
        wide   = [-25,-12, 0,12,25]
        narr   = [-18, -9, 0, 9,18]
        degs   = narr if narrow else wide
        best_sc = LP_COLL_SC; best_st = None; best_cl = 0.0
        for d in degs:
            st = math.radians(d)
            if self._hard_rej(st, bearings): continue
            sc, cl, ok = self._rollout(qt, rx, ry, rth, st, path)
            if not ok: continue
            better = sc < best_sc - LP_HYSTER
            if not better and best_st is not None:
                if abs(sc - best_sc) < LP_TIE and cl > best_cl: better = True
            if best_st is None or better:
                best_sc = sc; best_st = st; best_cl = cl
        if best_st is None: return False, 0.0, 0.0, rth, LP_SPEED
        steer = best_st; kept = False
        if self.prev_score < LP_COLL_SC and best_sc >= self.prev_score - LP_HYSTER:
            steer = self.prev_steer; kept = True
        nh = ang(rth + steer)
        tx = LP_REACT_FWD * math.cos(nh); ty = LP_REACT_FWD * math.sin(nh)
        self.prev_steer = steer
        if not kept: self.prev_score = best_sc
        return True, tx, ty, nh, LP_SPEED

    # ── Step 6: ESCAPE ────────────────────────────────────────────────────────
    def _escape(self, qt, rx, ry, rth, ir, path):
        if self.cyc - self.esc_start >= LP_ESC_TO:
            self.replan_req = True; return False, 0.0, 0.0, rth, 0.0
        if self.esc_rot_acc >= LP_ESC_FULL:
            self.replan_req = True; return False, 0.0, 0.0, rth, 0.0
        if self.esc_phase == 0:
            rh = ang(rth + math.pi)
            tx = ESC_REV_MM*math.cos(rh); ty = ESC_REV_MM*math.sin(rh)
            self.esc_rev_cyc += 1
            if self.esc_rev_cyc >= LP_ESC_RCYC: self.esc_phase = 1
            return True, tx, ty, rh, LP_REV_SPEED
        self.esc_rot_acc += LP_ESC_RSTEP
        th = ang(self.esc_heading + self.esc_rot_acc)
        fp_occ, occ, unk, bear, fw = self._window(qt, rx, ry, th, ir)
        if not fp_occ and occ < LP_CLUSTER:
            ok, tx, ty, h, sp = self._reactive(qt, rx, ry, th, ir, bear, fw, path)
            if ok:
                self.mode = REACT; self.prev_score = LP_COLL_SC
                return True, tx, ty, h, sp
        return True, 0.0, 0.0, th, LP_SPEED * 0.5   # rotate in place

    # ── Step 4: Pure Pursuit stub ─────────────────────────────────────────────
    def _pp(self, path, rx, ry, rth):
        if not path: return 0.0, 0.0, rth, LP_SPEED
        while self.wp_idx + 1 < len(path):
            dx = path[self.wp_idx][0] - rx; dy = path[self.wp_idx][1] - ry
            if math.hypot(dx, dy) < LP_WP_REACH: self.wp_idx += 1
            else: break
        if self.wp_idx >= len(path): self.wp_idx = len(path)-1
        wx, wy = path[self.wp_idx]
        dx, dy = wx-rx, wy-ry
        if math.hypot(dx, dy) < 1.0: return 0.0, 0.0, rth, 0.0
        h = math.atan2(dy, dx)
        return dx, dy, h, LP_SPEED

    # ── Step 10: speed scaling ────────────────────────────────────────────────
    def _scale(self, spd, unk, fw, ir, recv):
        s = 1.0
        if unk >= LP_UNK_THR:               s *= LP_SC_UNK
        if fw < 2.0*ir + LP_NARROW_MRG:    s *= LP_SC_NARR
        if recv:                            s *= LP_SC_RECV
        return max(0.0, spd * s)

    # ── Main update ───────────────────────────────────────────────────────────
    def update(self, qt, rx, ry, rth, path, override=False):
        """
        Returns (cmd_valid, tx, ty, heading, speed, mode).
        tx/ty are relative displacement (mm); heading absolute (rad).
        """
        if override: return False, 0.0, 0.0, rth, 0.0, self.mode
        th_f = self._filt(rth)
        sigma = self._sigma()
        ir = self.robot_r + 1.5 * sigma * LP_FWD_NEAR
        self.inflate_r = ir
        recv = (self.mode == RECV)
        self._upd_recv(sigma)
        if self.mode == RECV: recv = True
        fp_occ, occ, unk, bear, fw = self._window(qt, rx, ry, th_f, ir)
        if fp_occ:
            self.mode = STOP; self.cyc += 1
            return True, 0.0, 0.0, th_f, 0.0, STOP
        has_cl = occ >= LP_CLUSTER
        tx = ty = 0.0; heading = th_f; speed = 0.0; valid = False
        if self.mode == ESCAPE:
            run, tx, ty, heading, speed = self._escape(qt, rx, ry, th_f, ir, path)
            if not run: self.mode = PP
            valid = True
        elif not has_cl:
            if self.mode != RECV: self.mode = PP
            self.blk_cyc = 0
            tx, ty, heading, speed = self._pp(path, rx, ry, th_f)
            valid = True
        else:
            if self.mode != RECV: self.mode = REACT
            self.blk_cyc += 1
            ok, tx, ty, heading, speed = self._reactive(qt, rx, ry, th_f, ir, bear, fw, path)
            if not ok:
                self.mode = ESCAPE
                self.esc_phase = 0; self.esc_rev_cyc = 0
                self.esc_heading = th_f; self.esc_rot_acc = 0.0
                self.esc_start = self.cyc
                run, tx, ty, heading, speed = self._escape(qt, rx, ry, th_f, ir, path)
                if not run: self.mode = PP
            else:
                self.blk_cyc = 0
            valid = True
        if self.blk_cyc >= LP_BLKD_THR: self.replan_req = True
        if valid and speed > 0:
            speed = self._scale(speed, unk, fw, ir, recv)
        self.cyc += 1
        return valid, tx, ty, heading, speed, self.mode


# ══════════════════════════════════════════════════════════════════════════════
# Scenario + Metrics
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class Scenario:
    name:  str
    desc:  str
    boxes: List[Tuple]   # (x1,y1,x2,y2) obstacles
    start: Tuple         # (x, y, theta)
    goal:  Tuple         # (x, y)
    path:  List[Tuple]   # global waypoints [(x,y), ...]


@dataclass
class Metrics:
    collisions:    int   = 0
    cte_sum:       float = 0.0
    cte_n:         int   = 0
    reactive_acts: int   = 0
    escape_count:  int   = 0
    replan_count:  int   = 0
    path_mm:       float = 0.0
    cycles:        int   = 0
    goal_reached:  bool  = False
    mode_hist:     List[int] = field(default_factory=list)

    @property
    def avg_cte(self): return self.cte_sum / max(1, self.cte_n)

    def row(self, label):
        status = "GOAL" if self.goal_reached else f"T/O {self.cycles}cyc"
        return (f"{label:8s}  coll={self.collisions}  cte={self.avg_cte:5.0f}mm"
                f"  react={self.reactive_acts}  esc={self.escape_count}"
                f"  replan={self.replan_count}  path={self.path_mm:6.0f}mm  {status}")


def build_scenarios():
    cy = SIM_H / 2   # 2000 mm

    # ── S1: Open corridor ─────────────────────────────────────────────────────
    s1 = Scenario(
        name="S1: Open corridor",
        desc="No obstacles. Establishes baseline. Both robots reach goal in PP mode.",
        boxes=[],
        start=(200, cy, 0.0), goal=(5800, cy),
        path=[(200, cy), (2900, cy), (5800, cy)],
    )

    # ── S2: Box in path (ESCAPE scenario) ─────────────────────────────────────
    # 800 × 800 mm box centred on y=2000. Symmetric bearing spread forces
    # forbidden sector to cover all ±25° candidates → LP enters ESCAPE.
    # Baseline drives straight into it. LP reverses + rotates + recovers.
    s2 = Scenario(
        name="S2: Box in path  (ESCAPE avoidance)",
        desc="Symmetric 800×800 box. All reactive candidates forbidden → ESCAPE. Baseline collides.",
        boxes=[(2400, 1600, 3200, 2400)],
        start=(200, cy, 0.0), goal=(5800, cy),
        path=[(200, cy), (2800, cy), (5800, cy)],
    )

    # ── S3: Thin asymmetric box (REACTIVE scenario) ───────────────────────────
    # 200 mm tall box mostly above y=2000. Occupied-cell bearings are clustered
    # on the positive (upper) side → forbidden sector excludes negative candidates.
    # LP steers ≤ -12° under the box; baseline collides with the lower edge.
    s3 = Scenario(
        name="S3: Thin off-axis box  (REACTIVE avoidance)",
        desc="200 mm tall box, asymmetric bearings. LP reactive steer clears it. Baseline clips lower edge.",
        boxes=[(2200, 1950, 3200, 2150)],
        start=(200, cy, 0.0), goal=(5800, cy),
        path=[(200, cy), (2800, cy), (5800, cy)],
    )

    # ── S4: U-trap (ESCAPE full rotation) ─────────────────────────────────────
    # Three boxes form a pocket. Even after reversing and rotating, the LP
    # needs a large heading change to find a clear exit. Tests ESCAPE timeout /
    # replan request.
    s4 = Scenario(
        name="S4: U-trap  (ESCAPE + rotation)",
        desc="Three-sided pocket. LP reverses, rotates up to 360° looking for exit. Replan on timeout.",
        boxes=[
            (2800, 1600, 3400, 2400),   # back wall
            (2000, 2200, 3400, 2700),   # top jaw
            (2000, 1300, 3400, 1800),   # bottom jaw
        ],
        start=(200, cy, 0.0), goal=(5800, cy),
        path=[(200, cy), (2800, cy), (5800, cy)],
    )

    return [s1, s2, s3, s4]


def build_qt(sc: Scenario):
    qt = QT()
    qt.fill_free()                                          # whole world known-free
    border = 100.0
    qt.mark_rect(0, 0, SIM_W, border)                      # outer walls
    qt.mark_rect(0, SIM_H-border, SIM_W, SIM_H)
    qt.mark_rect(0, 0, border, SIM_H)
    qt.mark_rect(SIM_W-border, 0, SIM_W, SIM_H)
    for b in sc.boxes:
        qt.mark_rect(*b)
    return qt


def is_collision(rx, ry, sc: Scenario):
    # Count a collision only when the robot centre enters the obstacle box.
    # LP's footprint check (fp_r ≈ 78mm) stops the robot before its centre
    # reaches the boundary; the baseline has no such check → only baseline
    # centres ever enter boxes.
    for x1, y1, x2, y2 in sc.boxes:
        if x1 <= rx <= x2 and y1 <= ry <= y2:
            return True
    # Outer boundary: robot driven entirely out of the world
    if rx < 10 or rx > SIM_W - 10 or ry < 10 or ry > SIM_H - 10:
        return True
    return False


# ══════════════════════════════════════════════════════════════════════════════
# Simulation agents
# ══════════════════════════════════════════════════════════════════════════════

class Agent:
    def __init__(self, sc: Scenario, qt: QT, use_lp: bool):
        self.sc = sc; self.qt = qt; self.use_lp = use_lp
        self.x, self.y, self.theta = sc.start
        self.wp_idx = 0
        self.lp = LP() if use_lp else None
        self.m = Metrics()
        self.trail = [(self.x, self.y)]
        self.prev_mode = PP
        self.active = True

    def step(self):
        if not self.active: return
        if self.m.cycles >= MAX_CYCLES:
            self.active = False; return

        if self.use_lp:
            ok, tx, ty, heading, speed, mode = self.lp.update(
                self.qt, self.x, self.y, self.theta, self.sc.path)
            # mode transitions
            if mode == REACT and self.prev_mode != REACT:
                self.m.reactive_acts += 1
            if mode == ESCAPE and self.prev_mode != ESCAPE:
                self.m.escape_count += 1
            if self.lp.replan_req:
                self.m.replan_count += 1
                self.lp.replan_req = False
            self.prev_mode = mode

            if not ok: self.m.cycles += 1; return
            # Apply movement
            dist = math.hypot(tx, ty)
            if dist > 1.0:
                move = min(dist, MOVE_MM)
                nx = self.x + move * math.cos(heading)
                ny = self.y + move * math.sin(heading)
            else:
                # rotate-in-place (ESCAPE phase 1 rotation step)
                self.theta = heading
                self.m.cycles += 1; return
        else:
            # Baseline: advance waypoints, drive straight toward next
            path = self.sc.path
            while self.wp_idx + 1 < len(path):
                dx = path[self.wp_idx][0] - self.x
                dy = path[self.wp_idx][1] - self.y
                if math.hypot(dx, dy) < LP_WP_REACH: self.wp_idx += 1
                else: break
            idx = min(self.wp_idx, len(path)-1)
            dx, dy = path[idx][0]-self.x, path[idx][1]-self.y
            dist = math.hypot(dx, dy)
            if dist < 1.0: self.m.cycles += 1; return
            heading = math.atan2(dy, dx)
            nx = self.x + MOVE_MM * math.cos(heading)
            ny = self.y + MOVE_MM * math.sin(heading)

        # Collision check
        if is_collision(nx, ny, self.sc):
            if self.m.collisions == 0:
                self.m.collisions += 1
            self.active = False
            self.m.cycles += 1
            return

        moved = math.hypot(nx-self.x, ny-self.y)
        self.m.path_mm += moved
        self.x = nx; self.y = ny; self.theta = heading

        cte, _ = path_cte(self.sc.path, self.x, self.y, self.theta)
        self.m.cte_sum += abs(cte); self.m.cte_n += 1

        if len(self.trail) == 0 or math.hypot(self.x-self.trail[-1][0], self.y-self.trail[-1][1]) > 60:
            self.trail.append((self.x, self.y))
            if len(self.trail) > 600: self.trail = self.trail[-600:]

        if self.use_lp and self.lp:
            self.m.mode_hist.append(self.lp.mode)
        else:
            self.m.mode_hist.append(PP)

        # Goal check
        gx, gy = self.sc.goal
        if math.hypot(self.x-gx, self.y-gy) < GOAL_REACH_MM:
            self.m.goal_reached = True; self.active = False

        self.m.cycles += 1


# ══════════════════════════════════════════════════════════════════════════════
# Drawing
# ══════════════════════════════════════════════════════════════════════════════

def w2s(wx, wy):
    sx = int(MAP_OX + (wx / SIM_W) * MAP_PX_W)
    sy = int(MAP_OY + (1.0 - wy / SIM_H) * MAP_PX_H)
    return sx, sy


def draw_map(surf, qt: QT, sc: Scenario):
    scale_x = MAP_PX_W / SIM_W; scale_y = MAP_PX_H / SIM_H
    for xmn, ymn, sw, sh, val in qt.pack():
        spx = sw * scale_x; spy = sh * scale_y
        if spx < 0.4: continue
        px = int(MAP_OX + xmn * scale_x)
        py = int(MAP_OY + (1.0 - (ymn+sh)/SIM_H) * MAP_PX_H)
        iw = max(1, int(spx)); ih = max(1, int(spy))
        fill = C_UNK if val == 0 else (C_FREE if val < 0 else (80, max(20, 80-val*2), max(20, 80-val*2)))
        pygame.draw.rect(surf, fill, (px, py, iw, ih))
    # obstacle outlines
    for x1,y1,x2,y2 in sc.boxes:
        sx1,sy1 = w2s(x1,y2); sx2,sy2 = w2s(x2,y1)
        pygame.draw.rect(surf, C_OBS, (sx1, sy1, sx2-sx1, sy2-sy1), 2)


def draw_path(surf, path):
    if len(path) < 2: return
    pts = [w2s(x, y) for x, y in path]
    for i in range(len(pts)-1):
        dx, dy = pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1]
        L = math.hypot(dx, dy)
        if L < 1: continue
        n = max(1, int(L / 14))
        for k in range(n):
            if k % 2 == 0:
                ax = int(pts[i][0] + dx*k/n); ay = int(pts[i][1] + dy*k/n)
                bx = int(pts[i][0] + dx*(k+0.5)/n); by = int(pts[i][1] + dy*(k+0.5)/n)
                pygame.draw.line(surf, C_PATH, (ax,ay), (bx,by), 1)


def draw_robot(surf, x, y, theta, color, label, mode=PP):
    sx, sy = w2s(x, y)
    mc = MODE_CLR.get(mode, C_DIM)
    pygame.draw.circle(surf, mc,    (sx, sy), 10)
    pygame.draw.circle(surf, color, (sx, sy), 10, 2)
    ex = int(sx + 14*math.cos(theta)); ey = int(sy - 14*math.sin(theta))
    pygame.draw.line(surf, color, (sx,sy), (ex,ey), 2)
    if HAS_PYGAME:
        fnt = pygame.font.SysFont("Courier New", 9)
        surf.blit(fnt.render(label, True, color), (sx+12, sy-6))


def draw_trail(surf, trail, color, width=2):
    if len(trail) < 2: return
    pts = [w2s(x, y) for x, y in trail]
    pygame.draw.lines(surf, color, False, pts, width)


def draw_mode_bar(surf, hist, ox, oy, w, h, font):
    pygame.draw.rect(surf, (40,40,55), (ox, oy, w, h))
    n = len(hist)
    if n == 0: return
    bar_w = max(1, w // max(n, 1))
    for i, m in enumerate(hist[-w:]):
        bx = ox + i * bar_w
        c = MODE_CLR.get(m, C_DIM)
        pygame.draw.rect(surf, c, (bx, oy, bar_w, h))
    pygame.draw.rect(surf, C_DIM, (ox, oy, w, h), 1)


def draw_panel(surf, fonts, sc, base_m, lp_m, paused, speed_mult, cyc, n_cyc):
    sm, md, lg = fonts
    pygame.draw.rect(surf, C_PNL, (PNL_X, 0, PNL_W, WIN_H))
    y = 10
    def txt(s, f=None, c=C_TXT):
        nonlocal y
        f = f or sm
        surf.blit(f.render(s, True, c), (PNL_X+10, y)); y += f.get_height()+3
    def sep():
        nonlocal y; y += 4
        pygame.draw.line(surf, (55,55,72), (PNL_X+6,y), (PNL_X+PNL_W-10,y)); y += 7

    txt("SLAMborghini — LP Sim", lg, C_TTL)
    txt(sc.name, md, C_TTL); y += 2
    # wrap description
    words = sc.desc.split()
    line = ""; lines = []
    for w in words:
        if len(line)+len(w)+1 > 42: lines.append(line); line = w
        else: line = (line+" "+w).strip()
    if line: lines.append(line)
    for l in lines: txt(l, sm, C_DIM)
    sep()

    # Metrics
    def metric_row(label, bv, lv, good_low=True):
        nonlocal y
        diff = lv - bv
        if good_low: better = diff < -1
        else:        better = diff > 1
        c_l = C_GRN if better else (C_ORG if abs(diff)>0.5 else C_TXT)
        surf.blit(sm.render(f"{label:12s}", True, C_TXT),       (PNL_X+10, y))
        surf.blit(sm.render(f"BASE {bv:>7.1f}", True, C_BASE),  (PNL_X+10+104, y))
        surf.blit(sm.render(f"LP  {lv:>7.1f}", True, c_l),      (PNL_X+10+200, y))
        y += sm.get_height()+3

    txt("METRICS", md, C_TTL)
    metric_row("Collisions",   float(base_m.collisions),    float(lp_m.collisions))
    metric_row("Avg CTE mm",   base_m.avg_cte,              lp_m.avg_cte)
    metric_row("Path length",  base_m.path_mm,              lp_m.path_mm)
    metric_row("Cycles",       float(base_m.cycles),        float(lp_m.cycles))
    sep()

    txt("LP ONLY", md, C_TTL)
    txt(f"  Reactive activations : {lp_m.reactive_acts}")
    txt(f"  ESCAPE entries       : {lp_m.escape_count}")
    txt(f"  Replan requests      : {lp_m.replan_count}")
    txt(f"  Goal reached  BASE   : {'YES' if base_m.goal_reached else 'NO'}", c=C_GRN if base_m.goal_reached else C_BASE)
    txt(f"  Goal reached  LP     : {'YES' if lp_m.goal_reached else 'NO'}", c=C_GRN if lp_m.goal_reached else C_ORG)
    sep()

    # Mode history bar (LP)
    txt("LP MODE HISTORY", md, C_TTL)
    bar_h = 18; bar_w = PNL_W - 20
    draw_mode_bar(surf, lp_m.mode_hist, PNL_X+10, y, bar_w, bar_h, sm)
    y += bar_h + 4
    # Legend
    legend_x = PNL_X+10
    for mode, name in MODE_NAME.items():
        pygame.draw.rect(surf, MODE_CLR[mode], (legend_x, y+1, 12, 10))
        surf.blit(sm.render(name, True, C_TXT), (legend_x+15, y))
        legend_x += 60
    y += sm.get_height() + 8
    sep()

    # Sim status
    txt("SIMULATION", md, C_TTL)
    txt(f"  Cycle {cyc}/{n_cyc}")
    sc_label = "PAUSED" if paused else "RUNNING"
    txt(f"  {sc_label}   speed ×{speed_mult:.1f}", c=C_ORG if paused else C_GRN)
    sep()

    txt("CONTROLS", md, C_TTL)
    for line in ("SPACE  pause / resume",
                 "N      next scenario",
                 "R      restart current",
                 "+/-    speed  ×0.25 … ×8",
                 "Q/ESC  quit"):
        txt(line, sm, C_DIM)


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def print_summary(sc, base: Metrics, lp: Metrics):
    print(f"\n{'═'*70}")
    print(f"  {sc.name}")
    print(f"{'═'*70}")
    print("  " + base.row("BASELINE"))
    print("  " + lp.row("LP"))
    print(f"  LP reactive={lp.reactive_acts}  escape={lp.escape_count}  replan={lp.replan_count}")


def run_headless(scenarios):
    for sc in scenarios:
        qt = build_qt(sc)
        base = Agent(sc, qt, use_lp=False)
        lp_a = Agent(sc, qt, use_lp=True)
        for _ in range(MAX_CYCLES):
            base.step(); lp_a.step()
            if not base.active and not lp_a.active: break
        print_summary(sc, base.m, lp_a.m)


def main():
    scenarios = build_scenarios()
    if not HAS_PYGAME or "--headless" in sys.argv:
        run_headless(scenarios); return

    pygame.init()
    pygame.display.set_caption("SLAMborghini — Local Planner Sim")
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    clock  = pygame.time.Clock()

    try:
        sm = pygame.font.SysFont("Courier New", 11)
        md = pygame.font.SysFont("Courier New", 13)
        lg = pygame.font.SysFont("Courier New", 16, bold=True)
    except Exception:
        sm = md = lg = pygame.font.SysFont(None, 13)
    fonts = (sm, md, lg)

    sc_idx     = 0
    speed_mult = 1.0
    paused     = False

    def init_scenario(idx):
        sc  = scenarios[idx]
        qt  = build_qt(sc)
        ba  = Agent(sc, qt, use_lp=False)
        lpa = Agent(sc, qt, use_lp=True)
        return sc, qt, ba, lpa

    sc, qt, base_a, lp_a = init_scenario(sc_idx)

    last_ms = pygame.time.get_ticks()

    while True:
        now    = pygame.time.get_ticks()
        dt     = min((now - last_ms)/1000.0, 0.1)
        last_ms = now

        # ── events ────────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                print_summary(sc, base_a.m, lp_a.m)
                pygame.quit(); sys.exit()
            if ev.type == pygame.KEYDOWN:
                k = ev.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
                    print_summary(sc, base_a.m, lp_a.m)
                    pygame.quit(); sys.exit()
                elif k == pygame.K_SPACE:
                    paused = not paused
                elif k == pygame.K_n:
                    print_summary(sc, base_a.m, lp_a.m)
                    sc_idx = (sc_idx + 1) % len(scenarios)
                    sc, qt, base_a, lp_a = init_scenario(sc_idx)
                elif k == pygame.K_r:
                    sc, qt, base_a, lp_a = init_scenario(sc_idx)
                elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    speed_mult = min(8.0, speed_mult * 2)
                elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    speed_mult = max(0.25, speed_mult / 2)

        # ── simulation steps ──────────────────────────────────────────────────
        if not paused:
            steps_per_frame = max(1, int(speed_mult * 2))
            for _ in range(steps_per_frame):
                base_a.step()
                lp_a.step()

        # auto-advance when both done
        if not base_a.active and not lp_a.active:
            pass  # wait for user to press N

        # ── render ────────────────────────────────────────────────────────────
        screen.fill(C_BG)
        pygame.draw.rect(screen, C_MPBG, (MAP_OX, MAP_OY, MAP_PX_W, MAP_PX_H))

        draw_map(screen, qt, sc)
        draw_path(screen, sc.path)
        draw_trail(screen, base_a.trail, C_BASE, 1)
        draw_trail(screen, lp_a.trail,  C_LP,   2)

        # goal marker
        gx, gy = sc.goal
        gsx, gsy = w2s(gx, gy)
        pygame.draw.circle(screen, C_GOAL, (gsx, gsy), 10, 2)
        for arm in [(12,0),(0,12),(-12,0),(0,-12)]:
            pygame.draw.line(screen, C_GOAL, (gsx,gsy), (gsx+arm[0],gsy-arm[1]), 2)

        # robots
        bmode = PP
        lmode = lp_a.lp.mode if lp_a.lp else PP
        draw_robot(screen, base_a.x, base_a.y, base_a.theta, C_BASE, "BASE", bmode)
        draw_robot(screen, lp_a.x,   lp_a.y,   lp_a.theta,  C_LP,   "LP",   lmode)

        # map border + title
        pygame.draw.rect(screen, (60,60,80), (MAP_OX, MAP_OY, MAP_PX_W, MAP_PX_H), 2)
        lbl = sm.render(f"6 m × 4 m  |  BASELINE (red)  vs  LOCAL PLANNER (cyan)  |  {sc.name}", True, C_DIM)
        screen.blit(lbl, (MAP_OX, MAP_OY-18))

        cyc = max(base_a.m.cycles, lp_a.m.cycles)
        draw_panel(screen, fonts, sc, base_a.m, lp_a.m, paused, speed_mult, cyc, MAX_CYCLES)

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()
