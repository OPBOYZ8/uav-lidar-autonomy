#!/usr/bin/env python3
"""
UAV LiDAR Stack — Goal-Directed Navigator (Motor-Self-Detection-Fixed Edition)
===============================================================================
Root-cause chain fixed in this version:

  BUG 1 — Motor housing self-detection (NEW FIX)
  -----------------------------------------------
  Motor housings sit at a 3-D range of 0.306 m from the VLP-16.  With the
  previous min_range = 0.30 m and Gaussian noise (σ = 0.008 m), some returns
  landed at ~0.29 m (2σ below mean) and passed the filter.  These hit the
  LF (+25°→+70°) and RF (−70°→−25°) sectors because motors are at ±45° body
  diagonal.  The result: LF = RF ≈ 0.29 m on every tick, even in open space.

  Fix: min_range raised to 0.45 m in lidar_processing.launch and
  uav_lidar_system.launch.  Motor max range (0.322 m with 2σ noise) and rotor
  tip range (0.433 m) are both below 0.45 m → filtered.  Real wall returns at
  ≥ 0.45 m pass; hard_stop fires at 0.55 m → 0.10 m margin still present.

  BUG 2 — RC3 backward cascade from false LF/RF readings (REMOVED)
  -----------------------------------------------------------------
  With LF = RF ≈ 0.29 m (self-detected):
    • RC2 soft-stop: fn_eff = 0.29 / 0.6 = 0.483 m < hard_stop = 0.55 m
      → M-line completely zeroed (scale = 0.0)
    • RC3 symmetric corridor: bx_rc3 = −1.197 m/s backward body force
  Net: zero forward + ~1.2 m/s backward → drone moved AWAY from goal.
  RC3 is removed entirely (below).

  BUG 3 — FRONT backward repulsion fight (REDESIGNED)
  ----------------------------------------------------
  The previous `bx -= g*s` for F < safety_dist pushed the drone backward while
  the M-line attractor pushed it forward → bang-bang oscillation at walls.
  Replaced with lateral front-steering: when the front sector is blocked, the
  drone steers toward whichever diagonal (LF vs RF) or side (L vs R) has more
  clearance.  The RC2 soft-stop still reduces M-line attraction as the wall
  approaches, so forward speed tapers gracefully without any backward impulse.

Previously fixed bugs retained:
  • VLP-16 plugin blindspot (min_range 0.9 → 0.2 m in URDF)
  • Sector obstacle memory (CLEAR_RATE = 0.5 m/s) — prevents sudden all-clear
  RC1 — EMA alpha 0.65: faster avoidance response (40 ms)
  RC2 — Wide-cone soft-stop: ±70° effective forward cone
  RC4 — Dual-criterion stuck detection: speed + position-spread
  RC5 — 3-phase LiDAR-active recovery: back-off → lateral → cooldown
  RC6 — Proximity-aware M-line speed scaling (min 30%)

Sector layout (updated):
  FRONT     ±25°:    FRONT STEERING — steer toward clearer diagonal side
  LF  +25→+70°:      by -= g*s      LATERAL only (pushes right)
  RF  −25→−70°:      by += g*s      LATERAL only (pushes left)
  L   +70→+130°:     by -= g*s*0.6  LATERAL only
  R  −130→−70°:      by += g*s*0.6  LATERAL only

Navigation layers:
  Layer 1 (50 Hz): M-line attraction + LiDAR avoidance + boundary repulsion
  Layer 2 (0.2 Hz): A* on /map (background thread) — visualization + recovery ref

Default mission: start (−5, −5, 1.5) → goal (5, 5, 1.5)
  14.14 m diagonal crossing rack_C2, boxes, interior walls, rack_B1/B2,
  and pillar_ne (directly in path) — intentional obstacle-field navigation test.
"""

import rospy
import math
import threading
import heapq
import numpy as np

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_to_quat(yaw_rad):
    h = yaw_rad * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))


def world_to_cell(x, y, origin_x, origin_y, resolution):
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row, col


def cell_to_world(row, col, origin_x, origin_y, resolution):
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y


# ---------------------------------------------------------------------------
# A* (retained for visualization + recovery reference)
# ---------------------------------------------------------------------------

def astar(grid, start, goal, max_cost=1e9):
    rows, cols = grid.shape
    sr, sc = start
    gr, gc = goal
    if not (0 <= sr < rows and 0 <= sc < cols): return None
    if not (0 <= gr < rows and 0 <= gc < cols): return None
    if grid[gr, gc]: return None

    def h(r, c):
        return math.sqrt((r - gr) ** 2 + (c - gc) ** 2)

    open_set = [(h(sr, sc), 0.0, sr, sc)]
    g_score = {(sr, sc): 0.0}
    came_from = {}
    DIRS = [(-1,0,1.),( 1,0,1.),(0,-1,1.),(0,1,1.),
            (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]

    while open_set:
        f, g, r, c = heapq.heappop(open_set)
        if (r, c) == (gr, gc):
            path = []
            cur = (gr, gc)
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append((sr, sc))
            path.reverse()
            return path
        if g > g_score.get((r, c), max_cost) + 1e-6:
            continue
        for dr, dc, cost in DIRS:
            nr, nc = r+dr, c+dc
            if not (0 <= nr < rows and 0 <= nc < cols): continue
            if grid[nr, nc]: continue
            ng = g + cost
            if ng < g_score.get((nr, nc), max_cost):
                g_score[(nr, nc)] = ng
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(open_set, (ng + h(nr, nc), ng, nr, nc))
    return None


def smooth_path(cells, origin_x, origin_y, resolution, step_m=0.8):
    if not cells:
        return []
    waypoints = []
    accum = 0.0
    px, py = cell_to_world(cells[0][0], cells[0][1], origin_x, origin_y, resolution)
    waypoints.append((px, py))
    for r, c in cells[1:]:
        nx, ny = cell_to_world(r, c, origin_x, origin_y, resolution)
        accum += math.sqrt((nx-px)**2 + (ny-py)**2)
        if accum >= step_m:
            waypoints.append((nx, ny))
            accum = 0.0
        px, py = nx, ny
    return waypoints


# ---------------------------------------------------------------------------
# Main navigator node
# ---------------------------------------------------------------------------

class UAVNavigator:

    # Software boundary (warehouse walls at +/-8.1 m; keep drone within +/-6.0 m)
    BOUND        = 6.0    # m -- absolute hard clamp, cannot be overridden
    WALL_ZONE    = 2.5    # m -- virtual repulsion activation zone
    WALL_GAIN    = 6.0    # m/s -- peak virtual-wall repulsion speed

    # Obstacle memory clearance rate (m/s).
    # Once a sector registers an obstacle at distance D, it can only clear
    # (increase) at this rate, regardless of what the scan reports.
    # This absorbs the 100 ms scan latency and any sensor blindspot residue.
    CLEAR_RATE   = 0.5    # m/s (= 0.01 m/tick at 50 Hz)

    def __init__(self):
        rospy.init_node("uav_navigator", anonymous=False)
        rospy.loginfo("[nav] UAV Navigator (Motor-Self-Detection-Fixed, RC3-Free) initialising...")

        # ---- Parameters ----
        self.model_name     = rospy.get_param("~model_name",       "uav_lidar")
        self.ref_frame      = rospy.get_param("~reference_frame",  "world")
        self.update_rate    = rospy.get_param("~update_rate",      50.0)
        self.start_delay    = rospy.get_param("~start_delay",       5.0)

        self.start_x        = rospy.get_param("~start_x",  -5.0)
        self.start_y        = rospy.get_param("~start_y",  -5.0)
        self.start_z        = rospy.get_param("~start_z",   1.5)
        self.goal_x         = rospy.get_param("~goal_x",    5.0)
        self.goal_y         = rospy.get_param("~goal_y",    5.0)
        self.goal_z         = rospy.get_param("~goal_z",    1.5)

        self.cruise_speed   = rospy.get_param("~cruise_speed",        1.8)
        self.max_speed      = rospy.get_param("~max_speed",           3.0)
        self.safety_dist    = rospy.get_param("~safety_distance",     1.0)
        self.avoid_gain     = rospy.get_param("~avoidance_gain",      2.5)
        self.hard_stop      = rospy.get_param("~hard_stop_distance",  0.55)
        self.goal_tol       = rospy.get_param("~goal_tolerance",      0.4)
        self.replan_rate    = rospy.get_param("~replan_rate",         0.2)
        self.mline_gain     = rospy.get_param("~mline_gain",          1.0)
        self.smooth_alpha   = rospy.get_param("~smooth_alpha",        0.65)

        self._dt  = 1.0 / self.update_rate
        self.rate = rospy.Rate(self.update_rate)

        # Pre-compute M-line geometry
        mdx = self.goal_x - self.start_x
        mdy = self.goal_y - self.start_y
        mlen = math.sqrt(mdx**2 + mdy**2) or 1e-6
        self._mux = mdx / mlen           # unit vector along M-line
        self._muy = mdy / mlen
        self._mlx = -self._muy           # perpendicular (left of M-line)
        self._mly =  self._mux

        # ---- State ----
        self.cx   = self.start_x
        self.cy   = self.start_y
        self.cz   = self.start_z
        self.cyaw = 0.0

        self._scan_lock   = threading.Lock()
        self._latest_scan = None
        self._map_lock    = threading.Lock()
        self._latest_map  = None
        self._path_lock   = threading.Lock()
        self._astar_wps   = []

        # EMA state for avoidance velocity
        self._vxv_smooth  = 0.0
        self._vyv_smooth  = 0.0

        # RC4: dual-criterion stuck detection
        self._speed_hist   = []
        self._pos_hist     = []

        # RC5: 3-phase recovery state machine
        self._recovery_phase    = 0
        self._recovery_timer    = 0.0
        self._recovery_dir_x    = 0.0
        self._recovery_dir_y    = 0.0
        self._recovery_lat_x    = 0.0
        self._recovery_lat_y    = 0.0

        self.RECOV_BACKOFF_T    = 1.0
        self.RECOV_LATERAL_T    = 2.0
        self.RECOV_COOLDOWN_T   = 1.5
        self.RECOV_BACKOFF_SPD  = 0.8
        self.RECOV_LATERAL_SPD  = 1.0

        # Sensor obstacle memory: per-sector minimum that only clears gradually.
        # Keys: 'F', 'LF', 'RF', 'L', 'R'
        self._sector_mem = {k: float('inf')
                            for k in ('F', 'LF', 'RF', 'L', 'R')}

        # ---- Publishers ----
        self.pose_pub  = rospy.Publisher("/uav/pose",      PoseStamped, queue_size=5)
        self.path_pub  = rospy.Publisher("/uav/path",      Path,        queue_size=2)
        self.debug_pub = rospy.Publisher("/uav/nav_debug", String,      queue_size=5)

        # ---- Subscribers ----
        rospy.Subscriber("/scan", LaserScan,     self._scan_cb, queue_size=1)
        rospy.Subscriber("/map",  OccupancyGrid, self._map_cb,  queue_size=1)

        # ---- Service ----
        rospy.loginfo("[nav] Waiting for /gazebo/set_model_state ...")
        rospy.wait_for_service("/gazebo/set_model_state", timeout=60.0)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.loginfo(
            "[nav] start=(%.1f,%.1f,%.1f) goal=(%.1f,%.1f,%.1f) "
            "safety=%.1fm hard_stop=%.2fm cruise=%.1fm/s alpha=%.2f gain=%.1f",
            self.start_x, self.start_y, self.start_z,
            self.goal_x,  self.goal_y,  self.goal_z,
            self.safety_dist, self.hard_stop, self.cruise_speed,
            self.smooth_alpha, self.avoid_gain)

        rospy.sleep(self.start_delay)
        rospy.loginfo("[nav] Mission starting. Front-steering navigation active (RC3 removed).")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _scan_cb(self, msg):
        with self._scan_lock:
            self._latest_scan = msg

    def _map_cb(self, msg):
        with self._map_lock:
            self._latest_map = msg

    # -----------------------------------------------------------------------
    # Scan sector utility (raw read, no memory)
    # -----------------------------------------------------------------------

    def _sector_min_raw(self, scan, a_lo_deg, a_hi_deg):
        """Return the minimum range in the given angular sector (raw scan, no memory)."""
        a0 = math.radians(min(a_lo_deg, a_hi_deg))
        a1 = math.radians(max(a_lo_deg, a_hi_deg))
        n  = len(scan.ranges)
        b0 = clamp(int((a0 - scan.angle_min) / scan.angle_increment), 0, n-1)
        b1 = clamp(int((a1 - scan.angle_min) / scan.angle_increment), 0, n-1)
        vals = [r for r in scan.ranges[b0:b1+1]
                if math.isfinite(r) and scan.range_min < r < scan.range_max]
        return min(vals) if vals else float("inf")

    # -----------------------------------------------------------------------
    # Sector obstacle memory
    # -----------------------------------------------------------------------

    def _get_sectors(self, scan):
        """
        Read all 5 sectors from 'scan' and apply obstacle memory.

        Memory rule: a sector distance can only INCREASE (clear) at CLEAR_RATE
        m/s.  If the scan suddenly shows a much larger distance (wall vanished
        below sensor min_range, noise, or stale scan), the remembered value
        decreases slowly rather than jumping to "all clear" instantly.

        This is the main defence against the VLP-16 blindspot residue and the
        100 ms scan latency when approaching at cruise speed.

        Returns dict with keys 'F','LF','RF','L','R' — memory-stabilised distances.
        Also returns 'nearest' = min of all five.
        """
        raw = {
            'F' : self._sector_min_raw(scan,  -25,  +25),
            'LF': self._sector_min_raw(scan,  +25,  +70),
            'RF': self._sector_min_raw(scan,  -70,  -25),
            'L' : self._sector_min_raw(scan,  +70, +130),
            'R' : self._sector_min_raw(scan, -130,  -70),
        }
        result = {}
        max_increase = self.CLEAR_RATE * self._dt  # m per tick
        for k, v in raw.items():
            prev = self._sector_mem[k]
            if v <= prev:
                # Obstacle is the same distance or closer: accept immediately
                new_mem = v
            else:
                # Obstacle appears further away: only allow gradual clearance
                new_mem = min(v, prev + max_increase)
            self._sector_mem[k] = new_mem
            result[k] = new_mem
        result['nearest'] = min(result[k] for k in ('F','LF','RF','L','R'))
        return result

    def _reset_sector_mem(self):
        """Clear obstacle memory (called at start of recovery so stale walls
        don't confuse the escape direction after the drone has moved away)."""
        for k in self._sector_mem:
            self._sector_mem[k] = float('inf')

    # -----------------------------------------------------------------------
    # RC6: M-line attraction (proximity-aware speed scaling)
    # -----------------------------------------------------------------------

    def _mline_velocity(self, nearest_obs=float('inf')):
        """
        Compute world-frame attraction velocity along the M-line (start→goal).

        RC6: scales cruise_speed down when obstacles are nearby
        (minimum 30% to always maintain goal-direction bias).
        """
        dist_goal = math.sqrt((self.goal_x - self.cx)**2 +
                               (self.goal_y - self.cy)**2)
        if dist_goal < 1e-3:
            return 0.0, 0.0, 0.0

        goal_scale = min(1.0, dist_goal * 2.0)   # slow near goal

        # RC6: proximity scale
        obs_scale = 1.0
        if nearest_obs < self.safety_dist:
            ramp = self.safety_dist - self.hard_stop
            if ramp > 1e-6:
                obs_scale = max(0.3, (nearest_obs - self.hard_stop) / ramp)
            else:
                obs_scale = 0.3

        effective_speed = self.cruise_speed * goal_scale * obs_scale

        vx_along = self._mux * effective_speed
        vy_along = self._muy * effective_speed

        # Lateral correction: pull back toward M-line
        ex = self.cx - self.start_x
        ey = self.cy - self.start_y
        proj_lat = ex * self._mlx + ey * self._mly
        max_lat = effective_speed * 0.6
        lat_speed = clamp(-proj_lat * self.mline_gain, -max_lat, max_lat)
        vx_lat = self._mlx * lat_speed
        vy_lat = self._mly * lat_speed

        return vx_along + vx_lat, vy_along + vy_lat, dist_goal

    # -----------------------------------------------------------------------
    # LiDAR avoidance — front-steering edition (RC3 removed)
    # -----------------------------------------------------------------------

    def _scan_avoidance(self, yaw, sectors):
        """
        Compute avoidance velocity in body frame, then rotate to world frame.

        Accepts pre-computed memory-stabilised 'sectors' dict from _get_sectors().
        Returns (world_vx, world_vy, debug_string).

        Sector design (motor-self-detection-fixed, RC3-free):
          FRONT     ±25°:    FRONT STEERING — proportional lateral push toward
                             whichever diagonal (LF vs RF) has more clearance.
                             Falls back to wider side sectors (L vs R) when the
                             diagonals are symmetric.
                             NO backward force — soft-stop (RC2) handles decel.
          LF  +25→+70°:      by -= g*s*1.0   LATERAL only (pushes right)
          RF  −25→−70°:      by += g*s*1.0   LATERAL only (pushes left)
          L   +70→+130°:     by -= g*s*0.6   LATERAL only
          R  −130→−70°:      by += g*s*0.6   LATERAL only

        Why no FRONT backward force:
          The previous `bx -= g*s` pushed the drone away from the goal whenever
          a wall entered the ±25° cone.  Combined with RC3 (also backward), this
          dominated the M-line attractor and drove the drone AWAY from its goal.
          RC2 soft-stop already ramps the M-line to zero at hard_stop distance;
          an additional backward impulse is redundant and harmful.

        Why RC3 is removed:
          RC3 fired whenever LF ≈ RF (symmetric), which happened continuously
          due to motor-housing self-detection (LF = RF ≈ 0.29 m at min_range
          0.30 m).  The min_range fix (0.30 → 0.45 m) eliminates self-detection,
          but even with real symmetric obstacles RC3 produced a backwards force
          that overwhelmed the goal attractor.
        """
        if sectors is None:
            return 0.0, 0.0, "no_scan"

        F  = sectors['F']
        LF = sectors['LF']
        RF = sectors['RF']
        L  = sectors['L']
        R  = sectors['R']

        d = self.safety_dist
        g = self.avoid_gain
        bx = 0.0
        by = 0.0

        # FRONT sector: steer laterally toward the more open diagonal/side.
        # Magnitude proportional to how blocked the front is; no backward push.
        if F < d:
            s = ((d - F) / d) ** 1.5
            if LF > RF + 0.15:
                # Left diagonal clearly more open → steer left (by > 0 in FLU)
                by += g * s * 0.8
            elif RF > LF + 0.15:
                # Right diagonal clearly more open → steer right (by < 0 in FLU)
                by -= g * s * 0.8
            else:
                # Near-symmetric front blockage: tiebreak using wider side sector
                if L >= R:
                    by += g * s * 0.4   # left side more open → nudge left
                else:
                    by -= g * s * 0.4   # right side more open → nudge right

        # LF / RF: purely lateral (no bx change — oscillation prevention)
        if LF < d:
            s = ((d - LF) / d) ** 1.5
            by -= g * s           # LF blocked → push right

        if RF < d:
            s = ((d - RF) / d) ** 1.5
            by += g * s           # RF blocked → push left

        # Side sectors (lower gain — wide coverage, not primary avoidance)
        if L < d:
            s = ((d - L) / d) ** 1.5
            by -= g * s * 0.6     # L blocked → push right

        if R < d:
            s = ((d - R) / d) ** 1.5
            by += g * s * 0.6     # R blocked → push left

        # Body → world frame rotation (FLU body frame, standard 2-D rotation)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        wx = bx * cos_y - by * sin_y
        wy = bx * sin_y + by * cos_y

        dbg = (f"F={F:.2f} LF={LF:.2f} RF={RF:.2f} L={L:.2f} R={R:.2f}"
               f" body=({bx:+.2f},{by:+.2f}) world=({wx:+.2f},{wy:+.2f})")
        return wx, wy, dbg

    # -----------------------------------------------------------------------
    # RC2: Wide-cone proportional soft-stop
    # -----------------------------------------------------------------------

    def _apply_soft_stop(self, vxa, vya, sectors):
        """
        RC2: weighted 3-sector effective distance catches angled-wall approaches.

        Accepts pre-computed memory-stabilised 'sectors' dict from _get_sectors().
        SIDE_WEIGHT=0.6: a side obstacle at d triggers the same reduction as a
        front obstacle at d/0.6, preventing over-braking in wide corridors.

        With the VLP-16 blindspot fix, fn/lf_d/rf_d are now reliable down to
        0.2 m, so the ramp correctly reduces vxa to 0 at hard_stop=0.55 m.
        """
        if sectors is None:
            return vxa, vya

        fn   = sectors['F']
        lf_d = sectors['LF']
        rf_d = sectors['RF']

        SIDE_WEIGHT = 0.6
        fn_eff = fn
        if lf_d < self.safety_dist:
            fn_eff = min(fn_eff, lf_d / SIDE_WEIGHT)
        if rf_d < self.safety_dist:
            fn_eff = min(fn_eff, rf_d / SIDE_WEIGHT)

        if fn_eff < self.safety_dist:
            ramp = self.safety_dist - self.hard_stop
            if ramp < 1e-6:
                scale = 0.0
            else:
                scale = max(0.0, (fn_eff - self.hard_stop) / ramp)
            vxa *= scale
            vya *= scale
            if fn_eff < self.hard_stop:
                rospy.logwarn_throttle(1.0,
                    "[nav] HARD STOP fn=%.2f lf=%.2f rf=%.2f eff=%.2f scale=0",
                    fn, lf_d, rf_d, fn_eff)

        return vxa, vya

    # -----------------------------------------------------------------------
    # Virtual boundary repulsion (unchanged)
    # -----------------------------------------------------------------------

    def _boundary_repulsion(self):
        B  = self.BOUND
        Z  = self.WALL_ZONE
        G  = self.WALL_GAIN
        vx = 0.0
        vy = 0.0

        d = self.cx - (-B)
        if d < Z: vx += G * ((Z - d) / Z) ** 2

        d = B - self.cx
        if d < Z: vx -= G * ((Z - d) / Z) ** 2

        d = self.cy - (-B)
        if d < Z: vy += G * ((Z - d) / Z) ** 2

        d = B - self.cy
        if d < Z: vy -= G * ((Z - d) / Z) ** 2

        return vx, vy

    # -----------------------------------------------------------------------
    # Hard position clamp — absolute, always applied last
    # -----------------------------------------------------------------------

    def _clamp_position(self):
        B = self.BOUND
        self.cx  = clamp(self.cx,  -B,   B)
        self.cy  = clamp(self.cy,  -B,   B)
        self.cz  = clamp(self.cz,   0.4, 3.5)

    # -----------------------------------------------------------------------
    # A* replanning thread (background, visualization + recovery reference)
    # -----------------------------------------------------------------------

    def _replan_loop(self):
        rate = rospy.Rate(self.replan_rate)
        while not rospy.is_shutdown():
            self._run_astar()
            rate.sleep()

    def _run_astar(self):
        with self._map_lock:
            occ_map = self._latest_map
        if occ_map is None:
            return

        res  = occ_map.info.resolution
        w    = occ_map.info.width
        h    = occ_map.info.height
        ox   = occ_map.info.origin.position.x
        oy   = occ_map.info.origin.position.y
        data = np.array(occ_map.data, dtype=np.int8).reshape((h, w))
        occ  = data > 50

        inflate_cells = max(1, int(0.5 / res))
        from scipy.ndimage import binary_dilation
        struct = np.ones((2*inflate_cells+1, 2*inflate_cells+1), dtype=bool)
        try:
            inflated = binary_dilation(occ, structure=struct)
        except Exception:
            inflated = occ

        sr, sc = world_to_cell(self.cx,     self.cy,     ox, oy, res)
        gr, gc = world_to_cell(self.goal_x, self.goal_y, ox, oy, res)

        path_cells = astar(inflated, (sr, sc), (gr, gc))

        with self._path_lock:
            if path_cells and len(path_cells) > 1:
                self._astar_wps = smooth_path(path_cells, ox, oy, res, step_m=0.8)
            else:
                self._astar_wps = []

        self._publish_astar_path(path_cells, ox, oy, res)

    def _publish_astar_path(self, cells, ox, oy, res):
        if not cells:
            return
        path_msg = Path()
        path_msg.header = Header(frame_id="map", stamp=rospy.Time.now())
        for r, c in cells[::3]:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x, ps.pose.position.y = cell_to_world(r, c, ox, oy, res)
            ps.pose.position.z = self.goal_z
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    # -----------------------------------------------------------------------
    # RC4: Dual-criterion stuck detection
    # -----------------------------------------------------------------------

    def _update_stuck(self, speed, cx, cy):
        """
        RC4: dual criterion.
        A: avg speed < 0.06 m/s over 2 s — genuine no-motion stuck.
        B: position bounding-box diagonal < 0.25 m over 2 s — catches oscillation.
        """
        WINDOW_S      = 2.0
        SPEED_THRESH  = 0.06
        SPREAD_THRESH = 0.25
        window_ticks  = int(WINDOW_S * self.update_rate)

        self._speed_hist.append(speed)
        self._pos_hist.append((cx, cy))

        if len(self._speed_hist) > window_ticks:
            self._speed_hist.pop(0)
            self._pos_hist.pop(0)

        if len(self._speed_hist) < window_ticks:
            return False

        stuck_A = (sum(self._speed_hist) / len(self._speed_hist)) < SPEED_THRESH

        xs = [p[0] for p in self._pos_hist]
        ys = [p[1] for p in self._pos_hist]
        spread_sq = (max(xs) - min(xs))**2 + (max(ys) - min(ys))**2
        stuck_B = spread_sq < SPREAD_THRESH**2

        return stuck_A or stuck_B

    # -----------------------------------------------------------------------
    # RC5: Recovery — compute directions, enter Phase 1
    # -----------------------------------------------------------------------

    def _start_recovery(self, sectors):
        """
        RC5: Compute back-off and lateral directions from current sectors,
        then enter Phase 1 (back-off for RECOV_BACKOFF_T seconds).

        'sectors' is the current memory-stabilised sector dict (may be None).
        """
        # Back-off direction from current avoidance output
        wx_raw, wy_raw, _ = self._scan_avoidance(self.cyaw, sectors)
        mag = math.sqrt(wx_raw**2 + wy_raw**2)
        if mag > 0.1:
            self._recovery_dir_x = wx_raw / mag
            self._recovery_dir_y = wy_raw / mag
        else:
            gd = math.sqrt((self.goal_x - self.cx)**2 +
                           (self.goal_y - self.cy)**2) or 1.0
            self._recovery_dir_x = -(self.goal_x - self.cx) / gd
            self._recovery_dir_y = -(self.goal_y - self.cy) / gd

        # Lateral direction: choose side with most clearance
        if sectors is not None:
            L = sectors['L']
            R = sectors['R']
            sign = 1.0 if L >= R else -1.0
            cos_y = math.cos(self.cyaw)
            sin_y = math.sin(self.cyaw)
            self._recovery_lat_x = -sign * sin_y
            self._recovery_lat_y =  sign * cos_y
        else:
            gd = math.sqrt((self.goal_x - self.cx)**2 +
                           (self.goal_y - self.cy)**2) or 1.0
            self._recovery_lat_x = -(self.goal_y - self.cy) / gd
            self._recovery_lat_y =  (self.goal_x - self.cx) / gd

        self._recovery_phase = 1
        self._recovery_timer = self.RECOV_BACKOFF_T

        # Clear histories so stuck detector starts fresh after recovery
        self._speed_hist.clear()
        self._pos_hist.clear()
        self._vxv_smooth = 0.0
        self._vyv_smooth = 0.0
        # Clear obstacle memory so stale proximity doesn't block escape
        self._reset_sector_mem()

        rospy.logwarn("[nav] STUCK -> Phase 1 back-off dir=(%.2f,%.2f)",
                      self._recovery_dir_x, self._recovery_dir_y)

    # -----------------------------------------------------------------------
    # Pose publish + SetModelState
    # -----------------------------------------------------------------------

    def _publish_pose(self):
        ps = PoseStamped()
        ps.header = Header(frame_id="map", stamp=rospy.Time.now())
        ps.pose.position.x    = self.cx
        ps.pose.position.y    = self.cy
        ps.pose.position.z    = self.cz
        ps.pose.orientation   = yaw_to_quat(self.cyaw)
        self.pose_pub.publish(ps)

    def _set_model_state(self):
        s = ModelState()
        s.model_name       = self.model_name
        s.reference_frame  = self.ref_frame
        s.pose.position.x  = self.cx
        s.pose.position.y  = self.cy
        s.pose.position.z  = self.cz
        s.pose.orientation = yaw_to_quat(self.cyaw)
        try:
            self.set_state(s)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, "[nav] SetModelState: %s", e)

    # -----------------------------------------------------------------------
    # Main control loop
    # -----------------------------------------------------------------------

    def run(self):
        t = threading.Thread(target=self._replan_loop, daemon=True)
        t.start()

        log_t = 0.0
        alpha = self.smooth_alpha   # RC1: 0.65
        scan_dbg = "no_scan"

        while not rospy.is_shutdown():

            # --- Check goal reached ---
            dist_goal = math.sqrt((self.goal_x - self.cx)**2 +
                                   (self.goal_y - self.cy)**2)
            if dist_goal < self.goal_tol:
                rospy.loginfo("[nav] GOAL REACHED! dist=%.2fm", dist_goal)
                self._set_model_state()
                break

            # --- Read sectors ONCE per tick (with obstacle memory applied) ---
            # This is the single authoritative source of LiDAR data for this tick.
            # Both _scan_avoidance and _apply_soft_stop receive these values so
            # they never re-lock or re-compute the scan independently.
            with self._scan_lock:
                scan_snap = self._latest_scan
            sectors = None
            nearest_obs = float('inf')
            if scan_snap is not None:
                sectors = self._get_sectors(scan_snap)
                nearest_obs = sectors['nearest']
            else:
                rospy.logwarn_throttle(5.0, "[nav] No scan received yet")

            # --- Yaw: face goal direction ---
            goal_angle = math.atan2(self.goal_y - self.cy, self.goal_x - self.cx)
            yerr = goal_angle - self.cyaw
            while yerr >  math.pi: yerr -= 2 * math.pi
            while yerr < -math.pi: yerr += 2 * math.pi
            vyaw = clamp(yerr * 2.5, -2.0, 2.0)

            # --- Vertical control ---
            vz = clamp((self.goal_z - self.cz) * 2.0, -1.0, 1.0)

            # ==============================================================
            # RC5: 3-PHASE RECOVERY STATE MACHINE
            # ==============================================================
            if self._recovery_phase > 0:
                self._recovery_timer -= self._dt

                if self._recovery_phase == 1:
                    # Phase 1: Back-off (LiDAR ACTIVE)
                    vxa = self._recovery_dir_x * self.RECOV_BACKOFF_SPD
                    vya = self._recovery_dir_y * self.RECOV_BACKOFF_SPD
                    vxv_raw, vyv_raw, _ = self._scan_avoidance(self.cyaw, sectors)
                    vxv = alpha * vxv_raw + (1.0 - alpha) * self._vxv_smooth
                    vyv = alpha * vyv_raw + (1.0 - alpha) * self._vyv_smooth
                    self._vxv_smooth, self._vyv_smooth = vxv, vyv
                    vxb, vyb = self._boundary_repulsion()
                    vx = clamp(vxa + vxv + vxb, -self.max_speed, self.max_speed)
                    vy = clamp(vya + vyv + vyb, -self.max_speed, self.max_speed)
                    if self._recovery_timer <= 0.0:
                        self._recovery_phase = 2
                        self._recovery_timer = self.RECOV_LATERAL_T
                        rospy.loginfo("[nav] Phase 2 lateral dir=(%.2f,%.2f)",
                                      self._recovery_lat_x, self._recovery_lat_y)

                elif self._recovery_phase == 2:
                    # Phase 2: Lateral escape + 30% goal pull (LiDAR ACTIVE)
                    gd = math.sqrt((self.goal_x - self.cx)**2 +
                                   (self.goal_y - self.cy)**2) or 1.0
                    gux = (self.goal_x - self.cx) / gd
                    guy = (self.goal_y - self.cy) / gd
                    vxa = (self._recovery_lat_x * self.RECOV_LATERAL_SPD +
                           gux * self.cruise_speed * 0.3)
                    vya = (self._recovery_lat_y * self.RECOV_LATERAL_SPD +
                           guy * self.cruise_speed * 0.3)
                    vxv_raw, vyv_raw, _ = self._scan_avoidance(self.cyaw, sectors)
                    vxv = alpha * vxv_raw + (1.0 - alpha) * self._vxv_smooth
                    vyv = alpha * vyv_raw + (1.0 - alpha) * self._vyv_smooth
                    self._vxv_smooth, self._vyv_smooth = vxv, vyv
                    vxb, vyb = self._boundary_repulsion()
                    vx = clamp(vxa + vxv + vxb, -self.max_speed, self.max_speed)
                    vy = clamp(vya + vyv + vyb, -self.max_speed, self.max_speed)
                    if self._recovery_timer <= 0.0:
                        self._recovery_phase = 3
                        self._recovery_timer = self.RECOV_COOLDOWN_T
                        rospy.loginfo("[nav] Phase 3 cooldown, resuming M-line")

                elif self._recovery_phase == 3:
                    # Phase 3: Normal M-line, stuck detector suppressed
                    vxa, vya, _ = self._mline_velocity(nearest_obs)
                    vxa, vya = self._apply_soft_stop(vxa, vya, sectors)
                    vxv_raw, vyv_raw, _ = self._scan_avoidance(self.cyaw, sectors)
                    vxv = alpha * vxv_raw + (1.0 - alpha) * self._vxv_smooth
                    vyv = alpha * vyv_raw + (1.0 - alpha) * self._vyv_smooth
                    self._vxv_smooth, self._vyv_smooth = vxv, vyv
                    vxb, vyb = self._boundary_repulsion()
                    vx = clamp(vxa + vxv + vxb, -self.max_speed, self.max_speed)
                    vy = clamp(vya + vyv + vyb, -self.max_speed, self.max_speed)
                    if self._recovery_timer <= 0.0:
                        self._recovery_phase = 0
                        rospy.loginfo("[nav] Recovery complete, M-line resumed.")
                else:
                    vx = vy = 0.0

                sp = math.sqrt(vx**2 + vy**2)
                self.cx   += vx   * self._dt
                self.cy   += vy   * self._dt
                self.cz   += vz   * self._dt
                self.cyaw += vyaw * self._dt
                self._clamp_position()
                self._set_model_state()
                self._publish_pose()

                log_t += self._dt
                if log_t >= 0.5:
                    log_t = 0.0
                    rospy.loginfo("[nav] RECOVERY phase=%d timer=%.1fs spd=%.2f "
                                  "pos=(%.2f,%.2f)",
                                  self._recovery_phase, self._recovery_timer,
                                  sp, self.cx, self.cy)

                self.rate.sleep()
                continue

            # ==============================================================
            # LAYER 1: 50 Hz — M-line attraction + LiDAR avoidance
            # ==============================================================

            # RC6: M-line with proximity scaling
            vxa, vya, _ = self._mline_velocity(nearest_obs)

            # RC2: Wide-cone soft-stop (uses pre-computed sectors)
            vxa, vya = self._apply_soft_stop(vxa, vya, sectors)

            # LiDAR avoidance with front steering (uses pre-computed sectors)
            vxv_raw, vyv_raw, scan_dbg = self._scan_avoidance(self.cyaw, sectors)

            # RC1: EMA smoothing on avoidance (alpha=0.65)
            vxv = alpha * vxv_raw + (1.0 - alpha) * self._vxv_smooth
            vyv = alpha * vyv_raw + (1.0 - alpha) * self._vyv_smooth
            self._vxv_smooth = vxv
            self._vyv_smooth = vyv

            # Virtual boundary repulsion
            vxb, vyb = self._boundary_repulsion()

            # Sum and clamp
            vx = vxa + vxv + vxb
            vy = vya + vyv + vyb
            sp = math.sqrt(vx**2 + vy**2)
            if sp > self.max_speed:
                vx *= self.max_speed / sp
                vy *= self.max_speed / sp

            # Integrate
            self.cx   += vx   * self._dt
            self.cy   += vy   * self._dt
            self.cz   += vz   * self._dt
            self.cyaw += vyaw * self._dt

            # Hard clamp — cannot clip through warehouse walls
            self._clamp_position()

            self._set_model_state()
            self._publish_pose()

            # RC4: Dual-criterion stuck detection
            if self._recovery_phase == 0:
                if self._update_stuck(sp, self.cx, self.cy):
                    rospy.logwarn("[nav] STUCK detected (spd=%.2f) -> recovery", sp)
                    self._start_recovery(sectors)

            # Debug log at 2 Hz
            log_t += self._dt
            if log_t >= 0.5:
                log_t = 0.0
                ex = self.cx - self.start_x
                ey = self.cy - self.start_y
                lat_dev = ex * self._mlx + ey * self._mly
                nobs_str = f"{nearest_obs:.2f}" if nearest_obs < 99 else "inf"
                dbg = (f"pos=({self.cx:.2f},{self.cy:.2f},{self.cz:.2f}) "
                       f"goal_dist={dist_goal:.2f}m lat_dev={lat_dev:+.2f}m "
                       f"spd={sp:.2f}m/s nobs={nobs_str}m {scan_dbg}")
                self.debug_pub.publish(String(data=dbg))
                if abs(vxv) > 0.1 or abs(vyv) > 0.1:
                    rospy.loginfo("[nav] %s", dbg)

            self.rate.sleep()

        rospy.loginfo("[nav] Mission complete.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    nav = UAVNavigator()
    try:
        nav.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
