#!/usr/bin/env python3
"""
UAV LiDAR Stack — Obstacle Map Node
=====================================
Replaces hector_mapping (which crashes with exit code -11 / SIGSEGV).

Builds a 2D occupancy grid from LaserScan data using:
  - Log-odds probabilistic updates
  - Bresenham ray-casting (marks cells as free along ray, occupied at endpoint)
  - Drone pose from /ground_truth/odom (no TF dependency)

Publishes:
  /map            (nav_msgs/OccupancyGrid, 2 Hz)  — compatible with RViz + A*
  /uav/map_debug  (std_msgs/String, 0.5 Hz)        — diagnostics

This node is 100% Python, crash-proof, and handles NaN/inf gracefully.

Grid parameters:
  size       : 320×320 cells
  resolution : 0.05 m/cell
  coverage   : 16 m × 16 m  (matches warehouse ±8 m from centre)
  frame_id   : "map"
  origin     : (-8.0, -8.0) m  (world frame)
"""

import rospy
import math
import threading
import numpy as np

from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Header


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GRID_SIZE   = 320          # cells (both axes)
RESOLUTION  = 0.05         # m per cell
ORIGIN_X    = -8.0         # m: world x of cell (0,0) left edge
ORIGIN_Y    = -8.0         # m: world y of cell (0,0) bottom edge

# Log-odds update values
LOG_FREE    = math.log(0.35 / 0.65)   # ≈ -0.619
LOG_OCC     = math.log(0.75 / 0.25)   # ≈  1.099
LOG_MIN     = -3.0
LOG_MAX     = +3.0

# Occupancy thresholds for output
THRESH_OCC  = 0.65    # probability → output 100
THRESH_FREE = 0.40    # probability → output 0
# Between thresholds → output -1 (unknown)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def world_to_cell(wx, wy):
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = int((wy - ORIGIN_Y) / RESOLUTION)
    return row, col


def cell_in_bounds(row, col):
    return 0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE


def bresenham(r0, c0, r1, c1):
    """
    Yield (row, col) cells along the line from (r0,c0) to (r1,c1)
    using Bresenham's algorithm (integer arithmetic).
    Does NOT yield the final endpoint.
    """
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1
    err = dr - dc

    r, c = r0, c0
    while True:
        if (r, c) == (r1, c1):
            break
        yield r, c
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r   += sr
        if e2 < dr:
            err += dr
            c   += sc


def log_odds_to_prob(lo):
    e = math.exp(lo)
    return e / (1.0 + e)


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class ObstacleMapNode:

    def __init__(self):
        rospy.init_node("obstacle_map_node", anonymous=False)
        rospy.loginfo("[map] Obstacle Map Node initialising...")

        # Parameters
        self.publish_rate = rospy.get_param("~publish_rate",    2.0)
        self.max_range    = rospy.get_param("~max_range",       12.0)  # use only close returns
        self.scan_inc     = rospy.get_param("~scan_stride",     2)     # use every Nth ray
        self.decay_rate   = rospy.get_param("~decay_rate",      0.0)   # log-odds decay/tick (0=no decay)

        # Occupancy grid in log-odds form
        self._log_odds = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        self._grid_lock = threading.Lock()

        # Drone pose (from /ground_truth/odom)
        self._px   = 0.0
        self._py   = 0.0
        self._yaw  = 0.0
        self._pose_valid = False
        self._pose_lock  = threading.Lock()

        # Stats
        self._scan_count = 0
        self._cell_updates = 0

        # Publishers
        self.map_pub   = rospy.Publisher("/map",           OccupancyGrid, queue_size=2, latch=True)
        self.debug_pub = rospy.Publisher("/uav/map_debug", String,        queue_size=2)

        # Subscribers
        rospy.Subscriber("/ground_truth/odom", Odometry,  self._odom_cb, queue_size=5)
        rospy.Subscriber("/scan",              LaserScan, self._scan_cb, queue_size=1)

        # Publish timer
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_map)

        rospy.loginfo(
            "[map] Grid: %dx%d @ %.2fm, origin=(%.1f,%.1f), max_range=%.1f",
            GRID_SIZE, GRID_SIZE, RESOLUTION, ORIGIN_X, ORIGIN_Y, self.max_range)

    # -----------------------------------------------------------------------
    # Odometry callback
    # -----------------------------------------------------------------------

    def _odom_cb(self, msg):
        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._pose_lock:
            self._px   = px
            self._py   = py
            self._yaw  = yaw
            self._pose_valid = True

    # -----------------------------------------------------------------------
    # Scan callback — main map update
    # -----------------------------------------------------------------------

    def _scan_cb(self, msg):
        with self._pose_lock:
            if not self._pose_valid:
                return
            px   = self._px
            py   = self._py
            yaw  = self._yaw

        # Drone grid cell (sensor origin)
        or_, oc = world_to_cell(px, py)
        if not cell_in_bounds(or_, oc):
            return

        ranges     = msg.ranges
        angle_min  = msg.angle_min
        angle_inc  = msg.angle_increment
        rmin       = msg.range_min
        rmax       = min(msg.range_max, self.max_range)

        with self._grid_lock:
            updates = 0
            stride  = max(1, self.scan_inc)

            for i in range(0, len(ranges), stride):
                r = ranges[i]
                if not math.isfinite(r) or r < rmin or r > rmax:
                    continue

                angle_body = angle_min + i * angle_inc
                angle_world = yaw + angle_body

                # Endpoint in world frame
                ex = px + r * math.cos(angle_world)
                ey = py + r * math.sin(angle_world)
                er, ec = world_to_cell(ex, ey)

                # Ray-cast: mark free cells
                for fr, fc in bresenham(or_, oc, er, ec):
                    if cell_in_bounds(fr, fc):
                        self._log_odds[fr, fc] = max(
                            LOG_MIN, self._log_odds[fr, fc] + LOG_FREE)
                        updates += 1

                # Mark endpoint occupied
                if cell_in_bounds(er, ec):
                    self._log_odds[er, ec] = min(
                        LOG_MAX, self._log_odds[er, ec] + LOG_OCC)
                    updates += 1

            # Optional slow decay toward unknown (LOG_FREE decays to 0)
            if self.decay_rate > 0.0:
                self._log_odds -= self.decay_rate
                np.clip(self._log_odds, LOG_MIN, LOG_MAX, out=self._log_odds)

            self._cell_updates += updates

        self._scan_count += 1

    # -----------------------------------------------------------------------
    # Map publisher
    # -----------------------------------------------------------------------

    def _publish_map(self, event=None):
        now = rospy.Time.now()

        # Convert log-odds to OccupancyGrid values (-1, 0..100)
        with self._grid_lock:
            lo = self._log_odds.copy()

        prob = np.exp(lo) / (1.0 + np.exp(lo))   # vectorised sigmoid

        out = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)
        out[prob > THRESH_OCC]  = 100
        out[prob < THRESH_FREE] = 0

        # OccupancyGrid message (row-major, origin at bottom-left)
        msg = OccupancyGrid()
        msg.header = Header(frame_id="map", stamp=now)
        msg.info   = MapMetaData()
        msg.info.map_load_time  = now
        msg.info.resolution     = RESOLUTION
        msg.info.width          = GRID_SIZE
        msg.info.height         = GRID_SIZE
        origin_pose = Pose()
        origin_pose.position    = Point(x=ORIGIN_X, y=ORIGIN_Y, z=0.0)
        origin_pose.orientation = Quaternion(w=1.0)
        msg.info.origin         = origin_pose
        msg.data                = out.flatten().tolist()

        self.map_pub.publish(msg)

        # Debug stats
        occ_count = int(np.sum(out == 100))
        free_count = int(np.sum(out == 0))
        dbg = (f"scans={self._scan_count} occ={occ_count} "
               f"free={free_count} updates={self._cell_updates}")
        self.debug_pub.publish(String(data=dbg))

    # -----------------------------------------------------------------------
    # Spin
    # -----------------------------------------------------------------------

    def spin(self):
        rospy.spin()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    node = ObstacleMapNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
