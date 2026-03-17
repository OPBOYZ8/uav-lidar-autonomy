#!/usr/bin/env python3
"""
UAV LiDAR Stack - Drone Waypoint Controller WITH Obstacle Avoidance
=====================================================================
ROOT CAUSE FIX
--------------
The original controller used pure lerp -> SetModelState with ZERO
connection to any LiDAR topic.  /scan had no subscribers at all.

This version adds:
  1. rospy.Subscriber('/scan', LaserScan, ...)  <- wires LiDAR to motion
  2. Sector-based obstacle detection from LaserScan
  3. Body-frame repulsion rotated to world frame via current yaw
  4. Velocity-based motion integrator (replaces lerp)
  5. Hard-stop when front obstacle < hard_stop_dist

CONTROL LOOP (50 Hz):
  v_attract = proportional pull toward waypoint   (world frame)
  v_avoid   = repulsion from LaserScan sectors    (body -> world frame)
  v_total   = v_attract + v_avoid  (clamped to max_speed)
  new_pose  = integrate(v_total * dt)  ->  SetModelState
"""

import rospy
import math
import threading

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def yaw_to_quat(yaw_rad):
    h = yaw_rad * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class WaypointController:

    WAYPOINTS = [
        ( 0.0,  0.0, 1.5,   0, "Home"),
        ( 0.0,  0.0, 1.5,  90, "Turn north"),
        (-6.0,  6.5, 1.5,   0, "NW corner"),
        (-6.0,  0.0, 1.5,   0, "W aisle mid"),
        (-6.0, -6.5, 1.5,   0, "SW corner"),
        (-3.0, -6.5, 1.5,  90, "CW south"),
        (-3.0,  0.0, 1.5,  90, "CW mid"),
        (-3.0,  6.5, 1.5,  90, "CW north"),
        ( 0.0,  6.5, 1.5, 180, "N centre"),
        ( 0.0,  0.0, 1.5, 180, "Centre"),
        ( 0.0, -6.5, 1.5, 180, "S centre"),
        ( 3.0, -6.5, 1.5, 270, "CE south"),
        ( 3.0,  0.0, 1.5, 270, "CE mid"),
        ( 3.0,  6.5, 1.5, 270, "CE north"),
        ( 6.0,  6.5, 1.5, 180, "NE corner"),
        ( 6.0,  0.0, 1.5, 180, "E aisle mid"),
        ( 6.0, -6.5, 1.5, 180, "SE corner"),
        ( 0.0,  0.0, 1.5,   0, "Return home"),
    ]

    def __init__(self):
        rospy.init_node("drone_waypoint_controller", anonymous=False)
        rospy.loginfo("[waypoint_ctrl] Init with obstacle avoidance enabled.")

        self.model_name      = rospy.get_param("~model_name",         "uav_lidar")
        self.ref_frame       = rospy.get_param("~reference_frame",    "world")
        self.update_rate     = rospy.get_param("~update_rate",         50.0)
        self.move_duration   = rospy.get_param("~move_duration",        5.0)
        self.dwell_time      = rospy.get_param("~dwell_time",           2.0)
        self.loop_mission    = rospy.get_param("~loop_mission",         True)
        self.start_delay     = rospy.get_param("~start_delay",         10.0)
        self.cruise_speed    = rospy.get_param("~cruise_speed",         1.0)
        self.safety_distance = rospy.get_param("~safety_distance",      2.5)
        self.avoidance_gain  = rospy.get_param("~avoidance_gain",       2.5)
        self.hard_stop_dist  = rospy.get_param("~hard_stop_distance",   0.8)
        self.max_speed       = rospy.get_param("~max_speed",            2.0)

        self.rate = rospy.Rate(self.update_rate)
        self._dt  = 1.0 / self.update_rate
        self._scan_lock   = threading.Lock()
        self._latest_scan = None

        self.status_pub = rospy.Publisher("/uav/waypoint_status", String, queue_size=5)
        self.debug_pub  = rospy.Publisher("/uav/avoidance_debug",  String, queue_size=5)

        # ROOT CAUSE FIX: subscribe to /scan so LiDAR data reaches controller
        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self._scan_cb, queue_size=1)
        rospy.loginfo("[waypoint_ctrl] Subscribed to /scan for obstacle avoidance.")

        rospy.loginfo("[waypoint_ctrl] Waiting for /gazebo/set_model_state ...")
        rospy.wait_for_service("/gazebo/set_model_state", timeout=60.0)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.loginfo(
            "[waypoint_ctrl] safety=%.1fm gain=%.1fm/s hard_stop=%.1fm cruise=%.1fm/s",
            self.safety_distance, self.avoidance_gain,
            self.hard_stop_dist, self.cruise_speed)

        rospy.sleep(self.start_delay)
        rospy.loginfo("[waypoint_ctrl] Mission starting.")

    def _scan_cb(self, msg):
        with self._scan_lock:
            self._latest_scan = msg

    def _sector_min(self, scan, a_lo_deg, a_hi_deg):
        """Minimum finite range in body-frame sector [a_lo_deg, a_hi_deg]."""
        a0 = math.radians(min(a_lo_deg, a_hi_deg))
        a1 = math.radians(max(a_lo_deg, a_hi_deg))
        n  = len(scan.ranges)
        b0 = clamp(int((a0 - scan.angle_min) / scan.angle_increment), 0, n - 1)
        b1 = clamp(int((a1 - scan.angle_min) / scan.angle_increment), 0, n - 1)
        vals = [r for r in scan.ranges[b0:b1 + 1]
                if scan.range_min < r < scan.range_max and math.isfinite(r)]
        return min(vals) if vals else float("inf")

    def _avoidance(self, yaw):
        """
        Compute avoidance velocity in world frame from LaserScan sectors.

        For each sector:
            s = (safety_distance - range) / safety_distance  in [0,1]

        Body-frame repulsion (+X fwd, +Y left):
            front      -> bx -= g*s
            front-left -> bx -= g*s*0.6,  by -= g*s*0.6   (push back-right)
            front-right-> bx -= g*s*0.6,  by += g*s*0.6   (push back-left)
            left       ->                 by -= g*s*0.75   (push right)
            right      ->                 by += g*s*0.75   (push left)

        World frame:
            wx = bx*cos(yaw) - by*sin(yaw)
            wy = bx*sin(yaw) + by*cos(yaw)
        """
        with self._scan_lock:
            scan = self._latest_scan
        if scan is None:
            return 0.0, 0.0, "no_scan"

        F  = self._sector_min(scan,  -30,  +30)
        FL = self._sector_min(scan,  +15,  +75)
        FR = self._sector_min(scan,  -75,  -15)
        L  = self._sector_min(scan,  +75, +135)
        R  = self._sector_min(scan, -135,  -75)

        d = self.safety_distance
        g = self.avoidance_gain
        bx = 0.0
        by = 0.0

        if F  < d: s = (d - F)  / d; bx -= g * s
        if FL < d: s = (d - FL) / d; bx -= g * s * 0.6; by -= g * s * 0.6
        if FR < d: s = (d - FR) / d; bx -= g * s * 0.6; by += g * s * 0.6
        if L  < d: s = (d - L)  / d;                    by -= g * s * 0.75
        if R  < d: s = (d - R)  / d;                    by += g * s * 0.75

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        wx = bx * cos_y - by * sin_y
        wy = bx * sin_y + by * cos_y

        dbg = (f"F={F:.2f} FL={FL:.2f} FR={FR:.2f} L={L:.2f} R={R:.2f}"
               f" | body=({bx:+.2f},{by:+.2f}) world=({wx:+.2f},{wy:+.2f})")
        return wx, wy, dbg

    def _set_pose(self, x, y, z, yaw):
        s = ModelState()
        s.model_name       = self.model_name
        s.reference_frame  = self.ref_frame
        s.pose.position.x  = x
        s.pose.position.y  = y
        s.pose.position.z  = z
        s.pose.orientation = yaw_to_quat(yaw)
        s.twist            = Twist()
        try:
            self.set_state(s)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, "[waypoint_ctrl] SetModelState: %s", e)

    def fly_to(self, target, prev):
        """
        Velocity-based flight with obstacle avoidance.
        v_total = v_attract + v_avoid, integrated at 50 Hz via SetModelState.
        """
        tx, ty, tz, tyaw_deg, _ = target
        px, py, pz, pyaw_deg, _ = prev
        tyaw = math.radians(tyaw_deg)
        pyaw = math.radians(pyaw_deg)

        cx, cy, cz, cyaw = px, py, pz, pyaw
        elapsed = 0.0
        log_t   = 0.0
        timeout = self.move_duration * 4.0

        while not rospy.is_shutdown():
            dxy = math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2)
            if dxy < 0.15 and abs(tz - cz) < 0.10:
                break

            spd = min(self.cruise_speed, dxy)
            if dxy > 0.05:
                vxa = (tx - cx) / dxy * spd
                vya = (ty - cy) / dxy * spd
            else:
                vxa = vya = 0.0

            vz   = clamp((tz - cz) * 2.0, -1.0, 1.0)
            yerr = tyaw - cyaw
            while yerr >  math.pi: yerr -= 2 * math.pi
            while yerr < -math.pi: yerr += 2 * math.pi
            vyaw = clamp(yerr * 2.0, -1.5, 1.5)

            vxv, vyv, dbg = self._avoidance(cyaw)

            # Hard-stop: cancel forward attraction when imminent collision
            with self._scan_lock:
                scan = self._latest_scan
            if scan is not None:
                fn = self._sector_min(scan, -25, +25)
                if fn < self.hard_stop_dist:
                    cos_y = math.cos(cyaw)
                    sin_y = math.sin(cyaw)
                    fwd   = vxa * cos_y + vya * sin_y
                    if fwd > 0.0:
                        vxa -= fwd * cos_y
                        vya -= fwd * sin_y
                    rospy.logwarn_throttle(
                        1.0, "[avoidance] HARD STOP: front=%.2fm", fn)

            vx = vxa + vxv
            vy = vya + vyv
            sp = math.sqrt(vx ** 2 + vy ** 2)
            if sp > self.max_speed:
                vx *= self.max_speed / sp
                vy *= self.max_speed / sp

            cx   += vx   * self._dt
            cy   += vy   * self._dt
            cz   += vz   * self._dt
            cyaw += vyaw * self._dt

            self._set_pose(cx, cy, cz, cyaw)
            self.rate.sleep()

            elapsed += self._dt
            log_t   += self._dt

            if log_t >= 0.5:
                log_t = 0.0
                self.debug_pub.publish(String(data=dbg))
                if abs(vxv) > 0.1 or abs(vyv) > 0.1:
                    rospy.loginfo("[avoidance] %s | cmd=(%+.2f,%+.2f)m/s",
                                  dbg, vx, vy)

            if elapsed > timeout:
                rospy.logwarn("[waypoint_ctrl] Segment timeout %.0fs", timeout)
                break

        self._set_pose(tx, ty, tz, tyaw)

    def run_mission(self):
        iteration = 0
        while not rospy.is_shutdown():
            iteration += 1
            rospy.loginfo("[waypoint_ctrl] === Pass %d ===", iteration)
            for i, wp in enumerate(self.WAYPOINTS):
                if rospy.is_shutdown():
                    return
                prev = self.WAYPOINTS[i - 1] if i > 0 else wp
                msg  = (f"WP {i+1}/{len(self.WAYPOINTS)}: {wp[4]} "
                        f"({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f}) yaw={wp[3]}deg")
                self.status_pub.publish(String(data=msg))
                rospy.loginfo("[waypoint_ctrl] %s", msg)
                self.fly_to(wp, prev)
                rospy.sleep(self.dwell_time)
            rospy.loginfo("[waypoint_ctrl] Pass %d done.", iteration)
            if not self.loop_mission:
                rospy.loginfo("[waypoint_ctrl] Mission complete.")
                break
            rospy.sleep(3.0)


def main():
    ctrl = WaypointController()
    try:
        ctrl.run_mission()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
