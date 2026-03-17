#!/usr/bin/env python3
"""
UAV LiDAR Stack - LiDAR Point Cloud Processor Node
=====================================================
Subscribes to raw VLP-16 point cloud data and applies a processing pipeline:

  1. PassThrough filter  — removes points outside a configurable Z-range
                           relative to the drone body (removes ground reflections
                           and points above the sensor's useful range)
  2. Range filter        — discards points closer than min_range (removes own
                           body returns) and farther than max_range
  3. Voxel-grid downsample — reduces point density to control downstream CPU load
  4. Horizontal LaserScan extraction — projects remaining points onto a 2D plane
                           to produce a sensor_msgs/LaserScan message suitable
                           for hector_mapping (which expects 2D scan input)

Published topics:
  /cloud_filtered  (sensor_msgs/PointCloud2) — filtered 3D cloud
  /scan            (sensor_msgs/LaserScan)   — 2D horizontal scan for SLAM

Subscribed topics:
  /velodyne_points (sensor_msgs/PointCloud2) — raw VLP-16 output

Parameters (loaded from processing_params.yaml):
  z_min, z_max       — passthrough Z filter bounds (metres, sensor frame)
  min_range          — minimum valid point distance (metres)
  max_range          — maximum valid point distance (metres)
  voxel_size         — voxel-grid leaf size (metres)
  scan_height_min/max — Z range projected into the 2D scan (metres, sensor frame)
  angle_min/max      — LaserScan angular limits (radians)
  angle_increment    — LaserScan angular resolution (radians)
  scan_range_min/max — LaserScan range limits (metres)
"""

import rospy
import numpy as np
import struct
import math

from sensor_msgs.msg import PointCloud2, LaserScan, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


# ---------------------------------------------------------------------------
# Helper: parse PointCloud2 → numpy array (x, y, z) without using ros_numpy
# ---------------------------------------------------------------------------
def pointcloud2_to_xyz(msg):
    """
    Convert a sensor_msgs/PointCloud2 message to an (N, 3) float32 numpy array.

    Handles both organized (width * height > 1) and unorganized clouds.
    Returns array shape (N, 3) with columns [x, y, z].
    NaN points (invalid LiDAR returns) are removed.
    """
    # Build field offset map
    field_offsets = {f.name: f.offset for f in msg.fields}
    x_off = field_offsets.get('x', 0)
    y_off = field_offsets.get('y', 4)
    z_off = field_offsets.get('z', 8)
    point_step = msg.point_step

    num_points = msg.width * msg.height
    data = msg.data  # bytes object

    # Pre-allocate output
    xyz = np.empty((num_points, 3), dtype=np.float32)

    # Vectorised extraction using numpy buffer view
    raw = np.frombuffer(data, dtype=np.uint8)
    # Each point occupies point_step bytes; we extract float32 from fixed offsets
    base = np.arange(num_points) * point_step
    xyz[:, 0] = np.frombuffer(raw[np.add.outer(base, np.arange(x_off, x_off+4)).ravel()].tobytes(),
                               dtype=np.float32)
    xyz[:, 1] = np.frombuffer(raw[np.add.outer(base, np.arange(y_off, y_off+4)).ravel()].tobytes(),
                               dtype=np.float32)
    xyz[:, 2] = np.frombuffer(raw[np.add.outer(base, np.arange(z_off, z_off+4)).ravel()].tobytes(),
                               dtype=np.float32)

    # Remove NaN / Inf
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def xyz_to_pointcloud2(xyz_array, frame_id, stamp):
    """
    Convert an (N, 3) float32 numpy array to a sensor_msgs/PointCloud2 message.
    """
    header = Header(frame_id=frame_id, stamp=stamp)
    fields = [
        PointField('x', 0,  PointField.FLOAT32, 1),
        PointField('y', 4,  PointField.FLOAT32, 1),
        PointField('z', 8,  PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(header, fields, xyz_array.tolist())


# ---------------------------------------------------------------------------
# Voxel-grid downsampling (implemented in numpy — no PCL dependency required)
# ---------------------------------------------------------------------------
def voxel_grid_downsample(xyz, voxel_size):
    """
    Approximate voxel-grid downsampling: keep one representative point per voxel.

    Algorithm:
      1. Compute voxel index for each point: idx = floor(point / voxel_size)
      2. Compute a unique hash key per voxel using a large prime pairing
      3. For each unique key, keep the first point encountered (centroid
         approximation; fast and sufficient for scan-matching purposes)

    Parameters
    ----------
    xyz        : (N, 3) float32 numpy array
    voxel_size : float, leaf size in metres

    Returns
    -------
    (M, 3) float32 numpy array, M <= N
    """
    if xyz.shape[0] == 0:
        return xyz

    # Map to voxel indices (integer)
    voxel_idx = np.floor(xyz / voxel_size).astype(np.int64)

    # Create a unique scalar key per (ix, iy, iz) triple using Cantor pairing
    # Large offsets prevent negative indices from causing collisions
    offset = 10000
    keys = (voxel_idx[:, 0] + offset) * (20000 ** 2) + \
           (voxel_idx[:, 1] + offset) * 20000 + \
           (voxel_idx[:, 2] + offset)

    # Find unique voxels and return their first representative point
    _, first_idx = np.unique(keys, return_index=True)
    return xyz[first_idx]


# ---------------------------------------------------------------------------
# LaserScan extraction from 3D cloud
# ---------------------------------------------------------------------------
def cloud_to_laserscan(xyz, angle_min, angle_max, angle_increment,
                        range_min, range_max, scan_z_min, scan_z_max):
    """
    Project points within [scan_z_min, scan_z_max] onto the XY plane and
    produce a LaserScan message (ranges array only; header is set by caller).

    Method:
      - Filter by Z height
      - Compute per-point azimuth: atan2(y, x)
      - Compute per-point range: sqrt(x^2 + y^2)
      - Bin into angle buckets; keep minimum range per bucket (nearest obstacle)

    Parameters
    ----------
    xyz              : (N, 3) float32 numpy array
    angle_min/max    : float, radians
    angle_increment  : float, radians per bin
    range_min/max    : float, metres
    scan_z_min/max   : float, metres (sensor frame filter for scan slice)

    Returns
    -------
    ranges : 1-D float32 numpy array (num_bins,)
    """
    # Z filter: extract a horizontal "slice" of the 3D cloud
    z_mask = (xyz[:, 2] >= scan_z_min) & (xyz[:, 2] <= scan_z_max)
    pts = xyz[z_mask]

    num_bins = int(round((angle_max - angle_min) / angle_increment))
    ranges = np.full(num_bins, np.inf, dtype=np.float32)

    if pts.shape[0] == 0:
        return ranges

    # Compute azimuth and 2D range
    azimuth = np.arctan2(pts[:, 1], pts[:, 0])   # -π to +π
    r2d = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)    # horizontal range

    # Range validity filter
    valid = (r2d >= range_min) & (r2d <= range_max)
    azimuth = azimuth[valid]
    r2d = r2d[valid]

    if azimuth.shape[0] == 0:
        return ranges

    # Compute bin indices
    bin_idx = np.floor((azimuth - angle_min) / angle_increment).astype(np.int32)
    in_range = (bin_idx >= 0) & (bin_idx < num_bins)
    bin_idx = bin_idx[in_range]
    r2d = r2d[in_range]

    # Fill bins with minimum range (nearest obstacle wins)
    # Use numpy.minimum.at for scatter-minimum
    np.minimum.at(ranges, bin_idx, r2d)

    # Replace inf with range_max (no-return convention).
    # Leaving inf causes hector_mapping and obstacle_map_node to crash / misread.
    ranges = np.where(np.isinf(ranges), np.float32(range_max), ranges)
    return ranges


# ---------------------------------------------------------------------------
# LidarProcessorNode
# ---------------------------------------------------------------------------
class LidarProcessorNode:
    """
    Main ROS node class for LiDAR point cloud processing.
    """

    def __init__(self):
        rospy.init_node('lidar_processor', anonymous=False)
        rospy.loginfo("[lidar_processor] Starting LiDAR processor node...")

        # -- Load parameters --
        self.z_min         = rospy.get_param('~z_min',          -0.5)
        self.z_max         = rospy.get_param('~z_max',           2.0)
        self.min_range     = rospy.get_param('~min_range',       0.5)
        self.max_range     = rospy.get_param('~max_range',      50.0)
        self.voxel_size    = rospy.get_param('~voxel_size',      0.10)
        self.scan_z_min    = rospy.get_param('~scan_z_min',     -0.15)
        self.scan_z_max    = rospy.get_param('~scan_z_max',      0.15)
        self.angle_min     = rospy.get_param('~angle_min',      -math.pi)
        self.angle_max     = rospy.get_param('~angle_max',       math.pi)
        self.angle_inc     = rospy.get_param('~angle_increment', 0.00349)  # ~0.2°
        self.scan_range_min= rospy.get_param('~scan_range_min',  0.1)
        self.scan_range_max= rospy.get_param('~scan_range_max', 50.0)
        self.scan_time     = rospy.get_param('~scan_time',       0.1)
        self.input_topic   = rospy.get_param('~input_topic',  '/velodyne_points')
        self.cloud_topic   = rospy.get_param('~cloud_topic',  '/cloud_filtered')
        self.scan_topic    = rospy.get_param('~scan_topic',    '/scan')

        # Statistics counters
        self._msg_count = 0
        self._last_stats_time = rospy.Time.now()

        # -- Publishers --
        self.cloud_pub = rospy.Publisher(
            self.cloud_topic, PointCloud2, queue_size=5)
        self.scan_pub = rospy.Publisher(
            self.scan_topic, LaserScan, queue_size=5)

        # -- Subscriber --
        self.cloud_sub = rospy.Subscriber(
            self.input_topic, PointCloud2,
            self.cloud_callback, queue_size=2)

        rospy.loginfo("[lidar_processor] Parameters loaded:")
        rospy.loginfo("  Input:       %s", self.input_topic)
        rospy.loginfo("  Cloud out:   %s", self.cloud_topic)
        rospy.loginfo("  Scan out:    %s", self.scan_topic)
        rospy.loginfo("  Z filter:    [%.2f, %.2f] m", self.z_min, self.z_max)
        rospy.loginfo("  Range:       [%.2f, %.2f] m", self.min_range, self.max_range)
        rospy.loginfo("  Voxel size:  %.3f m", self.voxel_size)
        rospy.loginfo("  Scan slice:  [%.2f, %.2f] m", self.scan_z_min, self.scan_z_max)
        rospy.loginfo("[lidar_processor] Ready and waiting for point clouds.")

    def cloud_callback(self, msg):
        """
        Main processing callback. Called for each incoming PointCloud2.

        Processing pipeline:
          raw cloud → Z passthrough → range filter → voxel downsample
          → publish filtered cloud
          → extract horizontal slice → publish LaserScan
        """
        t_start = rospy.Time.now()

        # 1. Parse raw PointCloud2 to numpy array
        try:
            xyz_raw = pointcloud2_to_xyz(msg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "[lidar_processor] Parse error: %s", str(e))
            return

        n_raw = xyz_raw.shape[0]
        if n_raw == 0:
            rospy.logwarn_throttle(5.0, "[lidar_processor] Empty point cloud received")
            return

        # 2. PassThrough Z filter (sensor frame)
        z_mask = (xyz_raw[:, 2] >= self.z_min) & (xyz_raw[:, 2] <= self.z_max)
        xyz_z = xyz_raw[z_mask]

        # 3. Range filter (removes own-body returns and far-field noise)
        r3d = np.linalg.norm(xyz_z[:, :3], axis=1)
        range_mask = (r3d >= self.min_range) & (r3d <= self.max_range)
        xyz_range = xyz_z[range_mask]

        # 4. Voxel-grid downsampling
        xyz_filtered = voxel_grid_downsample(xyz_range, self.voxel_size)

        n_filtered = xyz_filtered.shape[0]

        # 5. Publish filtered 3D cloud
        if self.cloud_pub.get_num_connections() > 0 and n_filtered > 0:
            cloud_msg = xyz_to_pointcloud2(xyz_filtered, msg.header.frame_id, msg.header.stamp)
            self.cloud_pub.publish(cloud_msg)

        # 6. Extract horizontal scan slice and publish LaserScan
        ranges = cloud_to_laserscan(
            xyz_filtered,
            self.angle_min, self.angle_max, self.angle_inc,
            self.scan_range_min, self.scan_range_max,
            self.scan_z_min, self.scan_z_max
        )

        scan_msg = LaserScan()
        scan_msg.header.stamp    = msg.header.stamp
        scan_msg.header.frame_id = msg.header.frame_id
        scan_msg.angle_min       = self.angle_min
        scan_msg.angle_max       = self.angle_max
        scan_msg.angle_increment = self.angle_inc
        scan_msg.time_increment  = 0.0
        scan_msg.scan_time       = self.scan_time
        scan_msg.range_min       = self.scan_range_min
        scan_msg.range_max       = self.scan_range_max
        scan_msg.ranges          = ranges.tolist()
        scan_msg.intensities     = []

        self.scan_pub.publish(scan_msg)

        # 7. Periodic statistics logging (every 10 seconds)
        self._msg_count += 1
        now = rospy.Time.now()
        dt = (now - self._last_stats_time).to_sec()
        if dt >= 10.0:
            hz = self._msg_count / dt
            proc_ms = (rospy.Time.now() - t_start).to_sec() * 1000.0
            rospy.loginfo(
                "[lidar_processor] %.1f Hz | raw=%d filtered=%d (%.1f%%) | proc=%.1fms",
                hz, n_raw, n_filtered,
                100.0 * n_filtered / max(n_raw, 1),
                proc_ms
            )
            self._msg_count = 0
            self._last_stats_time = now


def main():
    node = LidarProcessorNode()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
