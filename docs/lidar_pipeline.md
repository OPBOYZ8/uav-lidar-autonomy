# LiDAR Sensing and Processing Pipeline

This document describes the sensor hardware model, the four-stage processing pipeline, and the calibration decisions behind the `min_range` filter value.

---

## Sensor: Velodyne VLP-16

The VLP-16 is a 16-channel spinning LiDAR with the following characteristics as simulated by the Gazebo plugin:

| Property | Value |
|----------|-------|
| Channels | 16 |
| Vertical FOV | −15° to +15° (2° spacing) |
| Horizontal FOV | 360° |
| Angular resolution | 0.2° horizontal |
| Range | 0.2 m – 130 m |
| Output rate | 10 Hz (configurable) |
| Returns per scan | ~28,800 |
| Noise model | Gaussian, σ = 0.008 m |
| ROS topic | `/velodyne_points` (sensor_msgs/PointCloud2) |

The plugin is configured in `quadrotor_lidar.urdf.xacro`:

```xml
<plugin name="gazebo_ros_velodyne_controller"
        filename="libgazebo_ros_velodyne_laser.so">
  <topicName>/velodyne_points</topicName>
  <frameName>velodyne</frameName>
  <organize_cloud>false</organize_cloud>
  <min_range>0.2</min_range>    <!-- physics-layer cutoff; below sensor blindspot fix -->
  <max_range>130.0</max_range>
  <gaussianNoise>0.008</gaussianNoise>
</plugin>
```

> **Historical note**: the original plugin had `<min_range>0.9</min_range>`. This created a 0.9 m blindspot — any surface closer than 0.9 m produced no returns. The `lidar_processor` substituted missing returns with `range_max` (50 m), so the navigator saw "all clear" and applied full cruise speed into nearby walls. This was the root cause of all wall-collision failures prior to v0.3.

---

## Sensor Frame Convention

The VLP-16 is mounted at `xyz="0 0 0.091"` above `base_link` with `rpy="0 0 0"` — no rotation. This means the `velodyne` frame uses **FLU (Forward-Left-Up)** convention, identical to the drone's body frame:

- **+X = forward** (positive body roll axis)
- **+Y = left**
- **+Z = up**
- **Azimuth 0° = forward, +90° = left, +180°/−180° = backward, −90° = right**

The `lidar_processor` uses `atan2(y, x)` for azimuth, which correctly maps to this convention.

---

## Processing Pipeline (`lidar_processor_node.py`)

### Stage 1 — Z-Band Slice

```python
Z_MIN = -0.30   # m below sensor
Z_MAX = +0.30   # m above sensor
```

Only points within this band are retained. This eliminates:
- Ground returns (the drone is at 1.5 m altitude; the ground is ~1.4 m below the sensor)
- Ceiling/overhead structure if present
- Rotor blade returns at extreme vertical angles

The ±0.30 m band was chosen to be wide enough to capture returns from short obstacles at the cruise altitude, while rejecting the vast majority of ground and ceiling returns.

### Stage 2 — inf → range_max Substitution

The VLP-16 plugin outputs `inf` for beams that do not return (sky, long-range dropouts, or — before the min_range fix — the blindspot region). `sensor_msgs/LaserScan` does not handle `inf` gracefully in all downstream nodes.

```python
ranges = np.where(np.isinf(ranges), scan_range_max, ranges)
# scan_range_max = 50.0 m
```

After substitution, all values are finite. Downstream nodes (including `obstacle_map_node`) treat any range ≥ 49.9 m as "no return / free space".

### Stage 3 — min_range Filter (Motor Housing Suppression)

```python
MIN_RANGE = 0.45   # m
```

Points closer than 0.45 m are discarded. This value was derived from URDF geometry:

**Motor housing geometry:**
- URDF position: `pos_x = ±0.21 m, pos_y = ±0.21 m, z = 0.005 m above base_link`
- VLP-16 position: `z = 0.091 m above base_link`
- Vertical offset from VLP-16 to motor housing: `0.091 − 0.005 = 0.0735 m` (below sensor)
- Horizontal distance: `√(0.21² + 0.21²) = 0.297 m`
- **3D range from VLP-16**: `√(0.297² + 0.0735²) = 0.306 m`
- **Bearing angle**: `atan2(±0.21, ±0.21) = ±45°` → lands in LF (+25°–+70°) and RF (−70°–−25°) sectors

**With Gaussian noise (σ = 0.008 m):**
- Mean return: 0.306 m
- 2σ high: 0.322 m
- Setting `min_range = 0.45 m` filters **all** motor-housing returns

**Rotor tip geometry:**
- Rotor blades extend to ~0.43 m horizontal radius
- 3D range ≈ 0.433 m → also filtered at 0.45 m

**Margin verification:**
- Hard-stop distance = 0.55 m > min_range = 0.45 m → wall returns at hard-stop range still pass ✓
- Available margin: 0.55 − 0.45 = 0.10 m

**Historical failure**: with `min_range = 0.30 m`, the motor housing mean return (0.306 m) passed the filter. LF and RF sectors showed 0.29–0.31 m on every tick, even in open space. This triggered RC2 soft-stop and RC3 corridor pressure, driving the drone backwards from mission start.

### Stage 4 — Azimuth Binning → LaserScan

Surviving 3D points are projected to the XY plane. Each point is assigned to a bearing bin using:

```python
azimuth = atan2(y, x)   # radians; FLU convention
```

For each bin, the minimum range (nearest obstacle wins) is stored. The result is a `sensor_msgs/LaserScan` with:
- `angle_min = -π`, `angle_max = +π`
- `angle_increment` derived from bin width
- `range_min = 0.45 m` (matches stage 3)
- `range_max = 50.0 m`

---

## Downstream Consumers

### `obstacle_map_node.py`

Consumes `/scan` plus `/ground_truth/odom` for ray-casting origin. Iterates over each beam, marks cells along the ray as free, and marks the endpoint cell as occupied. Uses a log-odds update model:

```
log_odds_occ  = +0.85   # occupancy evidence
log_odds_free = -0.40   # free-space evidence
log_odds_min  = -2.0    # clamp
log_odds_max  = +3.5    # clamp
```

Grid: 320×320 cells at 0.05 m/cell → ±8 m coverage. Published as `nav_msgs/OccupancyGrid` at 2 Hz.

### `uav_navigator.py`

Reads `/scan` directly at 50 Hz for the reactive Layer 1 controller. Also subscribes to `/map` for the 0.2 Hz A\* Layer 2 planner. The navigator applies the sector obstacle memory filter on top of the already-filtered `/scan` values.

---

## Sensor Pipeline Validation

While the simulation is running, use `scripts/validate_pipeline.sh` to confirm all topics are live:

```bash
bash scripts/validate_pipeline.sh
```

Expected output:
```
Checking /velodyne_points              ... PASS  [sensor_msgs/PointCloud2]
Checking /cloud_filtered               ... PASS  [sensor_msgs/PointCloud2]
Checking /scan                         ... PASS  [sensor_msgs/LaserScan]
Checking /map                          ... PASS  [nav_msgs/OccupancyGrid]
Checking /uav/pose                     ... PASS  [geometry_msgs/PoseStamped]
Checking /uav/nav_debug                ... PASS  [std_msgs/String]
```
