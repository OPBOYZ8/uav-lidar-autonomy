# System Architecture

This document is the authoritative reference for the UAV LiDAR Autonomy system architecture. It describes every component, its responsibility, its inputs/outputs, and the design decisions behind it.

---

## Technology Stack

| Layer | Component | Choice | Rationale |
|-------|-----------|--------|-----------|
| OS | Ubuntu 20.04 LTS | WSL2-compatible; ROS Noetic target platform |
| Middleware | ROS Noetic | LTS release for Ubuntu 20.04; richest ecosystem for robotics prototyping |
| Simulator | Gazebo 11 Classic | Stable pairing with Noetic; best-in-class plugin support including VLP-16 |
| LiDAR | Velodyne VLP-16 plugin | Hardware-accurate 3D scan model with configurable noise; PointCloud2 output |
| Mapping | Custom `obstacle_map_node.py` | `hector_mapping` was removed (SIGSEGV on inf scan values); custom node is simpler, Python-native, no C++ dependencies |
| Navigation | Custom `uav_navigator.py` | Full control over all design decisions; no nav-stack blackbox |
| Visualisation | RViz | Native ROS 1 tool; full 3D, map, and path visualisation |
| Motion interface | Gazebo `SetModelState` | Kinematic pose injection; bypasses dynamics for fast, deterministic iteration |

**Why not ROS 2?** Ubuntu 20.04's supported ROS 2 releases are Foxy and Galactic, both of which are end-of-life. Noetic was already installed and provides a more mature package ecosystem for this workload.

**Why not `hector_mapping`?** `hector_mapping` crashed with exit code −11 (SIGSEGV) when the VLP-16 scan contained `inf` values. The custom `obstacle_map_node.py` consumes the already-sanitised `/scan` topic, has no C++ crash risk, and is transparent for debugging.

---

## Data Flow

```
Gazebo VLP-16 plugin
        │
        │  /velodyne_points  (sensor_msgs/PointCloud2, 10 Hz)
        ▼
lidar_processor_node.py
  ① Z-slice ±0.30 m around cruise altitude
  ② inf → scan_range_max (50 m)
  ③ min_range filter 0.45 m  ←  motor housing self-reflection suppression
  ④ azimuth binning → sensor_msgs/LaserScan
        │
        ├─── /cloud_filtered  (PointCloud2, 10 Hz)  ──► RViz
        │
        └─── /scan  (LaserScan, 10 Hz)
               │
               ├──► obstacle_map_node.py
               │       Bresenham ray-casting + log-odds update
               │       320×320 grid @ 0.05 m/cell
               │       /map  (OccupancyGrid, 2 Hz)  ──► uav_navigator, RViz
               │
               └──► uav_navigator.py  (reads directly, 50 Hz)
                       Layer 1: M-line + avoidance + soft-stop + recovery
                       Layer 2: A* background thread (0.2 Hz)
                       /gazebo/set_model_state  ──► Gazebo
                       /uav/pose, /uav/path, /uav/nav_debug  ──► RViz / monitoring
```

---

## ROS Topic Map

| Topic | Message type | Publisher | Subscriber(s) | Rate |
|-------|-------------|-----------|---------------|------|
| `/velodyne_points` | PointCloud2 | Gazebo VLP-16 plugin | lidar_processor | 10 Hz |
| `/cloud_filtered` | PointCloud2 | lidar_processor | RViz | 10 Hz |
| `/scan` | LaserScan | lidar_processor | obstacle_map_node, uav_navigator | 10 Hz |
| `/map` | OccupancyGrid | obstacle_map_node | uav_navigator (A*), RViz | 2 Hz |
| `/uav/pose` | PoseStamped | uav_navigator | RViz | 50 Hz |
| `/uav/path` | Path | uav_navigator (A*) | RViz | 0.2 Hz |
| `/uav/nav_debug` | String | uav_navigator | monitoring / logging | 2 Hz |
| `/ground_truth/odom` | Odometry | Gazebo P3D plugin | obstacle_map_node (ray origin) | 50 Hz |
| `/tf` | TF | robot_state_publisher | all | 50+ Hz |

---

## Package Descriptions

### `uav_lidar_sim`

Contains the robot model and simulation environment.

- **`urdf/quadrotor_lidar.urdf.xacro`** — quadrotor chassis with four motor housings, rotor visual joints, an IMU link, a LiDAR mount, and the VLP-16 sensor. The `gravity=false` flag in the Gazebo model block allows the drone to float without a flight controller. Key geometry used in sensor calibration:
  - Motor housings: pos_x = ±0.21 m, pos_y = ±0.21 m → diagonal 0.297 m, 0.0735 m below VLP-16 → 3D range 0.306 m
  - VLP-16 at xyz = "0 0 0.091" above base_link; FLU convention (no rotation)
- **`worlds/warehouse.world`** — 16 m × 16 m SDF world. Obstacle layout is fixed; modify this file to experiment with different configurations.
- **`launch/spawn_uav.launch`** — spawns the URDF model into a running Gazebo instance.

### `uav_lidar_processing`

Contains all perception and navigation Python nodes.

- **`src/lidar_processor_node.py`** — subscribes to `/velodyne_points`, applies the 4-stage filter chain, publishes `/cloud_filtered` and `/scan`.
- **`src/obstacle_map_node.py`** — subscribes to `/scan` and `/ground_truth/odom`, maintains a 320×320 log-odds occupancy grid, publishes `/map` at 2 Hz.
- **`src/uav_navigator.py`** — full navigation controller. See [`navigation.md`](navigation.md).
- **`src/drone_waypoint_controller.py`** — archived predecessor node. Kept for reference; not launched.
- **`config/processing_params.yaml`** — default sensor processing parameters.
- **`launch/lidar_processing.launch`** — launches all three active nodes with parameter overrides.

### `uav_lidar_mapping`

Archived `hector_mapping` configuration. The package builds cleanly but is not included in the master launch file. Retained for:
- Historical reference and comparison
- Potential future use with a different scan sanitisation strategy

### `uav_lidar_bringup`

System integration — the only package a user needs to interact with directly.

- **`launch/uav_lidar_system.launch`** — single self-contained launch file. Starts Gazebo, spawns the UAV model, launches all processing and navigation nodes, and opens RViz. All tunable parameters are exposed as `<arg>` elements.
- **`config/rviz/uav_lidar.rviz`** — pre-configured RViz layout showing: 3D point cloud (`/cloud_filtered`), 2D occupancy map (`/map`), A\* path (`/uav/path`), drone pose (`/uav/pose`), laser scan (`/scan`).

---

## TF Frame Tree

```
world
└── base_link          (drone body; positioned by SetModelState)
    ├── frame_cross    (structural arms, visual only)
    ├── rotor_*_arm    (motor arm links)
    ├── rotor_*_motor  (motor housing links — origin of self-reflection returns)
    ├── rotor_*_rotor  (spinning rotor visual joints)
    ├── imu_link       (IMU sensor; not used in current navigation)
    └── lidar_mount    (rigid mount for sensor)
        └── velodyne   (VLP-16 sensor origin; scan published in this frame)
```

The navigator reads drone pose from Gazebo ground truth and writes back via `SetModelState`. No `map → base_link` TF is published; the occupancy map uses ground-truth positions directly.

---

## Performance Notes (WSL2)

| Metric | Typical value |
|--------|--------------|
| VLP-16 raw points per scan | ~28,800 |
| After Z-slice + range filter | ~3,000–8,000 |
| lidar_processor CPU | <5% |
| obstacle_map_node CPU | <5% |
| uav_navigator CPU | <3% |
| Gazebo + VLP-16 plugin | 2–4 CPU cores |
| Total RAM | ~2 GB |

If Gazebo performance is poor on WSL2, try:
```xml
<!-- In warehouse.world, under <scene>: -->
<shadows>false</shadows>
```

---

## Design Decisions and Trade-offs

**SetModelState vs. cmd_vel**: Using `SetModelState` bypasses all flight physics, making the simulation deterministic and removing the need to tune a PID flight controller. The trade-off is that the motion model is unrealistic (instant velocity changes, no inertia). This is acceptable for the current goal of validating the perception and navigation algorithms; a hardware deployment would replace this with MAVROS.

**Custom mapping vs. hector_mapping**: `hector_mapping` crashed on `inf` scan values. Rather than patching or wrapping it, a custom Python mapper was implemented. The custom node is ~200 lines, fully transparent, and easier to modify than a compiled C++ package. The trade-off is that it lacks the multi-resolution scan-matching of `hector_mapping` and uses ground-truth odometry instead of scan-matching localization.

**Reactive controller vs. pure global planner**: A pure A\* planner would require a complete map before starting and would fail in partially-observed environments. The reactive layer provides immediate obstacle response at 50 Hz while the A\* planner runs at 0.2 Hz in the background. This combination allows the drone to start moving with zero prior map knowledge.
