# UAV LiDAR Autonomy

> **Autonomous quadrotor navigation through a cluttered indoor environment using real-time LiDAR perception and reactive obstacle avoidance — no GPS, no prebuilt map, no hardcoded path.**

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue?logo=ros)](http://wiki.ros.org/noetic)
[![Gazebo 11](https://img.shields.io/badge/Gazebo-11-orange)](https://gazebosim.org)
[![Python 3.8](https://img.shields.io/badge/Python-3.8-blue?logo=python)](https://www.python.org)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu)](https://ubuntu.com)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

<!-- Placeholder: replace with a GIF/screenshot of the simulation running -->
<!-- ![Demo](images/demo.gif) -->

## Project Overview

This project implements a full sensor-to-motion autonomy pipeline for a simulated quadrotor UAV equipped with a **Velodyne VLP-16 LiDAR**. The drone navigates a 16 m × 16 m indoor warehouse — filled with storage racks, boxes, partition walls, and pillars — flying from an arbitrary start pose to a goal pose entirely on the basis of what its LiDAR sees in real time.

There is no GPS, no prebuilt map loaded at startup, and no hand-authored waypoint sequence. The system builds a 2D occupancy map incrementally as the drone moves, uses a goal-directed potential-field controller to maintain forward progress, and steers around obstacles by reasoning about which direction offers the most clearance. The entire perception, mapping, and navigation stack is implemented from scratch in Python on top of **ROS Noetic + Gazebo 11 Classic**.

This project was developed as a deep investigation into sensor-driven UAV autonomy — covering the complete pipeline from raw 3D point cloud to closed-loop kinematic control — with a particular focus on robust failure-mode analysis and iterative hardware-accurate sensor modeling.

---

## Key Features

- **Velodyne VLP-16 simulation** via the Gazebo plugin, producing a full 360° 3D point cloud at 10 Hz
- **Custom LiDAR processing node**: slices the point cloud to cruise altitude, filters motor-housing self-reflections (a hardware-accurate effect derived from URDF geometry), and generates a 2D scan
- **Incremental 2D occupancy mapping** via Bresenham ray-casting — no `hector_mapping` or `gmapping` dependency
- **Dual-layer navigation**: 50 Hz reactive potential-field controller paired with a 0.2 Hz background A\* planner on the occupancy grid
- **M-line goal attractor** with lateral deviation correction, maintaining a straight-line trajectory in open space
- **Front-steering obstacle avoidance**: when a wall appears ahead, the drone steers *toward the clearer side* rather than pushing backward against the attractor
- **RC2 wide-cone soft-stop**: progressive M-line deceleration across a ±70° effective cone, catching angled-wall approaches
- **Sector obstacle memory**: each sector's minimum distance can only increase at 0.5 m/s, preventing sudden all-clear jumps from sensor latency or scan dropouts
- **EMA-smoothed avoidance** at α = 0.65, eliminating jitter at 50 Hz control frequency
- **Dual-criterion stuck detector** (speed average + position spread) with a 3-phase recovery sequence
- **Hard boundary clamp**: absolute position limit prevents the drone from escaping the warehouse regardless of control transients

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         Gazebo Simulation                            │
│   ┌──────────────────────────────────────────────────────────────┐   │
│   │  Quadrotor + VLP-16  (quadrotor_lidar.urdf.xacro)           │   │
│   │  warehouse.world — 16m×16m, racks / boxes / walls / pillars │   │
│   └───────────────────────────┬──────────────────────────────────┘   │
│                               │ /velodyne_points  (PointCloud2,10Hz) │
└───────────────────────────────┼──────────────────────────────────────┘
                                │
                   ┌────────────▼────────────┐
                   │  lidar_processor_node   │
                   │  ① Z-band slice ±0.30m  │
                   │  ② inf → range_max      │
                   │  ③ min_range 0.45m      │  ← motor-housing fix
                   │     (body self-filter)  │
                   └──────┬──────────┬───────┘
                          │          │
               /scan       │          │  /cloud_filtered
           (LaserScan)     │          │  (PointCloud2)
                          │          │
         ┌────────────────▼───┐   ┌──▼──────────────────┐
         │  obstacle_map_node │   │  RViz / downstream   │
         │  Bresenham ray-cast│   └─────────────────────-┘
         │  log-odds grid     │
         │  320×320 @ 0.05m   │
         └──────────┬─────────┘
                    │  /map  (OccupancyGrid, 2 Hz)
                    │
         ┌──────────▼──────────────────────────────────────┐
         │                  uav_navigator                   │
         │                                                  │
         │  Layer 1 — 50 Hz reactive controller             │
         │  ├─ M-line attractor (along + lateral correction)│
         │  ├─ RC2 wide-cone soft-stop                      │
         │  ├─ Front-steering avoidance (5 body sectors)    │
         │  ├─ EMA smoothing α=0.65                         │
         │  ├─ Sector obstacle memory (0.5 m/s clear rate)  │
         │  ├─ Boundary repulsion (virtual walls at ±6m)    │
         │  └─ 3-phase stuck recovery (RC4 detect + RC5)    │
         │                                                  │
         │  Layer 2 — 0.2 Hz A* background planner          │
         │  └─ Inflated occupancy → path → /uav/path (RViz) │
         └──────────┬──────────────────────────────────────-┘
                    │  /gazebo/set_model_state  (50 Hz)
                    ▼
             ┌─────────────┐
             │   Gazebo    │  ← pose updated kinematically each tick
             └─────────────┘

Debug topics published by navigator:
  /uav/pose       PoseStamped  — drone position at 50 Hz
  /uav/path       Path         — current A* plan
  /uav/nav_debug  String       — human-readable nav state at 2 Hz
```

For a detailed breakdown of each component, see [`docs/architecture.md`](docs/architecture.md).

---

## Simulation Environment

The warehouse world (`warehouse.world`) is a fully enclosed 16 m × 16 m space with 0.1 m-thick exterior walls at ±8.1 m. The interior obstacle layout is deliberately designed so that the default diagonal mission crosses multiple objects:

| Object | World position | Footprint |
|--------|---------------|-----------|
| Storage rack C1 | (−4, −5) | 0.4 × 3.0 m |
| Storage rack C2 | (−6, −5) | 0.4 × 3.0 m |
| Storage rack B1 | (+4, +5) | 0.4 × 3.0 m |
| Storage rack B2 | (+6, +5) | 0.4 × 3.0 m |
| Cardboard box 1 | (−2.5, −0.5) | 0.5 × 0.5 m |
| Cardboard box 2 | (+2.5, +0.5) | 0.5 × 0.5 m |
| Interior wall 1 | (0, −2) | 4.0 × 0.15 m |
| Interior wall 2 | (0, +2) | 4.0 × 0.15 m |
| Corner pillar NE | (+6.5, +6.5) | r = 0.15 m cylinder |
| Corner pillar SW | (−6.5, −6.5) | r = 0.15 m cylinder |

**Default mission**: (−5, −5, 1.5 m) → (5, 5, 1.5 m) — a 14.14 m diagonal that intersects rack_C2, two interior walls, rack_B1/B2, and pillar_NE directly. The drone must avoid all of these using only real-time LiDAR sensing.

---

## LiDAR Sensing and Processing

See [`docs/lidar_pipeline.md`](docs/lidar_pipeline.md) for the full breakdown. Summary:

The VLP-16 Gazebo plugin publishes a 3D point cloud at 10 Hz. `lidar_processor_node` applies four sequential stages:

1. **Z-band slice** — keeps only points within ±0.30 m of the drone's cruise altitude
2. **inf → range_max substitution** — prevents NaN/inf from propagating to downstream nodes
3. **min_range filter at 0.45 m** — suppresses motor-housing self-reflections. This value was derived from URDF geometry: motor housings sit at a 3D range of 0.306 m from the VLP-16; with Gaussian noise (σ = 0.008 m) their returns reach 0.322 m; rotor tips reach 0.433 m. Setting min_range = 0.45 m eliminates all self-returns while preserving wall returns at hard-stop distance (0.55 m).
4. **Azimuth → 2D scan** — projects surviving points to a `sensor_msgs/LaserScan` in the drone's body frame (FLU convention)

---

## Navigation and Obstacle Avoidance

See [`docs/navigation.md`](docs/navigation.md) for the full breakdown. Summary:

**M-line attractor (Layer 1, 50 Hz)**: A velocity is computed along the straight line from start to goal, scaled by distance and proximity, with a lateral correction term that pulls the drone back to the line if it has drifted sideways during avoidance.

**Reactive avoidance (Layer 1, 50 Hz)**: The 2D scan is divided into five body-frame sectors. Each sector within `safety_distance` (1.0 m) contributes a lateral body-frame push. The FRONT sector (±25°) applies front-steering — a lateral push toward whichever diagonal side has more clearance — rather than a backward force. This prevents the fight between the attractor and a backward repulsor that caused oscillation in earlier designs. The RC2 soft-stop progressively reduces M-line speed as the drone approaches a wall, reaching zero at `hard_stop_distance` (0.55 m).

**A\* planner (Layer 2, 0.2 Hz)**: Runs on the current occupancy grid in a background thread. Its output is published to RViz for visualization; it is not in the real-time control loop.

**Recovery (RC4 + RC5)**: Stuck detection fires if average speed < 0.06 m/s or position spread < 0.25 m over a 2-second window. Recovery proceeds in three timed phases: back-off, lateral escape, cooldown.

---

## Repository Structure

```
uav-lidar-autonomy/
├── README.md                              # This file
├── LICENSE                                # MIT License
├── .gitignore                             # ROS / Python / IDE excludes
├── CONTRIBUTING.md                        # Development and PR guidelines
├── CHANGELOG.md                           # Version and change history
│
├── docs/
│   ├── architecture.md                    # Full system architecture reference
│   ├── navigation.md                      # Navigation algorithm deep-dive
│   ├── lidar_pipeline.md                  # Sensor processing pipeline details
│   └── setup_wsl2.md                      # WSL2 + ROS Noetic setup guide
│
├── images/
│   └── PLACEHOLDER.md                     # Screenshot / figure instructions
│
├── scripts/
│   ├── setup.sh                           # Source ROS + workspace (run with source)
│   ├── install_deps.sh                    # Install all ROS + Python dependencies
│   ├── build.sh                           # catkin_make wrapper with logging
│   ├── run_simulation.sh                  # Full-stack launcher with CLI flags
│   └── validate_pipeline.sh              # Live topic health-check (post-launch)
│
└── catkin_ws/
    └── src/
        ├── uav_lidar_sim/                 # Robot model + Gazebo world
        │   ├── urdf/quadrotor_lidar.urdf.xacro
        │   ├── worlds/warehouse.world
        │   └── launch/spawn_uav.launch
        │
        ├── uav_lidar_processing/          # Perception + navigation nodes
        │   ├── src/
        │   │   ├── lidar_processor_node.py    # PointCloud2 → LaserScan
        │   │   ├── obstacle_map_node.py       # LaserScan → OccupancyGrid
        │   │   └── uav_navigator.py           # M-line + avoidance controller
        │   ├── config/processing_params.yaml
        │   └── launch/lidar_processing.launch
        │
        ├── uav_lidar_mapping/             # hector_mapping config (archived — see docs)
        │
        └── uav_lidar_bringup/             # Master launch + RViz config
            ├── config/rviz/uav_lidar.rviz
            └── launch/uav_lidar_system.launch
```

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 20.04 LTS (Focal) — native or WSL2 |
| ROS | Noetic Ninjemys |
| Gazebo | 11 Classic |
| Python | 3.8 |

> **Windows / WSL2 users**: see [`docs/setup_wsl2.md`](docs/setup_wsl2.md) for display configuration with WSLg.

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/<your-username>/uav-lidar-autonomy.git
cd uav-lidar-autonomy
```

### 2. Install dependencies

```bash
chmod +x scripts/install_deps.sh
./scripts/install_deps.sh
```

### 3. Build the workspace

```bash
chmod +x scripts/build.sh
./scripts/build.sh
```

---

## Running the Simulation

### Quick start

Open a terminal inside Ubuntu 20.04:

```bash
# WSL2 / WSLg — set display first
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1

source scripts/setup.sh
roslaunch uav_lidar_bringup uav_lidar_system.launch
```

Or use the launch helper:

```bash
./scripts/run_simulation.sh
./scripts/run_simulation.sh --headless        # no GUI
./scripts/run_simulation.sh --no-rviz         # Gazebo only
```

This opens Gazebo with the warehouse world, spawns the drone at (−5, −5, 1.5 m), starts all processing nodes, and opens RViz with the LiDAR scan, occupancy map, and A\* path overlays.

### Custom mission

```bash
roslaunch uav_lidar_bringup uav_lidar_system.launch \
  start_x:=-3 start_y:=-3 \
  goal_x:=6   goal_y:=4
```

### Key tunable parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `start_x / start_y` | −5, −5 | Start position (m) |
| `goal_x / goal_y` | 5, 5 | Goal position (m) |
| `cruise_speed` | 1.8 m/s | Nominal M-line speed |
| `max_speed` | 3.0 m/s | Velocity cap |
| `safety_distance` | 1.0 m | Avoidance onset range |
| `avoidance_gain` | 2.5 m/s | Peak avoidance force |
| `hard_stop_distance` | 0.55 m | M-line zeroed below this |
| `smooth_alpha` | 0.65 | EMA coefficient |
| `goal_tolerance` | 0.4 m | Goal-reached threshold |

---

## Monitoring

**Terminal 2** — stream navigation state:
```bash
source scripts/setup.sh
rostopic echo /uav/nav_debug
```

Normal output during open-space flight:
```
pos=(-3.21,-3.18,1.50) goal_dist=11.35m lat_dev=+0.04m spd=1.78m/s
nobs=inf F=inf LF=inf RF=inf L=inf R=inf body=(+0.00,+0.00) world=(+0.00,+0.00)
```

Normal output during obstacle avoidance:
```
pos=(-1.90,-1.87,1.50) goal_dist=10.12m lat_dev=+0.01m spd=1.12m/s
nobs=0.81m F=0.81m LF=0.73m RF=3.10m body=(+0.00,-1.24) world=(-0.88,-0.88)
```

`goal_dist` should decrease monotonically in open space. The drone steering around an obstacle will cause a brief increase, followed by resumption of decrease.

**Validate all topics are live** (while simulation is running):
```bash
bash scripts/validate_pipeline.sh
```

**Check map update rate**:
```bash
rostopic hz /map     # should be ~2 Hz
rostopic hz /scan    # should be ~10 Hz
```

---

## Results and Current Status

| Capability | Status |
|------------|--------|
| Sensor pipeline (PointCloud2 → LaserScan) | ✅ Operational |
| Motor-housing self-reflection suppression | ✅ Fixed (min_range 0.45 m) |
| Incremental occupancy grid mapping | ✅ Operational |
| M-line goal-directed navigation | ✅ Operational |
| Front-steering obstacle avoidance (no oscillation) | ✅ Operational |
| A\* background planner + RViz path visualization | ✅ Operational |
| Stuck detection + 3-phase recovery | ✅ Operational |
| Full 14 m diagonal warehouse crossing | 🔄 In validation |
| Quantitative success-rate benchmarking | 📋 Planned |
| Video / GIF demo | 📋 Planned |

> **In validation** = architecture complete and all unit tests pass; formal end-to-end success-rate measurement across randomized obstacle configurations is not yet complete.

---

## Known Limitations

- **Kinematic control only**: the drone is repositioned each tick via Gazebo's `SetModelState` service. There is no flight dynamics model, motor mixing, or attitude controller. Aerodynamic effects, rotor wash, and inertia are not simulated.
- **Fixed cruise altitude**: the drone operates in a horizontal plane at a fixed Z. Vertical obstacle avoidance is not implemented.
- **Ground-truth odometry**: the occupancy mapper uses Gazebo ground-truth pose for ray-casting origins. A hardware deployment would require onboard localization (e.g., EKF with IMU and scan-matching).
- **Static environment**: the planner and controller assume obstacles are stationary. Moving obstacles are not handled.
- **No global mission re-routing**: if A\* finds no path, the drone falls back to reactive control only. There is no higher-level mission abort or rerouting logic.

---

## Future Improvements

| Improvement | Priority |
|-------------|----------|
| MAVROS + PX4 flight controller integration | High |
| EKF-based localization (IMU + scan matching) — remove ground-truth dependency | High |
| 3D voxel mapping with OctoMap | Medium |
| Dynamic obstacle detection and avoidance | Medium |
| VFH+ or DWA replacement for the reactive layer | Medium |
| D\* Lite for replanning in changing environments | Medium |
| Multi-goal mission sequencing | Low |
| ROS 2 (Humble) port | Low |
| Hardware deployment on physical vehicle | Low |

---

## GitHub About

**Short description**:
> Autonomous LiDAR-guided quadrotor navigation in a simulated cluttered warehouse — ROS Noetic, Gazebo 11, VLP-16, reactive avoidance + A\* planning, full Python implementation.

**Topics / tags**:
`ros` · `ros-noetic` · `gazebo` · `lidar` · `velodyne` · `obstacle-avoidance` · `autonomous-navigation` · `uav` · `quadrotor` · `robotics` · `simulation` · `path-planning` · `occupancy-grid` · `python`

---

## License

Released under the **MIT License** — see [`LICENSE`](LICENSE) for terms.
