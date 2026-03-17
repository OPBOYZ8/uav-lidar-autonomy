# Changelog

All notable changes to this project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased]

---

## [0.4.0] — Motor Self-Detection Fix + RC3 Removal

### Fixed
- **Motor housing self-detection**: `min_range` raised from 0.30 m to 0.45 m in
  `lidar_processing.launch` and `uav_lidar_system.launch`. Motor housings sit at a 3D
  range of 0.306 m from the VLP-16; with Gaussian noise (σ = 0.008 m) their returns
  reached 0.322 m and appeared in the LF and RF sectors on every tick, even in open
  space. Rotor tips at 0.433 m are also now filtered. Hard-stop detection at 0.55 m
  is preserved.
- **Backward cascade from false LF/RF readings**: with motor returns causing LF = RF ≈
  0.29 m, the RC2 soft-stop computed fn_eff = 0.483 m < hard_stop and zeroed the
  M-line, while RC3 added −1.197 m/s backward body force. The drone was being driven
  away from its goal on every tick from mission start.

### Changed
- **RC3 corridor backward pressure removed**: RC3 fired whenever LF ≈ RF (symmetric),
  which coincided with motor self-detection. Even with real symmetric obstacles, the
  backward force overwhelmed the M-line attractor. RC3 has been deleted entirely.
- **FRONT sector redesigned**: the `bx -= g*s` backward force for the FRONT (±25°)
  sector replaced with *front steering* — a lateral body-frame push toward whichever
  diagonal side (LF vs RF) has more clearance, with wider side sectors (L vs R) as
  tiebreak. The RC2 soft-stop handles deceleration; no backward impulse is needed.
- **Default mission updated**: start (−5, −5, 1.5 m) → goal (5, 5, 1.5 m). The
  14.14 m diagonal path crosses rack_C2, two interior walls, rack_B1/B2, and
  pillar_NE — an intentional full obstacle-field navigation test.

---

## [0.3.0] — Sensor Blindspot Fix + Sector Obstacle Memory

### Fixed
- **VLP-16 plugin blindspot**: `<min_range>` in the Gazebo plugin was 0.9 m. Once the
  drone closed within 0.9 m of any surface, the plugin stopped publishing returns for
  it. The `lidar_processor` replaced missing returns with `range_max` (50 m), so the
  navigator saw "all clear" and applied full cruise speed into the wall. Fixed by
  lowering the plugin `min_range` to 0.2 m in `quadrotor_lidar.urdf.xacro`.

### Added
- **Sector obstacle memory** (`_sector_mem`): each sector's minimum distance is
  remembered and can only increase (clear) at `CLEAR_RATE = 0.5 m/s` per tick.
  Prevents sudden all-clear jumps from the sensor blindspot residue or 100 ms scan
  latency when approaching at cruise speed.
- `_sector_min_raw()` — raw sector read without memory
- `_get_sectors()` — applies memory stabilisation and returns a unified dict
- `_reset_sector_mem()` — clears memory at start of recovery so stale proximity
  readings don't block the escape direction

### Changed
- `_scan_avoidance()` and `_apply_soft_stop()` now accept pre-computed sectors dict
  (computed once per tick in `run()`) instead of re-reading the scan independently.

---

## [0.2.0] — Wall-Hardening Fixes (RC1–RC6)

### Added
- **RC1**: EMA smoothing on avoidance velocity (α = 0.65), replacing a simple
  proportional controller. Avoidance response time reduced from ~100 ms to ~40 ms.
- **RC2**: Wide-cone soft-stop using an effective front distance `fn_eff` derived from
  FRONT, LF, and RF sectors with a side-weight of 0.6. Catches angled-wall approaches
  that the ±25° FRONT sector alone would miss.
- **RC3** (later removed in v0.4.0): narrow corridor backward pressure.
- **RC4**: dual-criterion stuck detector — speed average < 0.06 m/s or position
  bounding-box diagonal < 0.25 m over a 2-second window.
- **RC5**: 3-phase stuck recovery sequence — back-off (1.0 s) → lateral escape (2.0 s)
  → cooldown (1.5 s). All phases keep LiDAR avoidance active.
- **RC6**: proximity-aware M-line speed scaling (minimum 30% of cruise speed), ensuring
  the goal attractor never completely disappears near obstacles.

---

## [0.1.0] — Initial Navigation Stack

### Added
- `uav_navigator.py`: M-line goal attractor with lateral correction, 5-sector LiDAR
  avoidance, boundary repulsion, A\* background planner, stuck detection.
- `obstacle_map_node.py`: custom Python 2D occupancy mapper using Bresenham
  ray-casting; replaces `hector_mapping` which crashed (SIGSEGV, exit code −11) on
  infinite scan values from the VLP-16.
- `lidar_processor_node.py`: PointCloud2 Z-slice, inf sanitisation, 2D scan
  extraction.

### Removed
- `hector_mapping` from the active launch pipeline. Config files retained in
  `uav_lidar_mapping/` for reference. Root cause: SIGSEGV on inf scan ranges.
- `drone_waypoint_controller.py`: replaced by `uav_navigator.py`. Retained in the
  source tree for reference.
