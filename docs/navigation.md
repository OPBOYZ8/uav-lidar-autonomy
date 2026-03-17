# Navigation and Obstacle Avoidance

This document describes the navigation algorithm implemented in `uav_navigator.py` in detail, including the mathematical formulation, design decisions, and the history of failure modes that shaped the current design.

---

## Overview

The navigator runs two concurrent layers:

| Layer | Frequency | Responsibility |
|-------|-----------|----------------|
| Layer 1 — reactive controller | 50 Hz | Real-time velocity commands: M-line attraction + LiDAR avoidance + boundary repulsion |
| Layer 2 — global planner | 0.2 Hz | A\* on occupancy grid; path published to RViz; not in the control loop |

Motion is applied by calling Gazebo's `SetModelState` service, which repositions the drone kinematically each tick.

---

## Layer 1: Reactive Controller

Each 50 Hz tick, the controller computes a world-frame velocity by summing three independent contributions:

```
v_total = v_mline + v_avoid + v_boundary
```

Then clamps to `max_speed = 3.0 m/s` and integrates into position.

### M-line Attraction

The M-line is the straight line from the start pose to the goal pose, precomputed at startup:

```python
mdx = goal_x - start_x
mdy = goal_y - start_y
mlen = sqrt(mdx² + mdy²)
mux, muy = mdx/mlen, mdy/mlen     # unit vector along M-line
mlx, mly = -muy, mux              # unit vector perpendicular (left of M-line)
```

Each tick:

1. **Along-line speed** scaled by distance to goal (slows near arrival) and proximity to obstacles (RC6):

   ```
   effective_speed = cruise_speed × goal_scale × obs_scale
   goal_scale = min(1.0, 2.0 × dist_goal)    # tapers in last 0.5 m
   obs_scale  = max(0.30, (nearest_obs - hard_stop) / (safety_dist - hard_stop))
   ```

2. **Lateral correction** pulls the drone back to the M-line if it has drifted sideways:

   ```
   proj_lat = (cx - start_x) × mlx + (cy - start_y) × mly   # signed deviation
   lat_speed = clamp(-proj_lat × mline_gain, -0.6 × eff_speed, +0.6 × eff_speed)
   ```

   `lat_speed > 0` means the drone is to the right of the M-line and is being pulled left.

3. **RC2 soft-stop** multiplies the M-line velocity by a proximity scale factor:

   ```
   fn_eff = min(F, LF / 0.6, RF / 0.6)     # effective front distance
   scale  = max(0.0, (fn_eff - hard_stop) / (safety_dist - hard_stop))
   v_mline *= scale
   ```

   This reaches zero when `fn_eff ≤ hard_stop = 0.55 m`. The 0.6 side-weight means a wall at 0.6 m in the LF or RF sector triggers the same reduction as a direct front obstacle at 1.0 m — catching angled approaches that a ±25° front sector alone would miss.

### LiDAR Avoidance Sectors

The 2D scan is read with obstacle memory applied (see below) and divided into five body-frame sectors each tick:

| Sector | Angular range | Body-frame effect |
|--------|---------------|-------------------|
| FRONT | ±25° | Front steering (see below) |
| LF (left-forward) | +25° to +70° | `by -= g × s` — push right |
| RF (right-forward) | −70° to −25° | `by += g × s` — push left |
| L (left) | +70° to +130° | `by -= g × s × 0.6` — push right |
| R (right) | −130° to −70° | `by += g × s × 0.6` — push left |

Force magnitude: `s = ((safety_dist − d) / safety_dist)^1.5`

The 1.5 exponent produces a smooth ramp: small forces at medium range, sharp forces when close.

`bx` and `by` are body-frame components (FLU convention). They are rotated to world frame before summing:

```python
wx = bx × cos(yaw) - by × sin(yaw)
wy = bx × sin(yaw) + by × cos(yaw)
```

### Front-Steering Design

The critical design decision is what happens when the FRONT sector (±25°) is blocked.

**Previous design (removed)**: `bx -= g × s` — a backward body-frame force proportional to blockage. This fought directly against the M-line attractor and caused bang-bang oscillation at every wall. When the motor housings were also causing false LF/RF readings, RC3 added another backward force on top, driving the drone away from its goal.

**Current design**: when `F < safety_dist`, a lateral `by` force is computed based on which diagonal side has more clearance:

```python
if F < d:
    s = ((d - F) / d) ** 1.5
    if LF > RF + 0.15:
        by += g * s * 0.8    # left more open → steer left
    elif RF > LF + 0.15:
        by -= g * s * 0.8    # right more open → steer right
    else:
        # symmetric: tiebreak on wider side sector
        by += g * s * 0.4 if L >= R else -(g * s * 0.4)
```

The RC2 soft-stop already handles deceleration as the wall approaches. No backward impulse is needed or wanted.

### Sector Obstacle Memory

The scan is read at most once per 50 Hz tick. Each sector's result is filtered through a memory:

```python
if new_reading <= prev_mem:
    new_mem = new_reading          # obstacle closer: accept immediately
else:
    new_mem = min(new_reading, prev_mem + CLEAR_RATE × dt)   # clearing: gradual
```

`CLEAR_RATE = 0.5 m/s`. At 50 Hz, a sector can clear by at most 0.01 m per tick. This prevents the navigator from suddenly seeing "all clear" due to:
- Scan latency (100 ms at 10 Hz, during which the drone may have moved 0.18 m)
- Residual sensor blindspot effects from the VLP-16 plugin
- Single-tick noise spikes

### EMA Smoothing (RC1)

The raw avoidance velocity is smoothed before applying:

```python
vxv = α × vxv_raw + (1 − α) × vxv_prev
```

`α = 0.65` gives a time constant of ~26 ms at 50 Hz — fast enough to react to new obstacles but smooth enough to eliminate control-frequency jitter.

### Boundary Repulsion

A virtual wall repulsion prevents the drone from reaching the physical Gazebo walls (at ±8.1 m) under any control transient:

```python
if drone is within WALL_ZONE = 2.5 m of the ±6.0 m software boundary:
    repulsion = WALL_GAIN × ((WALL_ZONE − d) / WALL_ZONE)²
```

A hard position clamp (`BOUND = 6.0 m`) is applied after every `SetModelState` call as the last line of defence.

---

## Layer 2: A\* Background Planner

Every 5 seconds, a background thread runs A\* on the latest occupancy grid snapshot:

1. Inflate obstacles by `inflate_cells = int(0.5 / resolution)` using `scipy.ndimage.binary_dilation`
2. Convert drone position and goal position to grid cells
3. Run 8-connected A\* with Euclidean heuristic
4. Smooth the resulting cell path to waypoints spaced 0.8 m apart
5. Publish to `/uav/path` for RViz

The path is **not fed into Layer 1**. It serves as a visualization aid and as the lateral direction reference during Phase 2 of stuck recovery.

---

## Stuck Detection and Recovery (RC4 + RC5)

### Detection (RC4)

The stuck detector runs each tick when `recovery_phase == 0`. It maintains 2-second sliding windows of speed and position:

- **Criterion A**: mean speed < 0.06 m/s → genuine no-motion stuck
- **Criterion B**: position bounding-box diagonal < 0.25 m → oscillating in place

Either criterion triggers recovery.

### Recovery (RC5)

Recovery proceeds through three phases with independent timers. LiDAR avoidance remains active throughout all phases.

| Phase | Duration | Behavior |
|-------|----------|----------|
| 1 — Back-off | 1.0 s | Move in the direction of the current avoidance output |
| 2 — Lateral escape | 2.0 s | Move toward the side sector with more clearance + 30% goal pull |
| 3 — Cooldown | 1.5 s | Resume M-line navigation; stuck detector suppressed |

On recovery start:
- Obstacle memory is cleared (stale proximity doesn't block escape)
- Speed and position histories are cleared (stuck detector starts fresh after recovery)
- EMA smooth state is zeroed

---

## Yaw Control

The drone's yaw is continuously slewed to face the goal direction:

```python
goal_angle = atan2(goal_y - cy, goal_x - cx)
yerr = goal_angle - cyaw    # wrapped to (-π, π)
vyaw = clamp(yerr × 2.5, -2.0, 2.0)  # rad/s
```

At 50 Hz, maximum yaw rate is 2.0 rad/s ≈ 115°/s, so the drone realigns within ~1.5 s from any initial heading.

This alignment is important for the avoidance sector geometry: sectors are defined in body frame (FLU), so the drone facing goal ensures that the FRONT sector always points toward the goal, and the LF/RF sectors catch objects the drone is about to fly into.

---

## Parameter Reference

All parameters are ROS param server entries, settable via launch file args:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~cruise_speed` | float | 1.8 | Nominal M-line speed (m/s) |
| `~max_speed` | float | 3.0 | Velocity cap (m/s) |
| `~safety_distance` | float | 1.0 | Avoidance onset distance (m) |
| `~avoidance_gain` | float | 2.5 | Peak avoidance force (m/s) |
| `~hard_stop_distance` | float | 0.55 | M-line scale → 0 below this (m) |
| `~goal_tolerance` | float | 0.4 | Goal-reached radius (m) |
| `~smooth_alpha` | float | 0.65 | EMA coefficient for avoidance |
| `~mline_gain` | float | 1.0 | Lateral M-line correction gain |
| `~replan_rate` | float | 0.2 | A\* replanning frequency (Hz) |
| `~update_rate` | float | 50.0 | Control loop frequency (Hz) |
| `~start_delay` | float | 5.0 | Seconds to wait for Gazebo to stabilise |

---

## Failure Mode History

| Version | Symptom | Root cause | Fix |
|---------|---------|------------|-----|
| Pre-v0.1 | Drone flies full speed into walls | VLP-16 plugin blindspot at 0.9 m | Lowered plugin `min_range` to 0.2 m |
| v0.1–v0.3 | Drone immediately moves away from goal | Motor housing self-detection: LF=RF≈0.29 m → RC3 backward force | Raised `min_range` to 0.45 m; removed RC3 |
| v0.2 | Oscillation at walls | FRONT `bx -= g*s` fought M-line attractor | Replaced with front steering |
| v0.2 | Drone never truly stuck — keeps oscillating | RC4 only checked speed, not spread | Added position bounding-box criterion |
