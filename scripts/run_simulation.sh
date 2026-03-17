#!/bin/bash
# =============================================================================
# UAV LiDAR Autonomy — Simulation Launcher
# =============================================================================
# Launches the complete stack: Gazebo, LiDAR processing, occupancy mapping,
# navigator, and (optionally) RViz.
#
# Prerequisites:
#   1. Ubuntu 20.04 (WSL2 or native)
#   2. Workspace built:  ./scripts/build.sh
#   3. For GUI on WSL2:  export DISPLAY=:0  (WSLg sets this automatically)
#
# Usage:
#   ./scripts/run_simulation.sh                    # Full GUI (default)
#   ./scripts/run_simulation.sh --headless         # No Gazebo GUI, no RViz
#   ./scripts/run_simulation.sh --no-rviz          # Gazebo GUI only
#   ./scripts/run_simulation.sh --start -3 -3      # Custom start position
#   ./scripts/run_simulation.sh --goal  4  6       # Custom goal position

set -e

STACK_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS_ROOT="$STACK_ROOT/catkin_ws"

# Defaults
GUI=true
RVIZ=true
START_X=-5.0
START_Y=-5.0
GOAL_X=5.0
GOAL_Y=5.0

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless)
            GUI=false
            RVIZ=false
            echo "[run] Headless mode: Gazebo GUI and RViz disabled."
            shift ;;
        --no-rviz)
            RVIZ=false
            echo "[run] RViz disabled."
            shift ;;
        --start)
            START_X="$2"; START_Y="$3"; shift 3 ;;
        --goal)
            GOAL_X="$2"; GOAL_Y="$3"; shift 3 ;;
        -h|--help)
            echo "Usage: $0 [--headless] [--no-rviz] [--start X Y] [--goal X Y]"
            exit 0 ;;
        *)
            echo "[warn] Unknown argument: $1"
            shift ;;
    esac
done

echo "=================================================="
echo "  UAV LiDAR Autonomy — Launch"
echo "  GUI:   $GUI    RViz: $RVIZ"
echo "  Start: ($START_X, $START_Y)   Goal: ($GOAL_X, $GOAL_Y)"
echo "=================================================="

# ---------------------------------------------------------------------------
# Source ROS environment
# ---------------------------------------------------------------------------
source /opt/ros/noetic/setup.bash

if [ ! -f "$WS_ROOT/devel/setup.bash" ]; then
    echo "[ERROR] Workspace not built. Run ./scripts/build.sh first."
    exit 1
fi
source "$WS_ROOT/devel/setup.bash"

# ---------------------------------------------------------------------------
# Check display for GUI mode
# ---------------------------------------------------------------------------
if [ "$GUI" = true ] || [ "$RVIZ" = true ]; then
    if [ -z "$DISPLAY" ]; then
        echo ""
        echo "[warn] DISPLAY is not set. GUI windows may not appear."
        echo "[tip]  On Windows 11 WSLg, run:  export DISPLAY=:0"
        echo "[tip]  On older WSL2:  export DISPLAY=\$(grep nameserver /etc/resolv.conf | awk '{print \$2}'):0.0"
        echo "[tip]  If Gazebo fails to render:  export LIBGL_ALWAYS_SOFTWARE=1"
        echo ""
    fi
fi

# ---------------------------------------------------------------------------
# Launch
# ---------------------------------------------------------------------------
echo "[run] Starting simulation..."

roslaunch uav_lidar_bringup uav_lidar_system.launch \
    gui:=$GUI \
    rviz:=$RVIZ \
    start_x:=$START_X \
    start_y:=$START_Y \
    goal_x:=$GOAL_X \
    goal_y:=$GOAL_Y
