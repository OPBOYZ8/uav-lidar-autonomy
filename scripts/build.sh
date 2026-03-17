#!/bin/bash
# =============================================================================
# UAV LiDAR Stack - Build Script
# =============================================================================
# Builds the catkin workspace and verifies compilation succeeds.
#
# Usage:
#   bash /home/hamza/robotics_projects/uav_lidar_stack/scripts/build.sh

set -e

STACK_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS_ROOT="$STACK_ROOT/catkin_ws"

echo "=================================================="
echo "  UAV LiDAR Stack - Build"
echo "  Workspace: $WS_ROOT"
echo "=================================================="

# Source ROS
source /opt/ros/noetic/setup.bash

# Build
cd "$WS_ROOT"
echo "[build] Running catkin_make ..."
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee "$STACK_ROOT/build.log"

echo ""
echo "[build] Build complete. Log saved to $STACK_ROOT/build.log"
echo "[build] To use the workspace: source $WS_ROOT/devel/setup.bash"
