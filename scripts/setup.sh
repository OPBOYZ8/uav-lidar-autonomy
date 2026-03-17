#!/bin/bash
# =============================================================================
# UAV LiDAR Stack - Environment Setup Script
# =============================================================================
# Sources ROS Noetic and the catkin workspace so all ROS commands work correctly.
# Run this script (or add it to ~/.bashrc) before using any ROS commands.
#
# Usage:
#   source /home/hamza/robotics_projects/uav_lidar_stack/scripts/setup.sh
#
# Or add to ~/.bashrc:
#   echo "source ~/robotics_projects/uav_lidar_stack/scripts/setup.sh" >> ~/.bashrc

set -e

STACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$STACK_ROOT/catkin_ws"

echo "[setup] UAV LiDAR Stack root: $STACK_ROOT"
echo "[setup] Catkin workspace:     $WS_ROOT"

# -- 1. Source ROS Noetic --
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo "[setup] Sourced ROS Noetic"
else
    echo "[setup] ERROR: ROS Noetic not found at /opt/ros/noetic"
    return 1
fi

# -- 2. Source catkin workspace (if built) --
if [ -f "$WS_ROOT/devel/setup.bash" ]; then
    source "$WS_ROOT/devel/setup.bash"
    echo "[setup] Sourced catkin workspace"
else
    echo "[setup] WARNING: Workspace not yet built. Run: cd $WS_ROOT && catkin_make"
fi

# -- 3. Export Gazebo model path so custom models are found --
export GAZEBO_MODEL_PATH="$STACK_ROOT/catkin_ws/src/uav_lidar_sim/models:$GAZEBO_MODEL_PATH"

# -- 4. Export GAZEBO_PLUGIN_PATH (usually not needed; plugins are in ROS path) --
# export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:$GAZEBO_PLUGIN_PATH"

# -- 5. Convenience aliases --
alias uav_launch="roslaunch uav_lidar_bringup uav_lidar_system.launch"
alias uav_build="cd $WS_ROOT && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee build.log"
alias uav_save_map="rosrun map_server map_saver -f $STACK_ROOT/catkin_ws/src/uav_lidar_mapping/maps/warehouse_map"
alias uav_rosgraph="rqt_graph"

echo "[setup] Environment ready. Aliases: uav_launch, uav_build, uav_save_map, uav_rosgraph"
