#!/bin/bash
# =============================================================================
# UAV LiDAR Autonomy — Dependency Installer
# =============================================================================
# Installs all ROS Noetic packages and Python libraries required by the project.
# Run once after cloning, before building.
#
# Usage:
#   chmod +x scripts/install_deps.sh
#   ./scripts/install_deps.sh

set -e

echo "=================================================="
echo "  UAV LiDAR Autonomy — Install Dependencies"
echo "=================================================="

# ---------------------------------------------------------------------------
# Verify ROS Noetic is present
# ---------------------------------------------------------------------------
if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "[ERROR] ROS Noetic not found at /opt/ros/noetic."
    echo "        See docs/setup_wsl2.md for full installation instructions."
    exit 1
fi

echo "[info] ROS Noetic found."

# ---------------------------------------------------------------------------
# ROS packages
# ---------------------------------------------------------------------------
echo ""
echo "[apt] Installing ROS Noetic packages..."

sudo apt-get install -y \
    ros-noetic-velodyne-simulator \
    ros-noetic-velodyne-description \
    ros-noetic-velodyne-gazebo-plugins \
    ros-noetic-velodyne-laserscan \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-hector-slam \
    ros-noetic-hector-mapping \
    ros-noetic-tf2-tools \
    ros-noetic-rqt-graph \
    ros-noetic-teleop-twist-keyboard \
    python3-catkin-tools

echo "[apt] ROS packages installed."

# ---------------------------------------------------------------------------
# Python packages
# ---------------------------------------------------------------------------
echo ""
echo "[pip] Installing Python packages..."

pip3 install --upgrade scipy numpy

echo "[pip] Python packages installed."

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "=================================================="
echo "  Dependencies installed successfully."
echo "  Next step: ./scripts/build.sh"
echo "=================================================="
