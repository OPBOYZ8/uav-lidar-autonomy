#!/bin/bash
# =============================================================================
# UAV LiDAR Autonomy — Pipeline Validation
# =============================================================================
# Checks that all expected ROS topics are live after launching the simulation.
# Run in a separate terminal while the simulation is already running.
#
# Usage:
#   source scripts/setup.sh
#   bash scripts/validate_pipeline.sh

set -e

echo "=================================================="
echo "  UAV LiDAR Autonomy — Pipeline Validation"
echo "=================================================="

source /opt/ros/noetic/setup.bash
WS_ROOT="$(cd "$(dirname "$0")/../catkin_ws" && pwd)"
if [ -f "$WS_ROOT/devel/setup.bash" ]; then
    source "$WS_ROOT/devel/setup.bash"
fi

PASS=0
FAIL=0
TIMEOUT=8   # seconds to wait for a single message

check_topic() {
    local topic="$1"
    local expected_type="$2"
    printf "  %-38s ... " "$topic"

    if timeout "$TIMEOUT" rostopic echo "$topic" --n 1 > /dev/null 2>&1; then
        actual_type=$(rostopic type "$topic" 2>/dev/null || echo "unknown")
        if [ -z "$expected_type" ] || [ "$actual_type" = "$expected_type" ]; then
            echo "PASS  [$actual_type]"
        else
            echo "WARN  [expected=$expected_type  got=$actual_type]"
        fi
        PASS=$((PASS + 1))
    else
        echo "FAIL  (no message in ${TIMEOUT}s)"
        FAIL=$((FAIL + 1))
    fi
}

echo ""
echo "--- Simulation topics ---"
check_topic "/velodyne_points"       "sensor_msgs/PointCloud2"
check_topic "/ground_truth/odom"     "nav_msgs/Odometry"
check_topic "/tf"                    ""

echo ""
echo "--- LiDAR processing ---"
check_topic "/cloud_filtered"        "sensor_msgs/PointCloud2"
check_topic "/scan"                  "sensor_msgs/LaserScan"

echo ""
echo "--- Occupancy mapping ---"
check_topic "/map"                   "nav_msgs/OccupancyGrid"

echo ""
echo "--- Navigation ---"
check_topic "/uav/pose"              "geometry_msgs/PoseStamped"
check_topic "/uav/nav_debug"         "std_msgs/String"
check_topic "/uav/path"              "nav_msgs/Path"

echo ""
echo "=================================================="
printf "  Results: %d PASSED  %d FAILED\n" "$PASS" "$FAIL"
echo "=================================================="

if [ "$FAIL" -eq 0 ]; then
    echo "  ALL CHECKS PASSED — pipeline is fully operational."
    exit 0
else
    echo "  $FAIL check(s) failed."
    echo "  Check running nodes with:   rosnode list"
    echo "  Check node logs with:       rosnode info <node_name>"
    exit 1
fi
