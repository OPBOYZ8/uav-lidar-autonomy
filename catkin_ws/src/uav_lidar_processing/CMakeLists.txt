cmake_minimum_required(VERSION 3.0.2)
project(uav_lidar_processing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

catkin_install_python(PROGRAMS
  src/lidar_processor_node.py
  src/drone_waypoint_controller.py
  src/uav_navigator.py
  src/obstacle_map_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
