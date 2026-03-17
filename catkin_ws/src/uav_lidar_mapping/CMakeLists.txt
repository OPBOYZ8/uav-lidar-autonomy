cmake_minimum_required(VERSION 3.0.2)
project(uav_lidar_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
)

catkin_package()

install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
