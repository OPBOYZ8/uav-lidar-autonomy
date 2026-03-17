cmake_minimum_required(VERSION 3.0.2)
project(uav_lidar_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  xacro
)

catkin_package()

# Install launch files, config, URDF, and worlds
install(DIRECTORY launch config urdf worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
