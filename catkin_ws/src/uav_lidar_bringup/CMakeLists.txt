cmake_minimum_required(VERSION 3.0.2)
project(uav_lidar_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
