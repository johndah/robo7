cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  phidgets
  sensor_msgs
  robo7_msgs
  kdl_conversions
  pcl_ros
)
find_package(Boost REQUIRED COMPONENTS random)

catkin_package(
 DEPENDS
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs phidgets sensor_msgs robo7_msgs pcl_ros
 INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
