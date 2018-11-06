cmake_minimum_required(VERSION 2.8.3)
project(classify)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge  
)

include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

