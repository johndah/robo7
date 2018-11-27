cmake_minimum_required(VERSION 2.8.3)
project(classify)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  classifiedObj.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs cv_bridge robo7_msgs message_runtime
)

include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

