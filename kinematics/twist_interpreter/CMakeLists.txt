cmake_minimum_required(VERSION 2.8.3)
project(twist_interpreter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robo7_msgs
  roscpp
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs geometry_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(twist_interpreter src/twist_interpreter.cpp)
target_link_libraries(twist_interpreter ${catkin_LIBRARIES})
