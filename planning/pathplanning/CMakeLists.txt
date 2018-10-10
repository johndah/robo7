cmake_minimum_required(VERSION 2.8.3)
project(planning)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robo7_msgs
  geometry_msgs
  phidgets
)


catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs phidgets
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pathplanning src/pathplanning.cpp)
target_link_libraries(pathplanning ${catkin_LIBRARIES})
add_dependencies(pathplanning ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
