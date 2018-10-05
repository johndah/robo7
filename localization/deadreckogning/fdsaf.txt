cmake_minimum_required(VERSION 2.8.3)
project(dead_reckogning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  phidgets
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs phidgets
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(dead_reckogning src/dead_reckogning.cpp)
target_link_libraries(dead_reckogning ${catkin_LIBRARIES})
add_dependencies(dead_reckogning dead_reckogning_cpp)
