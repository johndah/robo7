cmake_minimum_required(VERSION 2.8.3)
project(deadreckogning)

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


add_executable(deadreckogning src/deadreckogning.cpp)
target_link_libraries(deadreckogning ${catkin_LIBRARIES})
add_dependencies(deadreckogning deadreckogning_cpp)
