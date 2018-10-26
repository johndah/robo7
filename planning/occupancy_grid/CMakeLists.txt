cmake_minimum_required(VERSION 2.8.3)
project(occupancy_grid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robo7_msgs
  robo7_srvs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs robo7_srvs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(gate_controller src/occupancy_grid_server.cpp)
target_link_libraries(gate_controller ${catkin_LIBRARIES})
add_dependencies(gate_controller ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
