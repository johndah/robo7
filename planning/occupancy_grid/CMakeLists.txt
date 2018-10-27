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

add_executable(occupancy_grid_server src/occupancy_grid_server.cpp)
target_link_libraries(occupancy_grid_server ${catkin_LIBRARIES})
add_dependencies(occupancy_grid_server ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
