cmake_minimum_required(VERSION 2.8.3)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  robo7_msgs
  phidgets
  pcl_conversions
  pcl_ros
  robo7_srvs
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs robo7_msgs phidgets pcl_conversions pcl_ros robo7_srvs sensor_msgs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(update_map src/update_map.cpp)
target_link_libraries(update_map ${catkin_LIBRARIES})
add_dependencies(update_map ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(update_occupancy_grid src/update_occupancy_grid.cpp)
target_link_libraries(update_occupancy_grid ${catkin_LIBRARIES})
add_dependencies(update_occupancy_grid ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
