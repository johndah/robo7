cmake_minimum_required(VERSION 2.8.3)
project(map_maintenance)

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

add_executable(map_maintenance src/map_maintenance.cpp)
target_link_libraries(map_maintenance ${catkin_LIBRARIES})
add_dependencies(map_maintenance ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
