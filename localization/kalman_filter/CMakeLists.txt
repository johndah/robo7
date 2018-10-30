cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  phidgets
  sensor_msgs
  robo7_msgs
  robo7_srvs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs phidgets robo7_msgs robo7_srvs sensor_msgs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(kalman_filter src/kalman_filter.cpp)
target_link_libraries(kalman_filter ${catkin_LIBRARIES})
add_dependencies(kalman_filter ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
