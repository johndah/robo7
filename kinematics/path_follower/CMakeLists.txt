cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robo7_msgs
  roscpp
  std_msgs
  sensor_msgs
  robo7_srvs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs geometry_msgs sensor_msgs robo7_srvs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(pure_rotation src/pure_rotation.cpp)
target_link_libraries(pure_rotation ${catkin_LIBRARIES})
add_dependencies(pure_rotation ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_straight src/move_straight.cpp)
target_link_libraries(move_straight ${catkin_LIBRARIES})
add_dependencies(move_straight ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_follower_v2 src/path_follower_v2.cpp)
target_link_libraries(path_follower_v2 ${catkin_LIBRARIES})
add_dependencies(path_follower_v2 ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(go_to src/go_to.cpp)
target_link_libraries(go_to ${catkin_LIBRARIES})
add_dependencies(go_to ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
