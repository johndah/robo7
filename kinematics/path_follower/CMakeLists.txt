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


add_executable(breaker src/breaker.cpp)
target_link_libraries(breaker ${catkin_LIBRARIES})
add_dependencies(breaker ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(point_follower src/point_follower.cpp)
target_link_libraries(point_follower ${catkin_LIBRARIES})
add_dependencies(point_follower ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(destination_point src/destination_point.cpp)
target_link_libraries(destination_point ${catkin_LIBRARIES})
add_dependencies(destination_point ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_follower src/path_follower.cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})
add_dependencies(path_follower ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(go_to src/go_to.cpp)
target_link_libraries(go_to ${catkin_LIBRARIES})
add_dependencies(go_to ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pure_rotation src/pure_rotation.cpp)
target_link_libraries(pure_rotation ${catkin_LIBRARIES})
add_dependencies(pure_rotation ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_straight src/move_straight.cpp)
target_link_libraries(move_straight ${catkin_LIBRARIES})
add_dependencies(move_straight ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
