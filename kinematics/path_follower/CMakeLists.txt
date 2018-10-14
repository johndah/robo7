cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robo7_msgs
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs geometry_msgs sensor_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(breaker src/breaker.cpp)
target_link_libraries(breaker ${catkin_LIBRARIES})
add_dependencies(breaker ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_follower src/path_follower.cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})
add_dependencies(path_follower ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(destination_point src/destination_point.cpp)
target_link_libraries(destination_point ${catkin_LIBRARIES})
add_dependencies(destination_point ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
