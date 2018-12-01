cmake_minimum_required(VERSION 2.8.3)
project(path_testing)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robo7_msgs
  robo7_srvs
  geometry_msgs
  visualization_msgs
  phidgets
)


catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs robo7_srvs geometry_msgs visualization_msgs phidgets
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_testing src/path_testing.cpp)
target_link_libraries(path_testing ${catkin_LIBRARIES})
add_dependencies(path_testing ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_exploration_test src/path_exploration_test.cpp)
target_link_libraries(path_exploration_test ${catkin_LIBRARIES})
add_dependencies(path_exploration_test ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
