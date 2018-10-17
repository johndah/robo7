cmake_minimum_required(VERSION 2.8.3)
project(dead_reckoning)

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


add_executable(dead_reckoning src/dead_reckoning.cpp)
target_link_libraries(dead_reckoning ${catkin_LIBRARIES})
add_dependencies(dead_reckoning ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
