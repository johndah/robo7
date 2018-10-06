cmake_minimum_required(VERSION 2.8.3)
project(keyboard_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(keyboard_controller src/keyboard_controller.cpp)
target_link_libraries(keyboard_controller ${catkin_LIBRARIES})
add_dependencies(keyboard_controller keyboard_controller_cpp)
