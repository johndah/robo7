cmake_minimum_required(VERSION 2.8.3)
project(visualize)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  nav_msgs
  sensor_msgs
  robo7_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs tf nav_msgs sensor_msgs robo7_msgs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(obj_pos_vis src/obj_pos_vis.cpp)
target_link_libraries(obj_pos_vis ${catkin_LIBRARIES})
add_dependencies(obj_pos_vis ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
