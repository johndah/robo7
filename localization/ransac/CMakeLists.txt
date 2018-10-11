cmake_minimum_required(VERSION 2.8.3)
project(ransac)

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


add_executable(scan_to_coordinates src/scan_to_coordinates.cpp)
target_link_libraries(scan_to_coordinates ${catkin_LIBRARIES})
add_dependencies(scan_to_coordinates ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
