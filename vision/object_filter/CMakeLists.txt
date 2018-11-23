cmake_minimum_required(VERSION 2.8.3)
project(object_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robo7_msgs
  robo7_srvs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs robo7_srvs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(object_filter src/object_filter.cpp)

target_link_libraries(object_filter
${catkin_LIBRARIES}
)

add_dependencies(object_filter ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
