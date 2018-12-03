cmake_minimum_required(VERSION 2.8.3)
project(brain)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  robo7_msgs
  robo7_srvs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs robo7_msgs robo7_srvs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)


add_executable(brain src/brain.cpp)

target_link_libraries(brain
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

add_dependencies(brain ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
