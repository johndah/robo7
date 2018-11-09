cmake_minimum_required(VERSION 2.8.3)
project(heuristic_grids)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robo7_msgs
  robo7_srvs
  cv_bridge
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs robo7_srvs cv_bridge
)

find_package(OpenCV REQUIRED)


include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)


add_executable(heuristic_grids_server src/heuristic_grids_server.cpp)

target_link_libraries(heuristic_grids_server
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

add_dependencies(heuristic_grids_server ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
