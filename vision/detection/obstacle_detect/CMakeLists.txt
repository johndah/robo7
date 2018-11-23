cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detect)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  roscpp
  robo7_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge geometry_msgs image_transport robo7_msgs
)

include_directories(include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS})

add_executable(obstacle_detect src/obstacle_detect.cpp)

target_link_libraries(obstacle_detect
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

add_dependencies(obstacle_detect ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
