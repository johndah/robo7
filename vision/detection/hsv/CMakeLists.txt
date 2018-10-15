cmake_minimum_required(VERSION 2.8.3)
project(hsv)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge geometry_msgs image_transport
)

include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS})

add_executable(adjust_threshold src/adjust_threshold.cpp)

target_link_libraries(adjust_threshold 
${catkin_LIBRARIES} 
${OpenCV_LIBRARIES})

add_dependencies(adjust_threshold ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

add_executable(color_detect src/color_detect.cpp)

target_link_libraries(color_detect 
${catkin_LIBRARIES} 
${OpenCV_LIBRARIES})

add_dependencies(color_detect ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

