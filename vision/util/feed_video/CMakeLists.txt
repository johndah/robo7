cmake_minimum_required(VERSION 2.8.3)
project(feed_video)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS})

add_executable(feed_video src/feed_video.cpp)

target_link_libraries(feed_video 
${catkin_LIBRARIES} 
${OpenCV_LIBRARIES})

add_dependencies(feed_video ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

