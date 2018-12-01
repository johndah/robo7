cmake_minimum_required(VERSION 2.8.3)
project(object_saver)

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

add_executable(object_saver src/object_saver.cpp)

target_link_libraries(object_saver
${catkin_LIBRARIES}
)

add_dependencies(object_saver ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
