cmake_minimum_required(VERSION 2.8.3)
project(test_service)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  robo7_msgs
  phidgets
  pcl_conversions
  pcl_ros
  robo7_srvs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs robo7_msgs phidgets pcl_conversions pcl_ros robo7_srvs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(test_service src/test_service.cpp)
target_link_libraries(test_service ${catkin_LIBRARIES})
add_dependencies(test_service ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
