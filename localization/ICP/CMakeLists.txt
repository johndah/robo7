cmake_minimum_required(VERSION 2.8.3)
project(icp)

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

add_executable(ICP_test src/ICP_test.cpp)
target_link_libraries(ICP_test ${catkin_LIBRARIES})
add_dependencies(ICP_test ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(icp src/ICP.cpp)
target_link_libraries(icp ${catkin_LIBRARIES})
add_dependencies(icp ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
