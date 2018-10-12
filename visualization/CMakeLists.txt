cmake_minimum_required(VERSION 2.8.3)
project(visualization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  nav_msgs
  sensor_msgs
  robo7_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs tf nav_msgs sensor_msgs robo7_msgs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(visualization src/visualization.cpp)
target_link_libraries(visualization ${catkin_LIBRARIES})
add_dependencies(visualization ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(laserXY src/laserXY.cpp)
target_link_libraries(laserXY ${catkin_LIBRARIES})
add_dependencies(laserXY ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lidar_plot src/lidar_plot.cpp)
target_link_libraries(lidar_plot ${catkin_LIBRARIES})
add_dependencies(lidar_plot ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
