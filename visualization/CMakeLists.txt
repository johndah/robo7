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

add_executable(paths src/paths.cpp)
target_link_libraries(paths ${catkin_LIBRARIES})
add_dependencies(paths ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(laserXY src/laserXY.cpp)
target_link_libraries(laserXY ${catkin_LIBRARIES})
add_dependencies(laserXY ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lidar_plot src/lidar_plot.cpp)
target_link_libraries(lidar_plot ${catkin_LIBRARIES})
add_dependencies(lidar_plot ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dest_point_plot src/dest_point_plot.cpp)
target_link_libraries(dest_point_plot ${catkin_LIBRARIES})
add_dependencies(dest_point_plot ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wall_lidar_plot src/wall_lidar_plot.cpp)
target_link_libraries(wall_lidar_plot ${catkin_LIBRARIES})
add_dependencies(wall_lidar_plot ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(plot_map src/plot_map.cpp)
target_link_libraries(plot_map ${catkin_LIBRARIES})
add_dependencies(plot_map ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(plot_map_corners src/plot_map_corners.cpp)
target_link_libraries(plot_map_corners ${catkin_LIBRARIES})
add_dependencies(plot_map_corners ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(plot_lidar_corners src/plot_lidar_corners.cpp)
target_link_libraries(plot_lidar_corners ${catkin_LIBRARIES})
add_dependencies(plot_lidar_corners ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(plot_path_discretized src/plot_path_discretized.cpp)
target_link_libraries(plot_path_discretized ${catkin_LIBRARIES})
add_dependencies(plot_path_discretized ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(the_map_update_plot src/the_map_update_plot.cpp)
target_link_libraries(the_map_update_plot ${catkin_LIBRARIES})
add_dependencies(the_map_update_plot ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(occupancy src/occupancy.cpp)
target_link_libraries(occupancy ${catkin_LIBRARIES})
add_dependencies(occupancy ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(slam_visualization src/slam_visualization.cpp)
target_link_libraries(slam_visualization ${catkin_LIBRARIES})
add_dependencies(slam_visualization ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
