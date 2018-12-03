cmake_minimum_required(VERSION 2.8.3)
project(exploration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robo7_msgs
  robo7_srvs
  cv_bridge
  geometry_msgs
  visualization_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs robo7_srvs cv_bridge geometry_msgs visualization_msgs
)

find_package(OpenCV REQUIRED)


include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)


include(CheckCXXCompilerFlag)

check_cxx_compiler_flag(-std=c++11 HAS_STD_CPP11_FLAG)
if(HAS_STD_CPP11_FLAG)
  add_compile_options(-std=c++11)
endif()

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration ${catkin_LIBRARIES})
add_dependencies(exploration ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mapping_grids_server src/mapping_grids_server.cpp)
target_link_libraries(mapping_grids_server
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})
add_dependencies(mapping_grids_server ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(local_exploration src/local_exploration.cpp)
target_link_libraries(local_exploration ${catkin_LIBRARIES})
add_dependencies(local_exploration ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
