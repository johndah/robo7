cmake_minimum_required(VERSION 2.8.3)
project(pathplanning)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robo7_msgs
  robo7_srvs
  geometry_msgs
  visualization_msgs
  phidgets
)


catkin_package(
 CATKIN_DEPENDS roscpp std_msgs robo7_msgs robo7_srvs geometry_msgs visualization_msgs phidgets
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag(-std=c++11 HAS_STD_CPP11_FLAG)
if(HAS_STD_CPP11_FLAG)
  add_compile_options(-std=c++11)
endif()

add_executable(pathplanning src/pathplanning.cpp)
target_link_libraries(pathplanning ${catkin_LIBRARIES})
add_dependencies(pathplanning ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
