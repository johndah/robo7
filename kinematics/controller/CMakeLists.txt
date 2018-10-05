cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robo7_msgs
  std_msgs
)

#add_message_files(
#  FILES
#  motor_encoder.msg
#)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp robo7_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})
add_dependencies(motor_controller motor_controller_cpp)
