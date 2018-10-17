cmake_minimum_required(VERSION 2.8.3)
project(gate_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  robo7_srvs
  arduino_servo_control
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs robo7_srvs arduino_servo_control
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(gate_controller src/gate_controller.cpp)
target_link_libraries(gate_controller ${catkin_LIBRARIES})
add_dependencies(gate_controller ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
