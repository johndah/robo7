cmake_minimum_required(VERSION 2.8.3)
project(robo7_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  PWM.msg
  BatteryStatus.msg
  Encoders.msg
  ServoMotors.msg
  WheelAngularVelocities.msg
  Odometry.msg
  ControllerParams.msg
  XY_coordinates.msg
  path.msg
  paths.msg
  trajectory_point.msg
  trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
  DEPENDS
)
