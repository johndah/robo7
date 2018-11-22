cmake_minimum_required(VERSION 2.8.3)
project(robo7_srvs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation geometry_msgs robo7_msgs sensor_msgs
)


add_service_files(
  FILES
  PickupAt.srv
  RansacWall.srv
  ICPAlgorithm.srv
  callServiceTest.srv
  IsGridOccupied.srv
  PathFollowerSrv.srv
  scanCoord.srv
  objectToRobot.srv
  path_planning.srv
  update_map.srv
  discretize_map.srv
  door_opener.srv
  distanceTo.srv
  GoTo.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  robo7_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs robo7_msgs sensor_msgs
  DEPENDS
)
