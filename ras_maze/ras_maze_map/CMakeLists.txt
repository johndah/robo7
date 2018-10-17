cmake_minimum_required(VERSION 2.8.3)
project(own_map)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs kdl_conversions robo7_msgs geometry_msgs sensor_msgs)
find_package(Boost REQUIRED COMPONENTS random)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs robo7_msgs geometry_msgs sensor_msgs
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(own_map src/own_map.cpp)
target_link_libraries(own_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS own_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
