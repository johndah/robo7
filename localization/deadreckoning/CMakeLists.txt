cmake_minimum_required(VERSION 2.8.3)
project(deadreckoning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  phidgets
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs phidgets
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(deadreckoning src/deadreckoning.cpp)
target_link_libraries(deadreckoning ${catkin_LIBRARIES})
add_dependencies(deadreckoning ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
