cmake_minimum_required(VERSION 2.8.3)
project(speaker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(speaker src/speaker.cpp)

target_link_libraries(speaker
${catkin_LIBRARIES}
)

add_dependencies(speaker ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
