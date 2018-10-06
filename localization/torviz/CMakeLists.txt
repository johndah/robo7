cmake_minimum_required(VERSION 2.8.3)
project(torviz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  nav_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs tf nav_msgs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(torviz src/torviz.cpp)
target_link_libraries(torviz ${catkin_LIBRARIES})
add_dependencies(torviz torviz_cpp)

add_executable(robotrviz src/robotrviz.cpp)
target_link_libraries(robotrviz ${catkin_LIBRARIES})
add_dependencies(robotrviz robotrviz_cpp)
