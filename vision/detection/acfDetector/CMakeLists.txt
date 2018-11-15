cmake_minimum_required(VERSION 2.8.3)
project(acfDetector)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  roscpp
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  detectedObj.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge geometry_msgs image_transport message_runtime
)

include_directories(include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS})

# /home/jtao/robo7_vision/ACF_detection/acf/_install/gcc-5-pic-hid-sections-lto/include)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_executable(acf_detector src/acf_detector.cpp)

target_link_libraries(acf_detector
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
/home/ras17/acf/_install/gcc-5-pic-hid-sections-lto/lib/libacf.so)

# /home/ras17/acf/_install/gcc-5-pic-hid-sections-lto/lib/libacf.so
# /home/jtao/robo7_vision/ACF_detection/acf/_install/gcc-5-pic-hid-sections-lto/lib/libacf.so

add_dependencies(acf_detector ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(multi_acf_detector src/multi_acf_detector.cpp)

target_link_libraries(multi_acf_detector
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
/home/ras17/acf/_install/gcc-5-pic-hid-sections-lto/lib/libacf.so)

add_dependencies(multi_acf_detector ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(trainSVM src/trainSVM.cpp)

target_link_libraries(trainSVM
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

add_dependencies(trainSVM ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
