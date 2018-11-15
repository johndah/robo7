# Vision

## Configuration

```c++
// install opencv for ros
$ sudo apt-get install ros-kinetic-vision-opencv
```

 if use ACF (aggregated channel feature) detector

1. Download pdollar matlab toolbox to ACF model

```
$ git clone https://github.com/pdollar/toolbox.git
```

2. Download the C++ implementation which can apply the trained model in ros, but need to build it on your PC

```c++
// download the c++ implementation
$ git clone https://github.com/elucideye/acf.git

// require gcc complier, CMake, Python, Polly
// add "PATH=$PATH:$HOME/polly/bin" to .bash_profile
$ source .bash_profile

// build the c++ implementation
$ export qvsJC2xEpIrp49xQ=[secure]
$ export TOOLCHAIN=gcc-5-pic-hid-sections-lto
$ export CONFIG=Release
$ export INSTALL=--strip
$ export TEST=--test
$ export GPU=OFF
$ export BUILD_SHARED=ON
$ export CXX=g++
$ export CC=gcc

$ source bin/hunter_env.sh

$ bin/travis_build.sh "${TOOLCHAIN}" "${CONFIG}" "${INSTALL}"
```

apt-get install libspdlog-dev


3. After building the C++ implementation, you can directly link the shared library in CMakeLists.txt

```
include_directories(
{directory of the .h file}
)

target_link_libraries({project_name}
{directory of the shared library}
)
```

Because C++ code need to be built by c++11 standardization, you need to enable c++11 in ros when building the project

```
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
```



## Dataset

On Oct 15 two video, **lab_maze.avi** and **lab_maze2.avi** were record.

use ffmpeg to extract images from videos

```
ffmpeg -i [video dir] -vf fps=10 %1d.png
```

another video **light.avi**

**Overall 14 classes of objects**

|                  |                    |                      |
| ---------------- | ------------------ | -------------------- |
| 0: Yellow Ball   | 1: Yellow Cube     |                      |
| 2: Green Cube    | 3: Green Cylinder  | 4: Green Hollow Cube |
| 5: Orange Cross  | 6: Orange Star     |                      |
| 7: Red Cylinder  | 8: Red Hollow Cube | 9: Red Ball          |
| 10: Blue Cube    | 11: Blue Triangle  |                      |
| 12: Purple Cross | 13: Purple Star    |                      |



### Data property

check the aspect ratio and size of different objects

![aspect_ratio_all](doc/aspect_ratio_all.jpg)



### Object pos

farthest visual distance 57 cm 

closest visual distance 5 cm



### Color

In opencv, the range of hsv

h: 0~180

s: 0~255

v: 0~255

 

## Log

| Time       | Done                                                         | To do                                         |
| ---------- | ------------------------------------------------------------ | --------------------------------------------- |
| 2018-11-13 | Integrated SVM color classifier into ros                     | 1. Select the best bbx                        |
|            |                                                              | 2. Purple and red objects detection bad       |
|            |                                                              | 3. Rosbag or save as a video                  |
| 2018-11-14 | Fix the problem why red and purple objects can not be detected. (BGR to RGB) | 1. Detect a image blurry or not               |
|            | Generate classify dataset                                    | 2. Extract the training dataset               |
|            |                                                              | 3. Split with training and validation dataset |
|            |                                                              |                                               |
|            |                                                              |                                               |

