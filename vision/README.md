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

