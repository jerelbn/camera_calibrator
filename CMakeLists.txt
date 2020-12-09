cmake_minimum_required(VERSION 3.10)
project(camera_calibrator)

# C++ parameters
set(CMAKE_CXX_STANDARD 14)

# Default to release build
if(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RELEASE)
endif()

# Find other packages
find_package(OpenCV REQUIRED)

## Include dirs
include_directories(include
    ${OpenCV_INCLUDE_DIRS})

## Build
add_executable(camera_calibrator
    src/main.cpp)
target_link_libraries(camera_calibrator
    ${OpenCV_LIBRARIES})