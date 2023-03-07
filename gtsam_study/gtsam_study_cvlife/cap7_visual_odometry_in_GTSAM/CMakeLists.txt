cmake_minimum_required(VERSION 3.0)
project(cap7)
set(CMAKE_CXX_STANDARD 14)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

include_directories("/usr/include/eigen3")

add_executable(CameraResectioning src/CameraResectioning.cpp)
target_link_libraries(CameraResectioning gtsam)

add_executable(StereoVOExample src/StereoVOExample.cpp)
target_link_libraries(StereoVOExample gtsam)
