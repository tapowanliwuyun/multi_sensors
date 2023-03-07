cmake_minimum_required(VERSION 3.0)
project(CameraResectioning)
set(CMAKE_CXX_STANDARD 11)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

include_directories("/usr/local/include/eigen3")

add_executable(${PROJECT_NAME} CameraResectioning.cpp)
target_link_libraries(${PROJECT_NAME} gtsam)
