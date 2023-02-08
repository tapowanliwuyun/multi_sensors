cmake_minimum_required(VERSION 3.0)
project(curve_fitting)
set(CMAKE_CXX_STANDARD 11)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

include_directories("/usr/include/eigen3")

add_executable(${PROJECT_NAME} src/curve_fitting.cpp)
target_link_libraries(${PROJECT_NAME} gtsam)
