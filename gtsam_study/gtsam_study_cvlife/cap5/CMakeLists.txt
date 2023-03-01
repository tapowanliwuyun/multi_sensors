cmake_minimum_required(VERSION 3.0)
project(ImuFactorExampleall)
set(CMAKE_CXX_STANDARD 11)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

include_directories("/usr/include/eigen3")

add_executable(ImuFactorExample2 src/ImuFactorExample2.cpp)
target_link_libraries(ImuFactorExample2 gtsam)

add_executable(ImuFactorsExample src/ImuFactorsExample.cpp)
target_link_libraries(ImuFactorsExample gtsam)