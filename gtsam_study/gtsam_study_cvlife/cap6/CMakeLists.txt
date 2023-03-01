cmake_minimum_required(VERSION 3.0)
project(cap6)
set(CMAKE_CXX_STANDARD 11)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

include_directories("/usr/include/eigen3")

add_executable(LocalizationExample src/LocalizationExample.cpp)
target_link_libraries(LocalizationExample gtsam)

add_executable(PlanarSLAMExample src/PlanarSLAMExample.cpp)
target_link_libraries(PlanarSLAMExample gtsam)

add_executable(my_gtsam_example src/my_gtsam_example.cpp)
target_link_libraries(my_gtsam_example gtsam)
