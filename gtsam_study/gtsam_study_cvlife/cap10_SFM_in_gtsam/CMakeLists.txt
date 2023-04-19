cmake_minimum_required(VERSION 3.0)
project(cap10)
set(CMAKE_CXX_STANDARD 14)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

include_directories("/usr/include/eigen3")

add_executable(SFMExample src/SFMExample.cpp)
target_link_libraries(SFMExample gtsam)

add_executable(SFMExampleExpressions src/SFMExampleExpressions.cpp)
target_link_libraries(SFMExampleExpressions gtsam)

add_executable(SFMExample_SmartFactor src/SFMExample_SmartFactor.cpp)
target_link_libraries(SFMExample_SmartFactor gtsam)

add_executable(SFMExample_SmartFactorPCG src/SFMExample_SmartFactorPCG.cpp)
target_link_libraries(SFMExample_SmartFactorPCG gtsam)

add_executable(SFMExample_bal src/SFMExample_bal.cpp)
target_link_libraries(SFMExample_bal gtsam)

add_executable(SFMExampleExpressions_bal src/SFMExampleExpressions_bal.cpp)
target_link_libraries(SFMExampleExpressions_bal gtsam)

add_executable(SFMExample_bal_COLAMD_METIS src/SFMExample_bal_COLAMD_METIS.cpp)
target_link_libraries(SFMExample_bal_COLAMD_METIS gtsam)

