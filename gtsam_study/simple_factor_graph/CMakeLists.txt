cmake_minimum_required(VERSION 2.8)

project(simple_factor_graph)
add_compile_options(-std=c++11 )

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GTSAM REQUIRED QUIET)
link_directories(${GTSAM_LIBRARY_DIRS})
include_directories("/usr/include/eigen3")


find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)

add_executable(simple_factor_graph src/simple_factor_graph.cpp)
target_link_libraries(simple_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(LSM_factor_graph src/LSM_factor_graph.cpp)
target_link_libraries(LSM_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(loopclosure_factor_graph src/loopclosure_factor_graph.cpp)
target_link_libraries(loopclosure_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(sphere_factor_graph src/sphere_factor_graph.cpp)
target_link_libraries(sphere_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(symbol_factor_graph src/symbol_factor_graph.cpp)
target_link_libraries(symbol_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(express_factor_graph src/express_factor_graph.cpp)
target_link_libraries(express_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(camera_correct_factor_graph src/camera_correct_factor_graph.cpp)
target_link_libraries(camera_correct_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(sfm_factor_graph src/sfm_factor_graph.cpp)
target_link_libraries(sfm_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread "/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2")

add_executable(kalman_factor_graph src/kalman_factor_graph.cpp)
target_link_libraries(kalman_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(imu_factor_graph src/imu_factor_graph.cpp)
target_link_libraries(imu_factor_graph ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread)

add_executable(ISAM1_example1 src/ISAM1_example1.cpp)
target_link_libraries(ISAM1_example1 ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread "/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2")

add_executable(ISAM2_example1 src/ISAM2_example1.cpp)
target_link_libraries(ISAM2_example1 ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread "/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2")

add_executable(ISAM2_example2 src/ISAM2_example2.cpp)
target_link_libraries(ISAM2_example2 ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread "/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2")

add_executable(ISAM2_imu_camera_correct src/ISAM2_imu_camera_correct.cpp)
target_link_libraries(ISAM2_imu_camera_correct ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} -ltbb -lgtsam -pthread "/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2")