cmake_minimum_required(VERSION 2.8.3)
project(imaging_lidar_place_recognition)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE " -O3 -Wall -g -pthread")

######################
### Packages
######################
#set(cv_bridge_1_DIR /usr/local/include/cv_bridge_1)
set(OpenCV_DIR "/home/bupo/anzhuang/opencv-3.4.1/build")
set(cv_bridge_DIR /home/bupo/cv_bridge_ws/devel/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    roslib
    # msg
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    # cv
    cv_bridge
    #cv_bridge_1
    # pcl
    pcl_conversions
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(DBoW3 REQUIRED)


#set(OpenCV_DIR /usr/local/share/Opencv)
######################
### Catkin
######################
catkin_package(
    DEPENDS PCL
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${DBoW3_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    ${DBoW3_LIBRARY_DIRS}
)

######################
### visual odometry
######################

file(GLOB source_files
    "src/*.cpp"
    "src/ThirdParty/DUtils/*.cpp"
    "src/ThirdParty/DVision/*.cpp"
)
# Visual Lopp
add_executable(${PROJECT_NAME}_main ${source_files})
target_compile_options(${PROJECT_NAME}_main PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${DBoW3_LIBRARIES} ${OpenMP_CXX_FLAGS})
