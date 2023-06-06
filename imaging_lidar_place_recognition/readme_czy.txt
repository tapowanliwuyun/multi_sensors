1. 编译

cd ~/imaging_lidar_place_recognition/src
git clone https://github.com/TixiaoShan/imaging_lidar_place_recognition.git
cd ..
catkin_make

1.1 编译报错

报错：
CMake Error at imaging_lidar_place_recognition/CMakeLists.txt:34 (find_package):
  By not providing "FindDBoW3.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "DBoW3", but
  CMake did not find one.

  Could not find a package configuration file provided by "DBoW3" with any of
  the following names:

    DBoW3Config.cmake
    dbow3-config.cmake

  Add the installation prefix of "DBoW3" to CMAKE_PREFIX_PATH or set
  "DBoW3_DIR" to a directory containing one of the above files.  If "DBoW3"
  provides a separate development package or SDK, be sure it has been
  installed.

解决方法：
已经下载了DBoW3，但是还是不行，可以参考：https://github.com/rmsalinas/DBow3
修改CMakeLists.txt

注释 find_package(DBoW3 REQUIRED)
增加 set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )

修改 target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${DBoW3_LIBS} ${OpenMP_CXX_FLAGS})



2. 运行
roslaunch imaging_lidar_place_recognition run.launch
rosbag play ../../bag_file/intensity/indoor_registered-001/indoor_registered.bag -r 0.5


2.1 运行报错
问题：
OpenCV Error: Bad argument (Unknown interpolation method) in resize, file /build/opencv-L2vuMj/opencv-3.2.0+dfsg/modules/imgproc/src/imgwarp.cpp, line 3367
OpenCV Error: Bad argument (Unknown interpolation method) in resize, file /build/opencv-L2vuMj/opencv-3.2.0+dfsg/modules/imgproc/src/imgwarp.cpp, line 3367
terminate called after throwing an instance of 'cv::Exception'
terminate called recursively
[imaging_lidar_place_recognition_main-2] process has died [pid 7004, exit code -6, cmd /home/bupo/my_study/imaging_lidar_place_recognition/devel/lib/imaging_lidar_place_recognition/imaging_lidar_place_recognition_main __name:=imaging_lidar_place_recognition_main __log:=/home/bupo/.ros/log/46f1813e-d84f-11ed-a9f9-02421a74df96/imaging_lidar_place_recognition_main-2.log].
log file: /home/bupo/.ros/log/46f1813e-d84f-11ed-a9f9-02421a74df96/imaging_lidar_place_recognition_main-2*.log


分析：通过打标确定代码错误是在 keyframe.cpp 文件的 void KeyFrame::computeWindowOrbPoint() 函数下 的  detector->detect(image_intensity, orb_window_keypoints, MASK);

经过查找资料确定就是  cv_bridge 的问题 ，cvbridge错误是因为ros里自带的OpenCV版本和你电脑的 不一致引起的

解决：


参考：https://blog.csdn.net/qq_36814762/article/details/110230127（通过该方法解决）3.最后尝试重新找cv_bridge的包进行编译

