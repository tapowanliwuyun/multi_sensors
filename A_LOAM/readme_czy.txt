1.编译
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
2.运行
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    rosbag play nsh_indoor_outdoor.bag
