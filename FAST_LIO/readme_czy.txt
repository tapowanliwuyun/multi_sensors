1. 编译
1.1 安装 livox_ros_driver

git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
cd ws_livox
catkin_make

2. 安装fast-lio
mkdir -p FAST_LIO/src
cd FAST_LIO/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
source ../ws_livox/devel/setup.bash
catkin_make
source devel/setup.bash

3. 运行
3.1 运行 livox Aia Rosbag

roslaunch fast_lio mapping_avia.launch
//rosbag play YOUR_DOWNLOADED.bag

cd ~/bag_file/avia/fast-lio/
rosbag play 2020-09-16-quick-shack.bag
