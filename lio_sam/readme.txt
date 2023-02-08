1. 安装依赖
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-robot-state-publisher

以及 gstam

2. 安装

cd ~/lio_sam/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd ..
catkin_make

3. 运行
source devel/setup.bash
roslaunch lio_sam run.launch

rosbag play park_dataset.bag
