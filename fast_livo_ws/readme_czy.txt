1. 编译

1）首先安装livox的驱动，这个就不过多描述了，catkin_make 就行了，然后记得编译文件的时候  source 一下

2）安装
Ubuntu 16.04~20.04. ROS Installation.

PCL>=1.6, Follow PCL Installation.

Eigen>=3.3.4, Follow Eigen Installation.

OpenCV>=3.2, Follow Opencv Installation.

我的电脑是
ubuntu 18.06 melodic
pcl1.8
eigen3.3.4
opencv3.4.1 

3）安装sophus

这个要安装 非模板类的sophus
如果你的电脑没有安装过sophus，那么你可以按照教程直接安装
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install


但是一般情况下，现在如果代码框架都是模板类的sophus，也就意味着你需要安装两个版本的sophus，那么你就可以参考：https://blog.csdn.net/fb_941219/article/details/104590842
但是，尚上面文章是假设 先安装了 非模板类然后安装了 模板类  ，可是我自己的电脑是 先安装了模板类，然后需要为了fast-livo安装非模板类

那么在进行
mkdir build
cd build
cmake -D CMAKE_INSTALL_PREFIX=/usr/local/sophus-template ..
make -j8
sudo make install


4）vikit

cd fast_livo_ws/src
git clone https://github.com/uzh-rpg/rpg_vikit.git

5)
cd fast_livo_ws/src
git clone https://github.com/hku-mars/FAST-LIVO
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash


然后你的sophus是和我一样的情况，先安装了模板类，然后需要为了fast-livo安装非模板类，进行了安装。那么就需要进行下面步骤之后在编译
之后，就需要对 CmakeLists 文件进行配置，这里需要对三个 CmakeLists 配置
a）FAST-LIVO
添加
set(Sophus_INCLUDE_DIR "/usr/local/sophus-template/include")

b）rpg_vikit/vikit_common
set(Sophus_INCLUDE_DIR "/usr/local/sophus-template/include")
set(Sophus_LIBRARIES "/usr/local/sophus-template/lib/libSophus.so")

c)rpg_vikit/vikit_ros
set(Sophus_INCLUDE_DIR "/usr/local/sophus-template/include")


2 运行
roslaunch fast_livo mapping_avia.launch


