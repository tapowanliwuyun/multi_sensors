这是r3live的安装方式
1. 安装依赖 
sudo apt-get install ros-melodic-cv-bridge ros-melodic-tf ros-melodic-message-filters ros-melodic-image-transport ros-melodic-image-transport*

2. 安装livox驱动程序
2.1 安装Livox-SDK
//https://gitcode.net/mirrors/Livox-SDK/Livox-SDK?utm_source=csdn_github_accelerator
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install

2.2 安装livox-ros-driver
##不用这个，解释看下面的
##git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src


git clone https://github.com/ziv-lin/livox_ros_driver_for_R2LIVE.git ws_livox/src

cd ws_livox
catkin_make
//使用如下命令更新当前 ROS 包环境
source ./devel/setup.sh

注意：作者在readme中说了，因为Livox-ros-driver官方发布的LiDAR数据和IMU数据都带有LiDAR的时间戳(每次记录都从0开始)，而图像的时间戳通常是用操作系统的时间戳记录的。为了使它们在相同的基于时间的情况下工作，作者修改了Livox-ros-driver的源代码，所以作者新的driver驱动包可以从livox_ros_driver_for_R2LIVE获得。

后面我也该了这个，由于我这个realsensel515和livox-hap都N100的imu是从操作系统的时间开始的，所以没必要修改

如果是为R3LIVE采集并运行自己的数据需要用它替换官方驱动程序。
原文链接：https://blog.csdn.net/handily_1/article/details/122271243


驱动程序中所有的 launch 文件都位于 “ws_livox/src/livox_ros_driver/launch” 路径下，具体可参考上文

2.3 选择性安装CGAL and pcl_viewer（我没装）

sudo apt-get install libcgal-dev pcl-tools

2.4 opencv 3.4.1 
2.5 pcl
2.6 eigen3
2.7 ceres 2.0.0

安装完后查看Ceres版本：
因为Ceres库版本的定义在 /usr/local/include/ceres/version.h 里面，所以

sudo cat /usr/local/include/ceres/version.h 

2.8 

3.编译3live

创建ros工作空间
mkdir -p r3live_ws/src 
cd r3live_ws/src

克隆代码：
git clone https://github.com/hku-mars/r3live.git

添加 livox_driver 环境变量：
cd ../
source livox_driver_for_r2live/devel/setup.bash 

编译：
catkin_make
source ~/catkin_ws/devel/setup.bash




4. 报错解决

4.1 报错一
CMake Error at r3live/r3live/CMakeLists.txt:125 (FIND_PACKAGE):
  By not providing "FindCGAL.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "CGAL", but
  CMake did not find one.

  Could not find a package configuration file provided by "CGAL" with any of
  the following names:

    CGALConfig.cmake
    cgal-config.cmake

  Add the installation prefix of "CGAL" to CMAKE_PREFIX_PATH or set
  "CGAL_DIR" to a directory containing one of the above files.  If "CGAL"
  provides a separate development package or SDK, be sure it has been
  installed.

解决：

sudo apt-get install libcgal-dev

5. 运行
5.1 点云运行
source devel/setup.bash
roslaunch r3live r3live_bag.launch
rosbag play YOUR_DOWNLOADED.bag

R3LIVE可以在任何时候保存你创建的地图。你只需要把鼠标移动到Control panel那个图标上，点击并按“S”键即可保存pcd地图，离线地图会保存在${HOME}/r3live_output文件夹下，在launch 文件中可以修改地图保存路径。

可以看出，R3LIVE的纹理效果非常好，地图也很稠密，随着地图越建越大，电脑内存也越占越大，我的电脑配置cpu是r7 3700x，gpu是1080TI，内存是16G。刚开始跑内存占50%左右，跑到最后内存会占用90%左右，但我swap空间分配了100个G，不知道是不是这个原因反正运行起来一点也不卡，很丝滑（狗头🐶）。

5.2mesh重建和纹理贴图
作者最后还提供了一个很好的工具，用于mesh化和贴图的工具，可以导出一些通用格式的文件，如“pcd”, “ply”, “obj”等。当把离线地图保存在磁盘上后(默认保存在目录:${HOME}/r3live output)，可以启动作者提供的工具来重建和纹理mesh。
roslaunch r3live r3live_reconstruct_mesh.launch


n. 参考
https://blog.csdn.net/handily_1/article/details/122271243




