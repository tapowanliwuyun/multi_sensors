1. 安装
1.1 安装 livox_ros_driver

git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
cd ws_livox
catkin_make

1.1 s-fast-lio

cd s_fast_lio/s_fast_lio/src
git clone https://github.com/zlwang7/S-FAST_LIO.git
cd ../
source ../ws_livox/devel/setup.bash
catkin_make
source s_fast_lio/s_fast_lio/devel/setup.bash

1.2 对于不同版本sophous
s-fast-lio使用应该是非模板类，但是我用的是模板类，所以会报错缺少
fatal error: sophus/so3.h: 没有那个文件或目录
 #include "sophus/so3.h"
把，so3.h修改为so3.hpp，然后在所有的 Sophus::SO3 修改为 Sophus::SO3d 即可
按照报错进行一个一个修改即可

2. 运行


