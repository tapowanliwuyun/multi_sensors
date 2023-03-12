
[toc]

# 1 安装与使用
## 1.1 编译
```bash
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 1.2 运行
```cpp
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    rosbag play nsh_indoor_outdoor.bag
```
# 2 代码流程介绍
## 2.1 scanRegistration
scanRegistration的功能就是提取特征点。

1. [main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L568)
接受来自cmake文件的部分参数，订阅bag包中发布的初始点云数据，经回调函数处理后发布话题：有序点云（删除过近点、设置索引），极大边线点集合，次极大边线点集合，极小平面点集合，次极小平面点集合,删除的点云

2. [main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L568)->[laserCloudHandler](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L120)

传入初始点云，发布话题：有序点云（删除过近点、设置索引），极大边线点集合，次极大边线点集合，极小平面点集合，次极小平面点集合,删除的点云

2.1. [main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L568)->[laserCloudHandler](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L147)

首先对点云滤波，去除NaN值得无效点云；去除在 Lidar 坐标系原点 MINIMUM_RANGE 距离以内的点；计算起始点和终止点角度；对于当前点，调整其到合理范围；对每一个点计算其 scanID 和该点对应的扫描到的时间，将结果放在intensity中。

2.2 [main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L568)->[laserCloudHandler](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L191)

对每一个点计算其 scanID 和该点对应的扫描到的时间，将结果放在intensity中，intensity=scanID+点对应的在线上的时间，整数部分是scan线束的索引，小数部分是相对起始时刻的时间。
为点云点找到对应的扫描线scanID，每条扫描线都有它固定的俯仰角，我们可以根据点云点的垂直角度为其寻找对应的扫描线。
并按照每条scan的点云进行存储

2.3 [main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L568)->[laserCloudHandler](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L303)

将所有scan点云 集合到一个点云里面去，但是使用两个数组标记起始和结果，

2.4  [main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L568)->[laserCloudHandler](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L314)

有序地计算曲率，这里的laserCloud是有序的点云，故可以直接这样计算，但是在每条scan的交界处计算得到的曲率是不准确的；
存储每个点的曲率、每个点初始自然序列、每个点附近的特征点数量、每个点的点类型（分为极大边线点、次极大边线点、极小平面点、次极小平面点）

2.5 [main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L568)->[laserCloudHandler](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L337)

曲率计算完成后进行特征分类，提取特征点有几点原则：
1）为了提高效率，每条扫描线分成6个扇区，在每个扇区内，寻找曲率最大的20个点，
    作为次极大边线点，其中最大的2个点，同时作为极大边线点；
2）寻找曲率最小的4个点，作为极小平面点，剩下未被标记的点，全部作为次极小平面点。
3） 为了防止特征点过多聚堆，每提取一个特征点（极大/次极大边线点，极小平面点），
    都要将这个点和它附近的点全都标记为“已选中”，在下次提取特征点时，将会跳过这些点。
    对于次极小平面点，采取降采样的方法避免过多聚堆。

2.6 [main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L568)->[laserCloudHandler](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L517)

发布
1）对点云滤波，去除NaN值得无效点云；去除在 Lidar 坐标系原点 MINIMUM_RANGE 距离以内的点；对点云排序之后，按照从第1个scan到最后一个scan，的点云
2）四种类型的点云（极大边线点、次极大边线点、极小平面点、次极小平面点）

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| velodyne_cloud_2  | /camera_init |  | 排序点云 | sensor_msgs::PointCloud2 |
| laser_cloud_sharp | /camera_init |  | 极大边线点 | sensor_msgs::PointCloud2 |
| laser_cloud_less_sharp | /camera_init |  | 次极大边线点 | sensor_msgs::PointCloud2 |
| laser_cloud_flat | /camera_init |  | 极小平面点 | sensor_msgs::PointCloud2 |
| laser_cloud_less_flat  | /camera_init |  | 次极小平面点 | sensor_msgs::PointCloud2 |

3）也可以选择把每一条scan都发布出去，需要把[PUB_EACH_LINE](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L549)设置为true

## 2.2 laserOdometry
scanRegistration的功能就是提取特征点。主要功能是前端的激光里程计和位姿粗估计。

1. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L241)
1)设置里程计的帧率；
2)订阅消息话题：从scanRegistration节点订阅的消息话题，分别为极大边线点  次极大边线点   极小平面点  次极小平面点 全部点云点
topic                                           frame_id
laser_cloud_sharp               camera_init
laser_cloud_less_sharp     camera_init
laser_cloud_flat                    camera_init
laser_cloud_less_flat           camera_init
velodyne_cloud_2            camera_init ;
1) 注册发布的话题：发布上一帧的边线点话题、平面点话题、全部有序点云话题，就是从scanRegistration订阅来的点云，未经其他处理、帧间的位姿变换话题、帧间的平移运动


2. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L277)

1）判断订阅的五个话题不为空；首先确保订阅的五个消息都有，有一个队列为空都不行
2） 获取五个话题的时间戳，如果关键点的时间戳和完整数据的时间戳有一个不相等，那么报错；因为同一帧的时间戳都是相同的，这里比较是否是同一帧
3）分别将五个点云消息取出来，同时转成pcl的点云格式；数据多个线程使用，这里先进行加锁，避免线程冲突

3. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L331)
1）一个什么也不干的初始化，没有延迟时间，主要用来跳过第一帧数据，直接作为第二帧的上一帧；
此外，跳过处理点云阶段，也相当于做了一些初始化；发布单位变换为激光雷达坐标系与里程计坐标系的关系；将第一帧特征点设置为上一帧特征点；使用第一帧的点云更新kd-tree；

4. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L338)
开始进行第二帧之后的处理

4.1 [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L342)
取出比较突出的特征点数量，极大边线点和极小平面点

4.1.1 [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L348)
进行点到线以及点到面的ICP，迭代2次（选择当前优化位姿的特征点匹配，并优化位姿（4次迭代），然后重新选择特征点匹配并优化）

4.1.1.1 [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L356)
定义ceres优化器，这里对旋转的求导使用自带的local param

4.1.1.2  [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L382)

基于最近邻原理建立corner特征点之间关联

基于最近邻原理建立corner特征点（边线点）之间的关联，每一个极大边线点去上一帧的次极大边线点中找匹配；

采用边线点匹配方法:假如在第k+1帧中发现了边线点i，通过KD-tree查询在第k帧中的最近邻点j，查询j的附近扫描线上的最近邻点l，

j与l相连形成一条直线l-j，让点i与这条直线的距离最短。

构建一个非线性优化问题：以点i与直线lj的距离为代价函数，以位姿变换T(四元数表示旋转+位移t)为优化变量

4.1.1.3 [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L499)
基于最近邻原理建立surf特征点之间关联

与上面的建立corner特征点之间的关联类似，寻找平面特征点O的最近邻点ABC，即基于最近邻原理建立surf特征点之间的关联

下面采用平面点匹配方法：
假如在第k+1帧中发现了平面点i，通过KD-tree查询在第k帧（上一帧）中的最近邻点j，
查询j的附近扫描线上的最近邻点l和同一条扫描线的最近邻点m，这三点确定一个平面，让点i与这个平面的距离最短；

构建一个非线性优化问题：以点i与平面lmj的距离为代价函数，以位姿变换T(四元数表示旋转+t)为优化变量。

4.1.1.4 [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L610)

调用ceres求解器求解 ，设定求解器类型，最大迭代次数，不输出过程信息，优化报告存入summary
基于构建的所有残差项，求解最优的当前帧位姿与上一帧位姿的位姿增量：para_q和para_t

4.2 [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L626)

新计算出的位姿增量
用最新计算出的位姿增量，更新上一帧的位姿，得到当前帧的位姿，注意这里说的位姿都指的是世界坐标系下的位姿

5. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L630）
发布里程计odom话题和path
处理一次发布一次

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ----  |
| /laser_odom_to_init | /camera_init | /laser_odom | 里程计：帧间的位姿变换话题 | nav_msgs::Odometry |
| /laser_odom_path | /camera_init |  | 路径：帧间的平移运动话题 | geometry_msgs::PoseStamped |

6. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L662）
将角特征和面特征变换到扫描端点，且去畸变，但没有调用

7. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L685）

更新配准的source 当前帧变上一帧

位姿估计完毕之后，当前边线点和平面点就变成了上一帧的边线点和平面点，把索引和数量都转移过去

使用上一帧的点云更新kd-tree，如果是第一帧的话是直接将其保存为这个的； kdtree设置当前帧，用来下一帧lidar odom使用

8. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L705）

控制后端节点的执行频率，降频后给后端发送，只有整除时才发布结果；因为激光雷达为10hz，所以每5帧执行一次发布

| topic | frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  |
|/laser_cloud_corner_last|/camera|发布上一帧的边线点话题|sensor_msgs::PointCloud2|
|/laser_cloud_surf_last|/camera|发布上一帧的平面点话题|sensor_msgs::PointCloud2|
|/velodyne_cloud_3|/camera|发布全部有序点云话题，就是从scanRegistration订阅来的点云，未经其他处理|sensor_msgs::PointCloud2|

9. [laserOdometry](./src/A-LOAM-NOTED/src/laserOdometry.cpp#L735）
记录发布后的帧数，便于每一定次数的帧数发布一次


## 2.3 laserMapping

求解位姿，使用cube方式更新地图，

1. [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L951)
main函数，读取cmake参数，定义发布和订阅：

2. [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L245)

开始进程

2.1 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L251)

为了保证LOAM算法整体的实时性，Mapping线程每次只处理 cornerLastBuf.front() 及其他与之时间同步的消息；

1）odometryBuf、 surfLastBuf、fullResBuf抛弃时间戳比最新的 cornerLastBuf 小的数据
2）选择 odometryBuf、surfLastBuf、fullResBuf与 cornerLastBuf 时间戳相等的数据，否则break结束进程
3）将cornerLastBuf、surfLastBuf、fullResBuf 转换为 pcl::PointCloud<PointType>::Ptr 格式
4）将 odometryBuf 转换为 q_wodom_curr（Eigen::Quaterniond） 和 t_wodom_curr （Eigen::Vector3d）
5）清空 cornerLastBuf 的历史缓存，为了LOAM的整体实时性

2.2 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L330)

上一帧的增量 wmap_wodom * 本帧Odometry位姿 wodom_curr ，旨在为本帧Mapping位姿w_curr设置一个初始值

2.3  [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L338)
初始化cube，管理cube


1）等效于以50m为单位进行缩放，因为LOAM用1维数组进行cube的管理，而数组的index只用是正数，所以要保证IJK坐标都是正数，所以加了laserCloudCenWidth/Heigh/Depth的偏移，来使得当前位置尽量位于submap的中心处，也就是位置尽量处于所有格子的中心附近，偏移laserCloudCenWidth/Heigh/Depth会动态调整，来保证当前位置尽量位于submap的中心处。

2）由于计算机求余是向零取整，为了不使（-50.0,50.0）求余后都向零偏移，当被求余数为负数时求余结果统一向左偏移一个单位，也即减一

3) 下面的6个while loop的作用：要注意世界坐标系下的点云地图是固定的，但是IJK坐标系我们是可以移动的，所以这6个while loop的作用就是调整IJK坐标系（也就是调整所有cube位置），使得五角星在IJK坐标系的坐标范围处于3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18，目的是为了防止后续向四周拓展 cube 时， index （即IJK坐标）成为负数。

以第一个循环举例：
在I方向上，将cube[I] = cube[I-1],最后一个空出来的cube清空点云，实现IJK坐标系向I轴负方向移动一个cube的效果，从相对运动的角度看，就是图中的五角星在IJK坐标系下向I轴正方向移动了一个cube，所以 centerCubeI 最后++， laserCloudCenWidth 也会++，为下一帧Mapping时计算五角星的IJK坐标做准备。

2.4 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L551)

构建submap


1）向IJ坐标轴的正负方向各拓展2个cube，K坐标轴的正负方向各拓展1个cube，上图中五角星所在的蓝色cube就是当前位置所处的cube，拓展的cube就是黄色的cube，这些cube就是submap的范围

记录submap中的所有cube的index，记为有效index

2）将有效 index 的 cube 中的点云叠加到一起组成 submap 的特征点云（分为点特征点云submap、面特征点云submap），并进行滤波降采样

2.5 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L595)
确保有足够的点面特征

定义ceres优化器，并赋予参数

2.5.1  [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L618)

构建点线残差

1）将点变换到世界坐标系下
2）寻找最近的五个点
3）采用PCA原理 寻找最近的两个点，构建点线残差

2.5.1  [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L691)

构建点面残差

1）将点变换到世界坐标系下
2）寻找最近的五个点
3）采用最小二乘原理，得到平面的法向量，构建点线残差

2.6 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L763)

开始求解

2.7 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L788)

完成ICP（迭代2次）的特征匹配后，用最后匹配计算出的优化变量w_curr，更新增量wmap_wodom，为下一次

2.8 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L792)

下面两个for loop的作用就是将当前帧的特征点云，逐点进行操作：转换到world坐标系并添加到对应位置的cube中

2.9 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L847)

因为新增加了点云，对之前已经存有点云的cube全部重新进行一次降采样;这个地方可以简单优化一下：如果之前的cube没有新添加点就不需要再降采样

2.10 [laserMapping](./src/A-LOAM-NOTED/src/laserMapping.cpp#L865)

1）发布局部地图，每五帧发布一次，所有的线面点，在地图坐标系下
2）发布全局地图，每20帧发布一次，所有的线面点，在地图坐标系下
3）发布当前帧点云，所有的点，在地图坐标系下
4）发布map更新后的里程计
5） 发布map更新后的位姿
6）发布tf变换 ：//camera_init是base，是世界坐标系的id，/aft_mapped是当前点云的坐标，是child

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /laser_cloud_surround | /camera_init |  | 发布局部地图，每五帧发布一次，所有的线面点，在地图坐标系下 | sensor_msgs::PointCloud2 |
| /laser_cloud_surround | /camera_init |  | 发布全局地图，每20帧发布一次，所有的线面点，在地图坐标系下 | /laser_cloud_map |
| /velodyne_cloud_registered | /camera_init |  | 发布当前帧点云，所有的点，在地图坐标系下 | /laser_cloud_map |
| /aft_mapped_to_init | /camera_init | /aft_mapped | 发布map更新后的里程计| nav_msgs::Odometry| 
| /aft_mapped_path | /camera_init |  | 发布map更新后的位姿 | geometry_msgs::PoseStamped |



# 3 涉及到的知识点
## 3.1. 计时
作者自己设计的计时类，以构造函数为起始时间，以toc()函数为终止时间，并返回时间间隔(ms)
[main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L133)

## 3.2 把点云从ros格式转到pcl的格式
[main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L143)

## 3.3 对点云滤波，去除NaN值得无效点云
[main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L147)

## 3.4 降采样，VoxelGrid
[main](./src/A-LOAM-NOTED/src/scanRegistration.cpp#L506)





