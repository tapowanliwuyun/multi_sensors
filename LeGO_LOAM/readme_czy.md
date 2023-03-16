[toc]

# 1 编译和运行
## 1.1 编译
```basn
	cd ~/LeGO_LOAM/src
	git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
	cd ..
	catkin_make
```

## 1.2 运行
	source devel/setup.bash
播放launch文件
	roslaunch lego_loam run.launch
播放现有的bag包
	rosbag play *.bag --clock --topic	
	如：rosbag play ~/bag_file/multi_sensor/LeGO_LOAM/2018-05-18-14-49-12_0.bag --clock 
	    rosbag play ~/bag_file/multi_sensor/LeGO_LOAM/2018-05-18-14-49-12_0.bag --clock --topic /velodyne_points /imu/data
注意：虽然 /imu/data 是可选的，但如果提供它可以大大提高估计精度。

# 2 代码解析

## 2.1 imageProjection.cpp
* [imageProjection](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp)

### 2.1.1 main函数

[imageProjectionL557](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L557)

1) 初始化 ImageProjection 类  查看2.1.2

### 2.1.2 初始化 ImageProjection 类

[imageProjectionL557](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L37)

#### 2.1.2.1 订阅原始的激光点云，进入回调函数 ImageProjection

[imageProjectionL557](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L188)

订阅激光雷达点云信息之后的回调函数

##### 2.1.2.1.1 转换ros消息格式为pcl库的点云格式

  [copyPointCloud L167](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L167)

1） 读取ROS点云转换为PCL点云，保存为 pcl::PointCloud<pcl::PointXYZI>::Ptr
2）移除无效的点云 Remove Nan points，保存为 pcl::PointCloud<pcl::PointXYZI>::Ptr
3）如果点云有"ring"通过，则保存为laserCloudInRing（pcl::PointCloud<PointXYZIR>::Ptr），作用是判断是不是使用了 velodyne 的激光雷达（因为velodyne雷达自带ring）

##### 2.1.2.1.2找到开始时刻和结束时刻的方向角度

  [findStartEndAngle L206](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L206)

1）计算开始点和结束点的航向角 (负号表示逆时针旋转)   ，加 2 * M_PI 表示已经转转了一圈
2）保证 所有角度 落在 [M_PI , 3M_PI] 上 
3）以及结束点和开始点之差
##### 2.1.2.1.3 将点云信息投影到 16 * 1800 分辨率的图像上（点云阵列上）

  [projectPointCloud L223](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L223)

遍历整个点云，对于每个点
1）提取点云中 x y z 坐标数值
2）判断是不是使用了 velodyne 的雷达，并根据此来提取当前点所在的线束；
是其他的雷达 就通过俯仰角 确定当前的激光点是来自哪个线束 index 
3）计算当前点的水平角，并判断其位置（0～1800）
4）计算当前点到激光雷达传感器的距离，要大于1.0，数值保存到 rangeMat 中 
5）定义距离图像为16✖1800，并赋值
6）将 index 和 横坐标存储在 intensity 中 整数部分是线束数值  小数部分是方向角度
7）深度图的索引值  index = 列号 +  行号 * 1800 
8）将当前点以两种方式存储
    a) fullCloud 存放的是坐标信息（二维的），此时的thisPoint有x、y、z、I信息，I为其在一帧中的位置
    b) fullInfoCloud 增加了点到传感器的距离信息（三维的） ，I为其距激光传感器的位置

##### 2.1.2.1.4 滤除地面点，将下7条扫描线且相邻两个线束之间的夹角小于10度的点标记为地面点

  [groundRemoval L282](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L282)


说明：
    groundMat 矩阵 默认是0，也就是非地面点
     -1, no valid info to check if ground of not 没有有效的信息判断是否为地面点
      0, initial value, after validation, means not ground    默认值，意味着非地面点
      1, ground   地面点

遍历前7条扫描线的每个点（前7个激光雷达扫描线束足够满足地面点的检测 所以只遍历 7 次）
1）如果之前计算过的 intensity 是 -1 则直接默认为是一个无效点（设-1）
2）计算相邻两个线束之间的夹角 
3） 如果夹角绝对数值小于 10 度， 则可以判断为平面（设1）
4）给地面点和无效点 标记一个符号 为 -1 ，标志存入 labelMat 矩阵（ -1 就代表是地面）
5）如果有topic订阅地面点云信息，则赋予 groundCloud 之前提取的地面点云

##### 2.1.2.1.5 分割点云，聚类

  [cloudSegmentation L337](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L337)

    目的是为了筛选出之后优化中使用的点

1）对非地面点逐点进行聚类； 如果labelMat[i][j]=0,表示没有对该点进行过分类，需要对该点进行聚类；调用 labelComponents 函数（查看2.1.2.1.5.1 对点云进行分割 聚类）

2）提取分割点云 用于激光雷达里程计；记录每个scan点云的起始序列和终止序列开，始4点和末尾6点舍去不要；
    a）如果 label 为 999999 则跳过（labelMat 数值为 999999 表示这个点是因为聚类数量不够30而被舍弃的点）；当列数为5的倍数，并且行数较大，可以认为非地面点的，将它保存进异常点云(界外点云)中（也就是对于scan大于7，每五个点选择一个存入 outlierCloud 中）；其他的点均跳过该循环
    b）如果为地，跳过index不是5的倍数的点（可以理解为地点，每五个选一个）；
    c）上面已去掉不符合条件点，然后进行信息的拷贝和保存操作；保存的点有：
                标记 地点，以便以后不再将其视为边特征点；segMsg.segmentedCloudGround
                标记点的列索引，以便稍后标记遮挡；segMsg.segmentedCloudColInd
                记录点的距离信息；segMsg.segmentedCloudRange
                记录该点在激光雷达坐标系下的坐标； segmentedCloud
3）如果有节点订阅 SegmentedCloudPure , 那么把 被有效聚类的点云数据保存到 segmentedCloudPure 中去
    地面点和无效点类别为-1
    
###### 2.1.2.1.5.1 对点云进行分割 聚类

  [labelComponents L337](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L411)

1）传进来的两个参数，按照坐标不同 分别给他们放到 X 与 Y 的数组中
2）进入循环处理传入的数据：
    a）取出传入的未分类的点位置，并且赋予其初始类别
    b）遍历整个点云 遍历前后左右四个点,求点之间的角度数值 ，并根据角度值判断是否属于一类，如果大于这个阈值，说明属于同一类
3）判断是否属于有效的聚类
    a）如果是 allPushedIndSize（） 累加的数量增加到了30 个 则判断这部分点云属于一个聚类
    b）如果垂直方向（点的类别存在于3个scan以上）上点的数量大于 5个 默认是一个有效的聚类
4）如果该类别有效，则类别标签+1；如果类别无效，则将所有无效的类别设置为999999类别，即无类别标志

##### 2.1.2.1.6 发布各类型的点云

  [cloudSegmentation L506](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L506)

    1）

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /segmented_cloud_info |  |  | 自定义msg；每条scan的起始序列 startRingIndex 和终止序列 endRingIndex （参考2.1.2.1.5）；根据初始点云计算的开始点 startOrientation 和结束点的航向角  endOrientation 、以及结束点和开始点之差 orientationDiff （参考2.1.2.1.2）；接地点 segmentedCloudGroundFlag 、点的列索引  segmentedCloudColInd 、点的距离信息 segmentedCloudRange 、（参考2.1.2.1.5）| cloud_msgs::cloud_info |
| /outlier_cloud | base_link |  | 含有异常信息的点云（参考2.1.2.1.5），是未被成功分类且scan>7且每五个选一个的点 | sensor_msgs::PointCloud2 |
| /segmented_cloud | base_link |  | 成功参与聚类的点云（参考2.1.2.1.5）（包含地点，但五个选一个之后） | sensor_msgs::PointCloud2 |
| /full_cloud_projected | base_link |  | 转换成图片的点云（参考2.1.2.1.3），存放的是坐标信息，此时的thisPoint有x、y、z、I信息，I为其在一帧中的位置 | sensor_msgs::PointCloud2 |
| /ground_cloud | base_link |  | 提取的所有地点（参考2.1.2.1.4） | sensor_msgs::PointCloud2 |
| /segmented_cloud_pure | base_link |  | 选择不是地面点(labelMat[i][j]!=-1)和没被舍弃的点（参考2.1.2.1.5） | sensor_msgs::PointCloud2 |
| /full_cloud_info | base_link |  | 存放坐标信息，此时的thisPoint有x、y、z、I信息，I为其距激光传感器的位置（参考2.1.2.1.3） | sensor_msgs::PointCloud2 |

## 2.2 featureAssociation.cpp

  [featureAssociation.cpp](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp)

### 2.2.1 main函数

  [featureAssociation.cppL1978](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1978)

1) 初始化 FeatureAssociation 类  查看2.1.2

### 2.2.2 初始化 ImageProjection 类

  [featureAssociation.cppL37](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L37)

#### 2.2.2.1 订阅成功参与聚类的点云 segmented_cloud ，进入回调函数 FeatureAssociation::laserCloudHandler

  [featureAssociation.cppL472](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L472)

1）获取当前 segmented_cloud 的时间戳，转换为秒
2）将数据格式从 sensor_msgs::PointCloud2 转换为 pcl::PointCloud<pcl::PointXYZI>::Ptr

#### 2.2.2.2 订阅了自定义的消息数据 segmented_cloud_info ，进入回调函数 FeatureAssociation::laserCloudInfoHandler

  [featureAssociation.cppL495](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L495)


1）获取当前 segmented_cloud_info 的时间戳，转换为秒
2）保存格式不变，cloud_msgs::cloud_info
3）将标志位 newSegmentedCloudInfo 置为 true ，表示有数据到来

#### 2.2.2.3 订阅未成功聚类的点 outlier_cloud ，进入回调函数 FeatureAssociation::outlierCloudHandler

  [featureAssociation.cppL485](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L485)

1）获取当前 outlier_cloud 的时间戳，转换为秒
2）将数据格式从 sensor_msgs::PointCloud2 转换为 pcl::PointCloud<pcl::PointXYZI>::Ptr

#### 2.2.2.4 imu数据 /imu/data ，进入回调函数 FeatureAssociation::imuHandler

  [featureAssociation.cppL440](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L440)

1）得到IMU读数下的翻滚角、俯仰角、偏航角
2）计算去除重力矢量后的 IMU 在车辆坐标系下的读数，参考：https://zhuanlan.zhihu.com/p/242559124；最多只会存储200个IMU数据作为缓存
3）存储imu数据的翻滚角、俯仰角、偏航角、三轴线加速度、三轴角加速度

#### 2.2.2.4.1 AccumulateIMUShiftAndRotation

  [featureAssociation.cppL400](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L400)

1）读取imu最新的翻滚角、俯仰角、偏航角、加速度
2）将加速度从车辆坐标系变换到地理坐标系（g所在的坐标系） https://zhuanlan.zhihu.com/p/242559124（注意这个变换可以参考自己的笔记p13，且这里的roll应该是rz，但是roll的定义是在imu坐标系下的，rz是在地理坐标系下的）
3）根据imu的速度和线/角加速度，计算地理坐标系下的 位移、速度、角度

### 2.2.3 运行程序

  [featureAssociation.cppL1928](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1928)

1）判断是否有新数据到来以及时间戳是否相对同步，并重置标志位
2）特征提取 


#### 2.2.3.1 特征提取

  [featureAssociation.cppL1945](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1945)

##### 2.2.3.1.1 利用IMU数据补偿点云畸变 adjustDistortion 

  [featureAssociation.cppL503](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L503)

遍历当前帧 segmentedCloud 
1. 对每个点做坐标轴变换，变换到地理坐标系下。
   这里激光点云的坐标系和imu坐标系相同（imuhandler() 中相同的坐标变换）
2. 计算偏航角yaw。
    因为前面有负号，ori=[-M_PI,M_PI]，因为 segInfo.orientationDiff 表示的范围是(PI,3PI)，在2PI附近；
    start-ori>M_PI/2，说明ori小于start，不合理，正常情况在前半圈的话，ori-stat范围[0,M_PI]；
    ori-start>3/2*M_PI,说明ori太大，不合理；
    end-ori>3/2*PI,ori太小；
    ori-end>M_PI/2,太大；
  3. 用 point.intensity 来保存时间，当前点据起始点的时间
  4. 寻找距离激光点云时间戳最近的IMU数据
      1）IMU数据时间戳大于当前激光点的时间；
      2）如果一直都没有，则把最后一个IMU数据作为最新的IMU数据，该条件内imu数据比激光数据早；
  5. 获取当前imu数据的状态（翻滚角、俯仰角、偏航角、加速度、速度），根据imu数据与激光数据关系选择直接获取或者插补。
      1）未找到 IMU 数据时间戳大于当前激光点的时间的 IMU 数据，则把最后一个IMU数据作为最新的IMU数据；
      该条件内imu数据比激光数据早；(打个比方,激光在9点时出现，imu现在只有8点的)；
      这种情况上面while循环是以imuPointerFront == imuPointerLast结束的；
      
      直接获取当前imu 数据的 翻滚角、俯仰角、偏航角、加速度、速度。
      2）在上面找到 IMU 数据时间戳大于当前激光点的时间的 IMU 数据，则进行插值；计算 当前imu 数据的 翻滚角、俯仰角、偏航角、加速度、速度。

      当前 timeScanCur + pointTime < imuTime[imuPointerFront]，而且 imuPointerFront 是最早一个时间大于 timeScanCur + pointTime 的imu数据指针；

      因为imuRollCur和imuPitchCur通常都在0度左右，变化不会很大，因此不需要考虑超过2M_PI的情况；imuYaw转的角度比较大，需要考虑超过2 * M_PI的情况，由于一般相邻IMU不会出现180度的变化，一旦出现，说明需要进行角度的变换

      3）如果是每帧第一个激光点。
        记录下此时 角度（翻滚角、俯仰角、偏航角）、速度和位移。
        如果imu数据充足，就对角度偏移进行插补（当前帧第一个点的IMU读数角度）。
        计算当前帧第一个点的IMU读数与上一帧第一个点的IMU读数之差。

        4）这里更新的是i=0时刻的rpy角，后面将速度坐标投影过来会用到i=0时刻的值（提前计算好i第一个激光点角度读数的cos和sin值）。（参考2.2.3.1.1.1 ）

    6. 如果不是每帧第一个点
        1）获取当前点相对于第一个点的 非匀速运动下的激光位置偏移量，并将该偏移量 从地理坐标系下，变换到 当前帧第一个点的 车辆坐标系下
        （参考 2.2.3.1.1.2 ）
        2）计算 当前点 到 第一个点 时的 速度差值，并将该速度差值 从地理坐标系下，变换到 当前帧第一个点的 车辆坐标系下（参考2.2.3.1.1.3）
        3） 将 当前点 的坐标变换到初始 i=0 时刻；将当前点 从当前帧的当前点的车辆坐标系下 变换到 地理坐标系（全局坐标系），反向变换

##### 2.2.3.1.1.1  提前计算好imu初始角度（roll、pitch、yaw）读数的cos和sin值 updateImuRollPitchYawStartSinCos

  [featureAssociation.cppL321](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L321)

1）计算 imuRollStart 、 imuPitchStart 、 imuYawStart 的cos和sin值

##### 2.2.3.1.1.2  得到 激光位置偏移量 ShiftToStartIMU

  [featureAssociation.cppL331](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L331)

  1）获取当前点相对于第一个点的 非匀速运动下的激光位置偏移量
  2）将 该非匀速运动下的激光位置偏移量 从地理坐标系下，变换到 当前帧第一个点的 车辆坐标系下

##### 2.2.3.1.1.3  得到 IMU速度差值 VeloToStartIMU

  [featureAssociation.cppL351](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L351)

  1）计算当前激光点到第一个激光点时的 速度差值
  2）将 速度差值 从地理坐标系下，变换到 当前帧第一个点的 车辆坐标系下

##### 2.2.3.1.1.4  将点的坐标变换到初始i=0时刻 TransformToStartIMU

  [featureAssociation.cppL371](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L371)

  1）将当前点 从当前帧的当前点的车辆坐标系下 变换到 地理坐标系（全局坐标系），反向变换
  2）将当前点 从地理坐标系下 变换到 第一个激光点时的IMU下 
  3）加上位置偏移量，这里 加 是因为激光运动和点云运动是相反的。 可以考虑加速运动下，PositionShift（imuShiftFromStartXCur）为正，并且点云测量坐标系变小，因此需要让点云坐标变大。




## 1.3 总结知识点
### 1.3.1 4邻居的 BFS 搜索聚类

计算两个点之间的角度，从而确定是否为一类

[labelComponents L337](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L411)

可以查看文章https://zhuanlan.zhihu.com/p/72932303












