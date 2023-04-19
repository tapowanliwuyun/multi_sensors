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
        3） 将 当前点 的坐标变换到初始 i=0 时刻；将当前点 从当前帧的当前点的车辆坐标系下 变换到 地理坐标系（全局坐标系），反向变换；并且消除加速度带来的运动误差；
        4）将当前 imu 数据时间 设施为 上一个 imu 数据时间

##### 2.2.3.1.1.1  提前计算好当前帧 imu初始角度（roll、pitch、yaw）读数的cos和sin值 updateImuRollPitchYawStartSinCos

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


##### 2.2.3.1.2 计算曲率 calculateSmoothness

  [featureAssociation.cppL660](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L660)

1. 计算曲率，lego-loam论文里面称之为粗糙度（roughness），与LOAM相同方法称之为curve or smoothness平滑度。
2. 这里的计算没有完全按照公式进行，缺少除以总点数i和r[i]
3. 将曲率的平方存入 cloudCurvature 
4.  将当前点设置为 未被选中过 存入 cloudNeighborPicked
5.  将当前点默认为 surfPointsLessFlatScan = 1 ， 存入 cloudLabel 
6.  将当前点的 曲率平方 和 序列i 存入 cloudSmoothness 

##### 2.2.3.1.3 标记出远点和离散点 markOccludedPoints

  [featureAssociation.cppL687](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L687)

遍历所有的点（前五个点和后六个点除外）
1. 获取当前点和下一个点的深度
2. 获取当前点和下一个点 之间的 相邻点位置 的距离，如果距离足够小，且深度差别过大，则将其  cloudNeighborPicked 设置为1 表示已经被选中过
3. 如果距离足够大，则获取 当前点 与 前一个点 和 下一个点 的深度差值 ；差值 大于当前点深度值一定比例 视为点突变很远 ，将该点视为噪声，则将其  cloudNeighborPicked 设置为1 表示已经被选中过

##### 2.2.3.1.4 特征提取 extractFeatures

  [featureAssociation.cppL725](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L725)

* 注意 面点（surf） 必须是 地面点，less surf 不一定是

1. 初始化四种类型点云的指针
2. 循环遍历每条scan，每条scan分成 6个子图 提取的特征点相对均匀
   1. 对于每个子图，获取当前子图的 初始点序列 和 结束点序列
   2. 将点按照 cloudSmoothness.value 从小到大排序
   3. 选出没有被选过的点（cloudNeighborPicked 为 0 ）、该点的曲率大于所设定的边点的阈值（cloudCurvature）、不是地面点（segInfo.segmentedCloudGroundFlag）的点，挑出曲率大的点，前两个点设置其为 shape edge（cloudLabel = 2），前20个点设置为less shape edge（cloudLabel = 1），将这些点设置为已选取（cloudNeighborPicked = 1）；检查该点附近是否有遮挡点，如果有，将该点附近的点的  cloudNeighborPicked 设置为 1
   4. 选出没有被选过的点（cloudNeighborPicked 为 0 ）、该点的曲率小于所设定的边点的阈值（cloudCurvature）、是地面点（segInfo.segmentedCloudGroundFlag）的点，挑出曲率小的点，前4个点设置其为 surf（cloudLabel = -1）；检查该点附近是否有遮挡点，如果有，将该点附近的点  cloudNeighborPicked 设置为 1
   5. 剩余的特征点全部为 less surf points（默认 surf（cloudLabel = 0），将所有less surf 存入 surfPointsLessFlatScan （表示每个scan的surf点）。
3. 对 surfPointsLessFlatScan 进行降采样，并汇总所有scan的 less surf 点到  surfPointsLessFlat 中

##### 2.2.3.1.5 发布提取的特征点云消息 publishCloud

  [featureAssociation.cppL844](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L844)

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
|  |  |  |  |  |
| /laser_cloud_sharp | /camera |  | sharp点（当前帧第一个点坐标系下），不是地面点（参考2.2.3.1.4） |  sensor_msgs::PointCloud2 |
| /laser_cloud_less_sharp | /camera |  | less sharp点（当前帧第一个点坐标系下），不是地面点（参考2.2.3.1.4） |  sensor_msgs::PointCloud2 |
| /laser_cloud_flat | /camera |  | flat点（当前帧第一个点坐标系下），是地面点（参考2.2.3.1.4） |  sensor_msgs::PointCloud2 |
| /laser_cloud_less_flat | /camera |  | less flat点（当前帧第一个点坐标系下），剩下的点 （参考2.2.3.1.4）|  sensor_msgs::PointCloud2 |


#### 2.2.3.2 特征关联（Feature Association）

  [featureAssociation.cppL1956](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1956)

##### 2.2.3.2.0 检查数据
  [featureAssociation.cppL1958](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1958)

首先判断 标志 systemInitedLM ，检查是否初始化 kdtree等数据，这里第一帧之后就有了，应该就是第一帧不处理设置的
只要检查一次就可以了，一次设置true，整个进程都是true，也就初始化的是否初始化为 false

##### 2.2.3.2.0.1 检查数据
  [featureAssociation.cppL1701](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1701)

1.  将 车辆坐标系下 当前帧 线点和面点 赋值laserCloudCornerLastNum 和laserCloudSurfLastNum
2.   车辆坐标系下 当前帧 线点和面点大于一定数量，将 设置为 上一帧 线点（laserCloudCornerLast）和面点 （laserCloudSurfLast），并将其赋值到kdtree中，
3.  发布 上一帧线点和面点 （车辆坐标系下 上一帧最后一个点时刻 也就是当前帧第一个点下）

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /laser_cloud_corner_last | /camera |  | 发布上一帧线点 | sensor_msgs::PointCloud2 |
| /laser_cloud_surf_last | /camera |  | 发布上一帧面点  | sensor_msgs::PointCloud2 |


4. 设置标志位  systemInitedLM 为 true
5. 
##### 2.2.3.2.1 更新初始位姿 updateInitialGuess

  [featureAssociation.cppL1736](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1736)

更新初始预测位姿，预测位姿，点云畸变处理
1. 将当前的imu角度赋予上一时刻的角度
2. 将当前的 车辆坐标系下 当前帧 最后一个点 - 第一个点 之间的imu车辆位置偏移量 存储
3. 将当前的 车辆坐标系下 当前帧 最后一个点 - 第一个点 之间的imu车辆速度偏移量 存储
4. 设置坐标变换 transformCur 的角度量 为 车辆坐标系下 当前帧 第一个点 - 最后一个点 之间 角度差值
5. 设置坐标变换 transformCur 的位移量 为 车辆坐标系下 第一个点 - 最后一个点 之间的位移差值

* 这里我理解 transformCur 就是 车辆坐标系下 当前帧 最后一个激光点 到 第一个点的 变换；后面主要当作 当前帧 到上一帧的变换

##### 2.2.3.2.2 特征点匹配 ， 前后帧匹配计算两帧之间的相对位姿变换 updateTransformation

  [featureAssociation.cppL1767](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1767)

1. 如果上一帧 角特征点数量 laserCloudCornerLastNum 和 面特征点数量较少  laserCloudSurfLastNum 返回不执行匹配过程
2. 开始面特征匹配迭代
   1. 传入迭代次数，在上一帧所有surf（less surf 和 surf）中寻找面点 最近临 三个点，计算三点平面法向量  和 点面距离，经过影响因子处理后存储起来，将当前面点也存储卡利（参考2.2.3.2.2.2）
   2. 如果当前帧寻找到点面特征的点 少于 10个，则跳过匹配
   3. 已知 当前点面距离和三点平面法向量，依据最小二乘法 计算 transformCur（当前帧初始点时刻车辆坐标系 到 上一帧初始点时刻车辆坐标系） 中的 rx rz ty（翻滚角、俯仰角、向上的位移），判断变化的大小是否大于一定程度，如果没有，则这次匹配失败
3. 开始线特征匹配迭代
   1. 传入迭代次数，在上一帧所有 sharp （less sharp 和 sharp ）中寻找线点 最近临 两个点，计算 点线距离的矢量方向 和 模，经过影响因子处理后存储起来，将当前线点也存储起来。
   2.  如果当前帧寻找到点线特征的点 少于 10个，则跳过匹配
   3.  已知 当前点面距离和三点平面法向量，依据最小二乘法 计算 transformCur（当前帧初始点时刻车辆坐标系 到 上一帧初始点时刻车辆坐标系） 中的ry tx tz （偏移角、平面的两个方向）



##### 2.2.3.2.2.1 变换到初始时刻 TransformToStart

  [featureAssociation.cppL918](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L918)

将 当前帧的当前点 从当前帧  当前点时刻车辆坐标系 变换到 第一个初始点时刻车辆坐标系 

1. 获取的是当前帧的当前点 pi 到 第一个点 的所占当前帧的比例
2. 获取两个点之间的角度和位置变换（车辆坐标系下）
3. 将当前帧的当前点 基于变换，从当前帧  当前点坐标系 变换到 第一个初始点坐标系 

##### 2.2.3.2.2.2 寻找点面特征 findCorrespondingSurfFeatures

  [featureAssociation.cppL1223](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1223)

传入迭代次数，在上一帧所有surf（less surf 和 surf）中寻找面点 最近临 三个点，计算三点平面法向量  和 点面距离，经过影响因子处理后存储起来，将当前面点也存储起来。

1. 对当前帧每一个面点进行遍历
   1. 将该面点 经过坐标变换到 当前帧初始点时刻；因为后面将上一时刻的特征点转换到了每一帧的末尾时刻，这里用的时候转换过来。
   2. 每五次迭代 进行一次 寻找匹配点
      1. 使用k点最近邻搜索 寻找该点在上一帧面点（上一帧最后点时刻坐标系下，也就是当前帧第一个点时刻 车辆坐标系下）的最近点1个
      2. 如果满足邻近点距离要求，获得该点在 laserCloudSurfLast （上一帧 最后点时刻 的车辆坐标系下，虽然一开始得到当前帧的的是在 初始点时刻坐标系下，但是转换成 上一帧时 变换到了 最后点时刻坐标系下）的序列；获得该点所在的激光线束 closestPointScan ；
         1. 从已找到的第一个激光点开始，往大序列开始寻找
            1. 大于第一个点 2.5 个线束的点不考虑
            2. 得到这个点 和 第一个激光点 的均方差
            3. 找到的点：一个是与第一个点在相同线束；一个不同线束。均根据均方差判断是否最近的点，并更新最近的那个点的 均方差 和 序列
         2. 同理 从已找到的第一个激光点开始，往小序列开始寻找
            1. 小于第一个点 2.5 个线束的点不考虑
            2. 得到这个点 和 第一个激光点 的均方差
            3. 找到的点：一个是与第一个点在相同线束；一个不同线束。均根据均方差判断是否最近的点，并更新最近的那个点的 均方差 和 序列
      3. 记录最终找到的三个点在 laserCloudSurfLast （上一帧 最后点时刻 的车辆坐标系下）的 序列
   3. 三个点都找到，就开始计算距离
      1. 获取三个点 （上一帧 最后点时刻 的车辆坐标系下）
      2. 计算三个点构成的平面 的法向量[pa,pb,pc]、第一个点 在法向量上的投影长度pd、以及模长 ps ，并且 归一化
      3. 求出 该面点 到 三点平面 的距离
      4. 计算影响因子（暂且不明）
      5. 在影响因子大于0.1 时
         1. 将 法向量和点面距离 经过影响因子处理后 保存进 coeffSel 
         2. 将 该面点 保存进 laserCloudOri（上一帧 最后点时刻 的车辆坐标系下）

##### 2.2.3.2.2.3 面特征匹配计算变换矩阵 calculateTransformationSurf

  [featureAssociation.cppL1357](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1357)

已知 当前点面距离和三点平面法向量，依据最小二乘法 计算 transformCur（当前帧初始点时刻车辆坐标系 到 上一帧初始点时刻车辆坐标系） 中的 rx rz ty（翻滚角、俯仰角、向上的位移）

1.  获取成功构建 点面特征 的面点数量，并遍历每一个面点
    1.  构建At*A*X = At*B ；A=[J的偏导]; B=[权重系数*(点到直线的距离)] ；注意面点仅更新 rx rz ty （翻滚角、俯仰角、向上的位移）
    2.  注意：这里权重系数作用：这个0.05 就是让求解出来的X 缩小20倍，进而使得更新的权重降低
    3.  通过QR分解的方法，求解方程AtA*X=AtB，得到X
    4.  判断是否发生退化，如果发生进行处理
    5.  得到最终的 X，更新 transformCur（当前帧初始点时刻车辆坐标系 到 上一帧初始点时刻车辆坐标系） 中的 rx rz ty，也就是 翻滚角、俯仰角、向上的位移
    6.  将 transformCur 中的无效值 设置为0  
    7.  判断变化的大小是否大于一定程度，如果没有，则这次匹配失败

##### 2.2.3.2.2.4 线特征匹配计算变换矩阵 findCorrespondingCornerFeatures

  [featureAssociation.cppL1112](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1112)

传入迭代次数，在上一帧所有 sharp （less sharp 和 sharp ）中寻找线点 最近临 两个点，计算 点线距离的矢量方向 和 模，经过影响因子处理后存储起来，将当前线点也存储起来。


1. 对当前帧每一个线点进行遍历
   1. 将该线点 经过坐标变换到 当前帧初始点时刻；因为后面将上一时刻的特征点转换到了每一帧的末尾时刻，这里用的时候转换过来。（参考2.2.3.2.2.1 ）
   2. 每五次迭代 进行一次 寻找匹配点
      1. 使用k点最近邻搜索 寻找该点在上一帧线点（上一帧最后点时刻坐标系下，也就是当前帧第一个点时刻 车辆坐标系下）的最近点1个
      2. 如果满足邻近点距离要求，获得该点在 laserCloudCornerLast （上一帧 最后点时刻 的车辆坐标系下，虽然一开始得到当前帧的的是在 初始点时刻坐标系下，但是转换成 上一帧时 变换到了 最后点时刻坐标系下）的序列；获得该点所在的激光线束 closestPointScan ；
         1. 从已找到的第一个激光点开始，往大序列开始寻找
            1. 大于第一个点 2.5 个线束的点不考虑
            2. 得到这个点 和 第一个激光点 的均方差
            3. 找到的点是与第一个点在不同线束；均根据均方差判断是否最近的点，并更新最近的那个点的 均方差 和 序列
         2. 同理 从已找到的第一个激光点开始，往小序列开始寻找
            1. 小于第一个点 2.5 个线束的点不考虑
            2. 得到这个点 和 第一个激光点 的均方差
            3. 找到的点是与第一个点在不同线束；均根据均方差判断是否最近的点，并更新最近的那个点的 均方差 和 序列
      3. 记录最终找到的量个点在 laserCloudCornerLast （上一帧 最后点时刻 的车辆坐标系下）的 序列
   3. 两个点都找到，就开始计算距离
      1. 获取两个点 （上一帧 最后点时刻 的车辆坐标系下）
      2. 计算 点线距离的矢量方向 和 模
      3. 计算影响因子（暂且不明）
      4. 在影响因子大于0.1 时
         1. 将 矢量方向和点线距离 经过影响因子处理后 保存进 coeffSel 
         2. 将 该面点 保存进 laserCloudOri（上一帧 最后点时刻 的车辆坐标系下）

##### 2.2.3.2.2.5 线特征匹配计算变换矩阵 calculateTransformationCorner

  [featureAssociation.cppL1471](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1471)

已知 当前点面距离和三点平面法向量，依据最小二乘法 计算 transformCur（当前帧初始点时刻车辆坐标系 到 上一帧初始点时刻车辆坐标系） 中的ry tx tz （偏移角、平面的两个方向）

1.  获取成功构建 点线特征 的面点数量，并遍历每一个线点
    1.  构建At*A*X = At*B ；A=[J的偏导]; B=[权重系数*(点到直线的距离)] ；注意线点仅更新 ry tx tz （偏移角、平面的两个方向）
    2.  注意：这里权重系数作用：这个0.05 就是让求解出来的X 缩小20倍，进而使得更新的权重降低
    3.  通过QR分解的方法，求解方程AtA*X=AtB，得到X
    4.  判断是否发生退化，如果发生进行处理
    5.  得到最终的 X，更新 transformCur（当前帧初始点时刻车辆坐标系 到 上一帧初始点时刻车辆坐标系） 中的 rx rz ty，也就是 偏移角、平面的两个方向
    6.  将 transformCur 中的无效值 设置为0  
    7.  判断变化的大小是否大于一定程度，如果没有，则这次匹配失败

##### 2.2.3.2.3 坐标变换，更新位姿 integrateTransformation

  [featureAssociation.cppL1802](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1802)


1. 根据已有的 帧间 transformCur （当前帧 到 上一帧）和 transformSum（0 到 上一帧 ）的变换，计算出 世界坐标系下 从0 到 当前帧的第一个点时刻的 旋转 rx, ry, rz（参考 2.2.3.2.3.1 ）
2.  imuShiftFromStartX 为 当前帧第一个点时刻的车辆坐标系下 当前帧 最后一个点 - 第一个点时刻的 imu位移偏量；
3.  将 此时的 帧间变换  transformCur （当前帧 到 上一帧），  去除 加速度的影响之后（因为是匀速运动模型），变换到 世界坐标系下 
4.  将tx、ty、tz赋予 从0到 当前帧的 第一个点时刻的  位置（世界坐标系下）
5. 将  当前帧的全局位姿（rx、ry、rz）从 当前帧第一个点云时刻 旋转 到 上一帧最后一个点时刻 世界坐标下 
6. 将计算好的 世界坐标系下 从0到 当前帧 初始时刻 的位姿
##### 2.2.3.2.3.1  将 帧间旋转变换 累计到 变换位姿 integrateTransformation

  [featureAssociation.cppL1083](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1083)

根据已有的 帧间 transformCur （当前帧 到 上一帧）和 transformSum（0 到 上一帧 ）的变换，计算出 从0 到 当前帧的 旋转 

* transformCur 是 （当前帧 到 上一帧），传入时 作了负号处理，所以是  上一帧 到 当前帧 

##### 2.2.3.2.3.2  将  当前帧的全局位姿从 当前帧第一个点云坐标系下 变换到最后一个点云坐标系下 PluginIMURotation

  [featureAssociation.cppL1023](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1023)

具体的原理可以参考笔记 p34

##### 2.2.3.2.4 发布激光里程计消息 publishOdometry

  [featureAssociation.cppL1838](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1838)

1. 首先将全局位姿 从角度 换算为 四元数（虽然这里涉及到 坐标轴变换，但是在下一步变过来了）
2. 然后 将 该变换矩阵 赋予 里程计位姿 
3. tf播报  该位姿变换


| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /laser_odom_to_init |/camera_init  | /laser_odom | 世界坐标系下 从0到 当前帧 初始时刻 的位姿(参考2.2.3.2.3.1) | nav_msgs::Odometry |
| tf |/camera_init  | /laser_odom |  | tf::StampedTransform |

##### 2.2.3.2.5 发布用于图优化的点云 publishCloudsLast

  [featureAssociation.cppL1870](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1870)

1.  提前计算好当前帧 imu初始角度（roll、pitch、yaw）读数的cos和sin值  （参考2.2.3.1.1.1 ）
2.  将 车辆坐标系下 当前帧线点和面点 从 当前帧第一个点时刻 变换到当前帧最后一个点时刻下；
3.  将 车辆坐标系下 当前帧 线点和面点 赋值laserCloudCornerLastNum 和laserCloudSurfLastNum
4.   车辆坐标系下 当前帧 线点和面点大于一定数量，将 设置为 上一帧 线点（laserCloudCornerLast）和面点 （laserCloudSurfLast），并将其赋值到kdtree中，
5.  判断每两帧发布一次，发布 上一帧线点和面点 （车辆坐标系下 上一帧最后一个点时刻 也就是当前帧第一个点下）
    1.  原封不动转移外点 outlierCloud（参考 2.2.3.2.5.2）


| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /outlier_cloud_last | /camera |  |  含有异常信息的点云（参考2.1.2.1.5），是未被成功分类且scan>7且每五个选一个的点 | sensor_msgs::PointCloud2 |
| /laser_cloud_corner_last | /camera |  | 发布上一帧线点 | sensor_msgs::PointCloud2 |
| /laser_cloud_surf_last | /camera |  | 发布上一帧面点  | sensor_msgs::PointCloud2 |

##### 2.2.3.2.5.1 发布用于图优化的点云 TransformToEnd

  [featureAssociation.cppL952](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L952)

变换四次，将在车辆坐标系下的 当前点 从 当前帧当前点时刻变换到 最后一个激光点，并且去除imu的影响

1. 计算当前点 在 在当前帧中的比例
2. 根据比例确定 当前点 所需要的变换 =  比例 * transformCur （车辆坐标系下 当前帧 最后一个激光点 到 第一个点的 变换）
3. 要将 在车辆坐标系下的 当前点 从 当前帧当前点时刻 变换到 当前帧第一个点时刻 ，需要的是 transformCur 的逆变换 
4. 获取完整的 车辆坐标系下 当前帧 最后一个激光点 到 第一个点的 变换
5. 要将 在车辆坐标系下的 当前点 从 当前帧第一个点时刻 变换到 当前帧最后一个点时刻 ，需要的是 -transformCur 的变换 
6. 将在车辆坐标系下的 当前点 从 当前帧第一个点时刻 imu 偏移 变换到 当前帧 当前帧 最后一个点时刻 imu偏移 

##### 2.2.3.2.5.2 原封不动转移异常信息的点 adjustOutlierCloud

  [featureAssociation.cppL1857](./src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp#L1857)

原封不动转移异常信息的点 outlierCloud

## 1.3 总结知识点
### 1.3.1 4邻居的 BFS 搜索聚类

计算两个点之间的角度，从而确定是否为一类

[labelComponents L337](./src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp#L411)

可以查看文章https://zhuanlan.zhihu.com/p/72932303


* 在之前所说的世界坐标系 应该就是里程计第一帧 所在的 车辆坐标系、
* 在这之后应该都称之为 里程计坐标系 ，世界坐标系就是世界坐标系了

## 2.3 mapOptmization.cpp
* [mapOptmization](./src/LeGO-LOAM/LeGO-LOAM/src/mapOptmization.cpp)

整体功能分为回环检测、可视化以及位姿全局优化，核心是位姿优化。

主体流程：订阅特征提取后的点云、里程计数据->计算当前姿态优化的初始值->提取局部关键帧->降采样->scan-to-map地图优化（线特征、面特征、L-M）->保存关键帧与因子图优化->闭环检测->发布TF变换、点云

### 2.3.1 main函数

1. //调用构造函数,订阅话题上一帧的surf,corner,outlier点云,imu,和laser_odom,
2. std::thread 构造函数，将MO作为参数传入构造的线程中使用
   1. 进行闭环检测与闭环的功能
   2. 该线程中进行的工作是publishGlobalMap(),将数据发布到ros中，可视化
3. 全局姿态优化的主要处理部分

### 2.3.2. 初始化 mapOptimization 类


| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /outlier_cloud_last | /camera |  |  含有异常信息的点云（参考2.1.2.1.5），是未被成功分类且scan>7且每五个选一个的点 | sensor_msgs::PointCloud2 |
| /laser_cloud_corner_last | /camera |  | 发布上一帧线点 | sensor_msgs::PointCloud2 |
| /laser_cloud_surf_last | /camera |  | 发布上一帧面点  | sensor_msgs::PointCloud2 |
| /laser_odom_to_init |/camera_init  | /laser_odom | 世界坐标系下 从0到 当前帧 初始时刻 的位姿(参考2.2.3.2.3.1) | nav_msgs::Odometry |

#### 2.3.2.1 订阅 laser_cloud_corner_last ，回调函数 mapOptimization::laserCloudCornerLastHandler

1. 将消息格式变换为 pcl::PointCloud<PointType>::Ptr 的 laserCloudCornerLast
2. 设置 表示位 newLaserCloudCornerLast 为 true

#### 2.3.2.2 订阅 laser_cloud_surf_last ，回调函数 mapOptimization::laserCloudSurfLastHandler

1. 将消息格式变换为 pcl::PointCloud<PointType>::Ptr 的 laserCloudSurfLast
2. 设置 表示位 newLaserCloudSurfLast 为 true

#### 2.3.2.3 订阅 outlier_cloud_last ，回调函数 mapOptimization::laserCloudOutlierLastHandler

1. 将消息格式变换为 pcl::PointCloud<PointType>::Ptr 的 laserCloudOutlierLast
2. 设置 表示位 newLaserCloudOutlierLast 为 true

#### 2.3.2.4 订阅 laser_odom_to_init ，回调函数 mapOptimization::laserOdometryHandler

1. 将消息格式位姿变换 变换为 transformSum （在世界坐标系下，从0帧到当前帧的 变换）
2. 设置 表示位 newLaserOdometry 为 true

#### 2.3.2.5 订阅 imu 数据 ，回调函数 mapOptimization::imuHandler

1. 获取最新的IMU数据索引 imuPointerLast
2. 为 IMU时间、数据队列 赋值（这里只获取 滚动角 imuRoll 和 俯仰角 imuPitch ）（imu 坐标系 下）(另外一个姿态值没有用到，所以就只记录了其中2个值)


### 2.3.3 全局姿态优化 MO.run()

判断是否有新的数据到来并且时间差值小于0.005；
如果timeLaserOdometry -timeLastProcessing >= mappingProcessInterval，则进行以下操作：
2.3.3.1 将坐标转移到世界坐标系下，得到可用于建图的Lidar坐标，即修改transformTobeMapped的值；
2.3.3.2 提取周围的关键帧；
2.3.3.3 下采样当前scan；
2.3.3.4 当前scan进行图优化过程；
2.3.3.5 保存关键帧和因子；
2.3.3.6 校正位姿；
2.3.3.7 发布Tf；
2.3.3.8 发布关键位姿和帧数据；

#### 2.3.3.1 将坐标转移到世界坐标系下，得到可用于建图的Lidar坐标，即修改transformTobeMapped的值 transformAssociateToMap
 float transformTobeMapped[6];// 机器人的位姿,全局姿态优化的目标变量：位姿优化的初始值->在地图优化过程中得到更新->经过因子图优化后的位姿
    float transformBefMapped[6];//  未经过scan-to-model优化的位姿，即里程计位姿。
    float transformAftMapped[6];//  由经过scan-to-model优化后的位姿赋值,并在因子图优化后进行修正。

1.  将 transformBefMapped 变换到世界坐标系下 
2.  这里三次变换合成一个；最终变换到经过全局位姿优化之后的 全局位姿下 transformTobeMapped

#### 2.3.3.2 提取周围关键帧 extractSurroundingKeyFrames

函数功能：根据当前位置，提取局部关键帧集合,以及对应的三个关键帧点云集合

my
1. 如果进行闭环检测
   1. 若 recentCornerCloudKeyFrames 中的点云数量不够， 清空后重新塞入新的点云直至数量够。 
   2. 如果 recentCornerCloudKeyFrames 中的点云数量足够，则pop其最前端的一个，再往队列尾部push一个；
   3. 填充局部点云集 laserCloudCornerFromMap 、 laserCloudSurfFromMap 、 laserCloudSurfFromMap
2. 如果不进行闭环检测
   1. 进行半径 surroundingKeyframeSearchRadius （50）内的邻域搜索，获取所有的关键帧，并下采样
   2. 双重循环，不断对比surroundingExistingKeyPosesID和surroundingKeyPosesDS中点的index,如果能够找到一样，说明存在关键帧。然后在队列中去掉找不到的元素，留下可以找到的。
   3. 再来一次双重循环，这部分比较有技巧，这里把在 当前 surroundingKeyPosesDS 内 surroundingExistingKeyPosesID 中没有的点放进内 surroundingExistingKeyPosesID 里， 这个队列专门存放周围存在的关键帧。
   4. 更新局部 laserCloudCornerFromMap 、 laserCloudSurfFromMap （把面点 和 离群点 都放进） 地图
3. 不管是否进行闭环过程，最后的输出都要进行一次下采样减小数据量的过程。
最后的输出结果是 laserCloudCornerFromMapDS （所有局部关键帧的角点集合） 和 laserCloudSurfFromMapDS （所有局部关键帧平面点和离群点的集合）。

#### 2.3.3.3 降采样匹配以及增加地图点云，回环检测  downsampleCurrentScan

1. 对来自msg的点云进行降采样
  1. laserCloudCornerLast   //角点点云
  2. laserCloudSurfLast     // 平面点点云
  3. laserCloudOutlierLast  // 离群点点云
  4. laserCloudSurfTotalLast // 平面点+离群点

流程

1. 下采样laserCloudCornerLast得到laserCloudCornerLastDS；
2. 下采样laserCloudSurfLast得到laserCloudSurfLastDS;
3. 下采样laserCloudOutlierLast得到laserCloudOutlierLastDS;
4. laserCloudSurfLastDS和laserCloudOutlierLastDS相加，得到laserCloudSurfTotalLast；
5. 下采样得到laserCloudSurfTotalLast，得到laserCloudSurfTotalLastDS;

#### 2.3.3.4 当前帧进行边缘优化，图优化以及进行LM优化的过程 scan2MapOptimization

根据现有地图与最新点云数据进行配准,从而更新机器人精确位姿与融合建图，它分为角点优化、平面点优化、配准与更新等部分。使用 scan-to-model 的策略优化了 变换 transformTobeMapped

1. 使用scan-to-model位姿优化，获得当前时间点机器人的位姿 transformTobeMapped；
2. 参考IMU的姿态 对 transformTobeMapped 进行中值滤波,获得最终的机器人位姿；
3. 为 transformBefMapped 赋值为 里程计位姿，即scan-to-model优化前的位姿；
4. 为 transformAftMapped 赋值为 transformTobeMapped 即scan-to-model优化后的位姿；

##### 2.3.3.4.1 cornerOptimization

1. 进行坐标变换,转换到全局坐标中去；
2. 进行5邻域搜索，得到结果后对搜索得到的5点求平均值；
3. 求矩阵matA1=[ax,ay,az]t*[ax,ay,az]，例如ax代表的是x-cx,表示均值与每个实际值的差值，求取5个之后再次取平均，得到matA1；
4. 求正交阵的特征值和特征向量，特征值：matD1，特征向量：保存在矩阵matV1中。因为求取的特征值是按照降序排列的，所以根据论文里面提到的：
1.如果这是一个边缘特征，则它的一个特征值远大于其余两个；
2.如果这是一个平面特征，那么其中一个特征值远小于其余两个特征值；
根据上面两个原则进行判断要不要进行优化。如果没有满足条件1，就不进行优化过程，因为这不是一个边缘特征。
5. 如果进行优化，进行优化的过程是这样的：先定义3组变量，


##### 2.3.3.4.2 surfOptimization

void surfOptimization(int)函数进行面优化，内容和函数cornerOptimization(int)的内容基本相同。
步骤如下：

1. 进行坐标变换,转换到全局坐标中去；
2. 进行5邻域搜索，得到结果后判断搜索结果是否满足条件(pointSearchSqDis[4] < 1.0)，不满足条件就不需要进行优化；
3. 将搜索结果全部保存到matA0中，形成一个5x3的矩阵；
4. 解这个矩阵cv::solve(matA0, matB0, matX0, cv::DECOMP_QR); 关于cv::solve函数，参考官网[官网](https://docs.opencv.org/ref/master/d2/de8/group__core__array.html#ga12b43690dbd31fed96f213eefead2373)
matB0是一个5x1的矩阵，需要求解的matX0是3x1的矩阵；
所以函数其实是在求解方程
matA0 ∗ matX0 = matB0  , 最后求得 matX0。 这个公式其实是在求由matA0中的点构成的平面的法向量matX0。
5. 求解得到的matX0=[pa,pb,pc,pd]，对[pa,pb,pc,pd]进行单位化， matB0=[-1,-1,-1,-1,-1]^t。
6. 误差在允许的范围内的话把这个点放到点云laserCloudOri中去，把对应的向量coeff放到coeffSel中。

##### 2.3.3.4.3 LMOptimization

这部分的代码是基于高斯牛顿法的优化，不是zhang ji论文中提到的基于L-M的优化方法
这部分的代码使用旋转矩阵对欧拉角求导，优化欧拉角，不是zhang ji论文中提到的使用angle-axis的优化


1. bool LMOptimization(int)函数是代码中最重要的一个函数，实现的功能是高斯牛顿优化(虽然写了是LMOptimization，但其实是用的高斯牛顿的方法)。
2. 首先是对laserCloudOri中数据的处理，将它放到matA中，matA就是误差对旋转和平移变量的雅克比矩阵。
3. 求完matA之后，再计算matAtA，matAtB，matX，方便后面的计算.
4. 初次优化时，特征值门限设置为100，小于这个值认为是退化了，修改matX，matX=matP*matX2
5. 最后将matX作为6个量复制到transformTobeMapped中去。
6. 在判断是否是有效的优化时，要求旋转部分的模长小于0.05m，平移部分的模长也小于0.05度。

#### 2.3.3.5 保存轨迹与位姿图,为回环检测做准备 saveKeyFramesAndFactor

程序开始：      
saveKeyFramesAndFactor(){      
1. 把上次优化得到的transformAftMapped(3:5)坐标点作为当前的位置，计算和再之前的位置的欧拉距离，距离太小并且cloudKeyPoses3D不为空(初始化时为空)，则结束；      
2. 如果是刚刚初始化，cloudKeyPoses3D为空，那么NonlinearFactorGraph增加一个PriorFactor因子， initialEstimate 的数据类型是 Values （其实就是一个map），这里在0对应的值下面保存一个Pose3，本次的 transformTobeMapped 参数保存到 transformLast 中去。      
3. 如果本次不是刚刚初始化，从transformLast得到上一次位姿，      从transformAftMapped得到本次位姿， gtSAMgraph.add(BetweenFactor),到它的约束中去， initialEstimate.insert(序号，位姿)。      
4. 不管是否是初始化，都进行优化，isam->update(gtSAMgraph, initialEstimate);   得到优化的结果：latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1), 将结果保存，cloudKeyPoses3D->push_back(thisPose3D);  cloudKeyPoses6D->push_back(thisPose6D);      
5. 对transformAftMapped进行更新;      
6. 最后保存最终的结果：      
cornerCloudKeyFrames.push_back(thisCornerKeyFrame);      
surfCloudKeyFrames.push_back(thisSurfKeyFrame);      
outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);      
}      

#### 2.3.3.6 是回环检测成功后将位姿图的数据依次更新 correctPoses


1. void correctPoses()的调用只在回环结束时进行(aLoopIsClosed == true)
2. 校正位姿的过程主要是将isamCurrentEstimate的x，y，z平移坐标更新到cloudKeyPoses3D中，另外还需要更新cloudKeyPoses6D的姿态角。
3. 关于isamCurrentEstimate：
isamCurrentEstimate是gtsam库中的Values类。

#### 2.3.3.7 发送tf变换 publishTF

1. 主题"/aft_mapped_to_init"发布优化后的位姿信息
2. msg.header.stamp 取的是 里程计时间信息
3. msg.pose 存储本坐标系位姿"/camera_init"  经过全局位姿优化的姿态
4. msg.twist 存储子坐标系位姿"/aft_mapped"  未经过全局优化的姿态,即里程计信息

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /aft_mapped_to_init |/camera_init  | /aft_mapped | 世界坐标系下 从0到 当前帧 初始时刻 的经过map优化后的位姿(参考2.2.3.2.3.1) | nav_msgs::Odometry |
| tf |/camera_init  | /aft_mapped |  | tf::StampedTransform |


#### 2.3.3.8 publishKeyPosesAndFrames

1. 如果有节点订阅"/key_pose_origin"这个话题，则用pubKeyPoses发布cloudKeyPoses3D；
2. 如果有节点订阅"/recent_cloud"这个话题，则用pubRecentKeyFrames发布laserCloudSurfFromMapDS；

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /key_pose_origin | /camera_init |  |  发布关键帧位姿点集合 | sensor_msgs::PointCloud2 |
| /recent_cloud | /camera_init |  |  所有局部关键帧平面点和离群点的集合，降采样之后 | sensor_msgs::PointCloud2 |
| /registered_cloud | /camera_init |  | 优化后世界坐标系下 当前帧 所有线点 和 面点 离群点 的集合 | sensor_msgs::PointCloud2 |

#### 2.3.3.9  clearCloud
1. 清空 laserCloudCornerFromMap 、 laserCloudSurfFromMap 、 laserCloudCornerFromMapDS 、 laserCloudSurfFromMapDS



### 2.3.4 闭环检测 loopClosureThread

主要功能是进行闭环检测和闭环修正。
关于std::thread的构造函数可以参考这里 https://cplusplus.com/reference/thread/thread/thread/



#### 2.3.4.1 开始执行闭环检测 performLoopClosure

函数流程：

1. 先进行闭环检测 detectLoopClosure()，如果返回true,则可能可以进行闭环，否则直接返回，程序结束。
2. 接着使用icp迭代进行对齐。
3. 对齐之后判断迭代是否收敛以及噪声是否太大，是则返回并直接结束函数。否则进行迭代后的数据发布处理。
4. 接下来得到 latestSurfKeyFrameCloud 和 nearHistorySurfKeyFrameCloudDS 之间的位置平移和旋转。
5. 然后进行图优化过程。

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /corrected_cloud | /camera_init |  |  map更新后的当前帧 变换到回环检测 局部点云地图 上 的点云 | sensor_msgs::PointCloud2 |




##### 2.3.4.1.1 闭环检测 detectLoopClosure

闭环检测 

| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /history_cloud | /camera_init |  |  当前帧检测的回环， 附近的历史帧周围局部点云集合 | sensor_msgs::PointCloud2 |


### 2.3.5 可视化 visualizeGlobalMapThread

1. 可视化发布当前帧 500 米 内的所有关键帧 的  点云地图 
2. 保存 当前帧 500 米 内的所有关键帧 的  点云地图 为 finalCloud.pcd
3.  保存 所有的关键帧 线点 到  cornerMap.pcd
4.  保存所有的 关键帧 面点 和 离群点 到 surfaceMap.pcd
5.  保存 所有关键帧位姿 到 trajectory.pcd


#### 2.3.5.1 publishGlobalMap

1. 通过KDTree进行最近邻搜索;
2. 通过搜索得到的索引放进队列;
3. 通过两次下采样，减小数据量;


| topic | frame_id|child_frame_id| 说明 | 消息类型 |
|  ----  | ----  | ----  | ----  | ---- |
| /laser_cloud_surround | /camera_init |  |  当前帧 500 米 内的所有关键帧 的  点云地图 | sensor_msgs::PointCloud2 |