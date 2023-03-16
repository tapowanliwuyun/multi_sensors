// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"
#include <iostream>
class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image 距离图像的距离矩阵
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed 数组用于广度优先的分段搜索过程，以提高速度
    uint16_t *queueIndY;

public:
    ImageProjection():
        nh("~"){
        // 订阅原始的激光点云
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);
        // 转换成图片的点云
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
         // 转换成图片的并带有距离信息的点云
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);
        // 发布提取的地面特征
        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        // 发布已经分割的点云
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        // 具有几何信息的分割点云
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        // 含有异常信息的点云
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());//输入的点云
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());//velodyne雷达点云的格式

        fullCloud.reset(new pcl::PointCloud<PointType>());//
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);// 默认非地点false
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));//默认是0，也就是非地面点
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));//点的聚类类别，-1是地面点的类别，0是默认的，999999是参与聚类但不是有效聚类的类别
        labelCount = 1;//类别起始从标签1开始

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);//初始化 fullCloud ，每一点都是（0，0，0，-1）
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);//初始化 fullInfoCloud ，每一点都是（0，0，0，-1）
    }

    ~ImageProjection(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        //1、读取ROS点云转换为PCL点云
        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // 2.移除无效的点云 Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // 3、have "ring" channel in the cloud
        // 如果点云有"ring"通过，则保存为laserCloudInRing
        // 判断是不是使用了 velodyne 的激光雷达
        if (useCloudRing == true){
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {//如果移除了无效点，就会设置is_dense为true
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }  
        }
    }

    // 订阅激光雷达点云信息之后的回调函数
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. Convert ros message to pcl point cloud 转换ros消息格式为pcl库的点云格式
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan  找到开始时刻和结束时刻的方向角度
        findStartEndAngle();
        // 3. Range image projection 将点云信息投影到 16 * 1800 分辨率的图像上（点云阵列上）
        projectPointCloud();
        // 4. Mark ground points 根据上下线束 俯仰角 判断是否是 地面  （角度小于10度 为地面），滤除地面点
        groundRemoval();
        // 5. Point cloud segmentation 点云分割 首先对点云进行聚类标记 然后通过聚类完成的标签 对点云分块存储
        cloudSegmentation();
        // 6. Publish all clouds发布各类型的点云
        publishCloud(); 
        // 7. Reset parameters for next iteration 重启
        resetParameters();
    }

    void findStartEndAngle(){
        // start and end orientation of this cloud
        // 1.开始点和结束点的航向角 (负号表示逆时针旋转)   
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        // 加 2 * M_PI 表示已经转转了一圈
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        // 2.保证 所有角度 落在 [M_PI , 3M_PI] 上 
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    // 3.点云消息处理成图像方式的阵列 将一帧点云变成 16* 1800 的图片
    //  rangeMat.at<float>(rowIdn, columnIdn) = range; 关键的代码  
    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;//垂直角，水平角
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;//当前点

        cloudSize = laserCloudIn->points.size();

        //遍历整个点云
        for (size_t i = 0; i < cloudSize; ++i){

            //1、提取点云中 x y z 坐标数值
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            // 2、判断是不是使用了 velodyne 的雷达，并根据此来提取当前点所在的线束
            if (useCloudRing == true){
                // 提取激光雷达线束到 rowIdn 
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                //  是其他的雷达 就通过俯仰角 确定当前的激光点是来自哪个线束 index 
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;//ang_bottom = 15.0+0.1;ang_res_y = 2.0;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            //3、计算当前点的水平角，并判断其位置（0～1800）
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;//计算水平角

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;//ang_res_x = 0.2;Horizon_SCAN = 1800;round()为四舍五入
            if (columnIdn >= Horizon_SCAN)//Horizon_SCAN = 1800;
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            //4、计算当前点到激光雷达传感器的距离，要大于1.0，数值保存到 rangeMat 中 
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);// 如果距离小于 1米 则过滤掉 通常是过滤自身（距离传感器比较近的点）
            if (range < sensorMinimumRange)//sensorMinimumRange = 1.0;
                continue;
            
            //5、定义距离图像为16✖1800
            rangeMat.at<float>(rowIdn, columnIdn) = range;
            //6、将 index 和 横坐标存储在 intensity 中 整数部分是线束数值  小数部分是方向角度
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;//点的强度重新定义为：整数部分为其所在的线束，小数部分为其所在的水平位置
            //7、深度图的索引值  index = 列号 +  行号 * 1800 
            index = columnIdn  + rowIdn * Horizon_SCAN;//
            //8、将当前点以两种方式存储
            fullCloud->points[index] = thisPoint; // 个人理解是 fullCloud 存放的是坐标信息，此时的thisPoint有x、y、z、I信息，I为其在一帧中的位置
            fullInfoCloud->points[index] = thisPoint;// fullInfoCloud 增加了点到传感器的距离信息 ，I为其距激光传感器的位置
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }

    //4.滤除地面点，将下7条扫描线且相邻两个线束之间的夹角小于10度的点标记为地面点

    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not 没有有效的信息判断是否为地面点
        //  0, initial value, after validation, means not ground    默认值，意味着非地面点
        //  1, ground   地面点
        for (size_t j = 0; j < Horizon_SCAN; ++j){//Horizon_SCAN = 1800;
            //  前7个激光雷达扫描线束足够满足地面点的检测 所以只遍历 7 次
            for (size_t i = 0; i < groundScanInd; ++i){//groundScanInd = 7;

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;
                // 如果之前计算过的 intensity 是 -1 则直接默认为是一个无效点
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                    
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;
                // 计算相邻两个线束之间的夹角 
                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
                // 如果夹角数值小于 10 度， 则可以判断为平面
                if (abs(angle - sensorMountAngle) <= 10){//sensorMountAngle = 0.0;
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1) 提取地面云（groundMat==1）
        // mark entry that doesn't need to label (ground and invalid point) for segmentation 标记不需要标记的条目（地面和无效点）以进行分割
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan 请注意，从0~N_SCAN-1删除地面，需要第16次扫描的标记标签矩阵的rangeMat
        // 给地面点 标记一个符号 为 -1 
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){//FLT_MAX为默认值
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        // 如果有topic订阅地面点云信息，则进行处理
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){//groundScanInd = 7;
                for (size_t j = 0; j < Horizon_SCAN; ++j){//Horizon_SCAN = 1800;
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }
    }
    //5、分割点云
    void cloudSegmentation(){
        // segmentation process 分割过程
        for (size_t i = 0; i < N_SCAN; ++i)// 16线的 一个线束一个的遍历
            for (size_t j = 0; j < Horizon_SCAN; ++j)// 水平 的 1800
                // 如果labelMat[i][j]=0,表示没有对该点进行过分类，需要对该点进行聚类
                if (labelMat.at<int>(i,j) == 0) // 对非地面有效点进行点云分割
                    labelComponents(i, j);//  调用这个函数 对点云进行分割 聚类

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
         // 提取分割点云 用于激光雷达里程计
        for (size_t i = 0; i < N_SCAN; ++i) { //N_SCAN = 16;

            // 表示第i线的点云起始序列和终止序列，开始4点和末尾6点舍去不要
            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){ //找到可用的 特征点 或者 地面点
                    // outliers that will not be used for optimization (always continue)
                    // 勾勒出优化过程中不被使用的值

                    // 1. 如果 label 为 999999 则跳过
                    // labelMat 数值为 999999 表示这个点是因为聚类数量不够30而被舍弃的点
                    // 需要舍弃的点直接continue跳过本次循环，
                    // 当列数为5的倍数，并且行数较大，可以认为非地面点的，将它保存进异常点云(界外点云)中；也就是对于scan大于7，每五个点选择一个存入 outlierCloud 中
                    // 然后再跳过本次循环
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){ //groundScanInd = 7;
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    // 2. 如果为地，跳过index不是5的倍数的点；可以理解为地点，每五个选一个；
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5) //Horizon_SCAN = 1800;
                            continue;
                    }
                    // 上面多个if语句已经去掉了不符合条件的点，这部分直接进行信息的拷贝和保存操作
                    // 保存完毕后sizeOfSegCloud递增
                    // mark ground points so they will not be considered as edge features later 标记接地点，以便以后不再将其视为边特征点
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later 标记点的列索引，以便稍后标记遮挡
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }
        
        // 如果有节点订阅SegmentedCloudPure,
        // 那么把点云数据保存到segmentedCloudPure中去
        // extract segmented cloud for visualization 提取分段点云以实现可视化
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    // 需要选择不是地面点(labelMat[i][j]!=-1)和没被舍弃的点
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};
        // 传进来的两个参数，按照坐标不同 分别给他们放到 X 与 Y 的数组中
        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;// 需要计算角度的点的数量
        int queueStartInd = 0;
        int queueEndInd = 1;
        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;//类别标签
            // Loop through all the neighboring grids of popped grid
            // 遍历整个点云 遍历前后左右四个点,求点之间的角度数值 
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)//如果该点已经划分过类别
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;//segmentAlphaX = ang_res_x / 180.0 * M_PI;ang_res_x = 0.2;
                else
                    alpha = segmentAlphaY;//segmentAlphaY = ang_res_y / 180.0 * M_PI;ang_res_y = 2.0;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));//可以查看文章https://zhuanlan.zhihu.com/p/72932303
                //判断是否属于一类，如果大于这个阈值，说明属于同一类
                if (angle > segmentTheta){ //segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;//用来标记垂直方向上此聚类的占比

                    allPushedIndX[allPushedIndSize] = thisIndX;//所有参与聚类的点
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }
        //std::cout << "allPushedIndSize = " << allPushedIndSize << std::endl;
        // check if this segment is valid
        bool feasibleSegment = false;
        // 如果是 allPushedIndSize 累加的数量增加到了30 个 则判断这部分点云属于一个聚类
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        // 如果垂直 方向上点的数量大于 5个 默认是一个有效的聚类
        else if (allPushedIndSize >= segmentValidPointNum){//segmentValidPointNum = 5;
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)//segmentValidLineNum = 3;
                feasibleSegment = true;            
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
        }else{ // segment is invalid, mark these points  如果不是有效的聚类，就把当前所有的点分到999999类中
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    
    void publishCloud(){
        // 1. Publish Seg Cloud Info
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
