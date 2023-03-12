// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


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

//scanRegistration的功能就是提取特征点。
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 0


/*
laserOdometry这个节点订阅了5个话题：
有序点云、极大边线点、次极大边线点、极小平面点、次极小平面点。

发布了4个话题：
有序点云、上一帧的平面点、上一帧的边线点、当前帧位姿粗估计。

主要功能是前端的激光里程计和位姿粗估计。
*/

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Lidar Odometry线程估计的frame在world坐标系的位姿P，Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0); // (q,x,y,z)
Eigen::Vector3d t_w_curr(0, 0, 0);

// 点云特征匹配时的优化变量
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

// 下面的2个分别是优化变量para_q和para_t的映射：表示的是两个world坐标系下的位姿P之间的增量，例如△P = P0.inverse() * P1
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

// 存放数据的queue
// 只能访问 queue<T> 容器适配器的第一个和最后一个元素。只能在容器的末尾添加新元素，只能从头部移除元素。
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;

// undistort lidar point
//将当前帧点云转换到上一帧，根据运动模型对点云去畸变
/*
激光雷达在运动过程中存在一定的运动畸变，即同一帧点云中，
各个点在采集时，LiDAR的位姿是不同的，就如同高速移动相机时拍摄的照片一样。

那么如何进行运动补偿呢？即将所有的点云补偿到某一个时刻。

常用的做法是补偿到起始时刻，如果有IMU，我们通过IMU得到的
雷达高频位姿，可以求出每个点相对起始点的位姿，就可以补偿回去。

如果没有IMU，我们可以使用匀速模型假设，使?上?个帧间?程记的
结果作为当前两帧之间的运动，假设当前帧也是匀速运动，可以估计出每个点相对起始时刻的位姿。

最后，当前点云中的点相对第一个点去除因运动产生的畸变，
效果相当于静止扫描得到的点云。下面是去除运动畸变的函数。

*/
/*
TransformToStart：将当前帧Lidar坐标系下的点云变换到上一帧Lidar坐标系下（也就是当前帧的初始位姿，起始位姿，
所以函数名是TransformToStart），因为kitti点云已经去除了畸变，所以不再考虑运动补偿。（如果点云没有去除畸变，
用slerp差值的方式计算出每个点在fire时刻的位姿，然后进行TransformToStart的坐标变换，一方面实现了变换到上一
帧Lidar坐标系下；另一方面也可以理解成点都将fire时刻统一到了开始时刻，即去除了畸变，完成了运动补偿）
*/
void TransformToStart(PointType const *const pi, PointType *const po)
{
    //interpolation ratio
    double s;
    // 由于kitti数据集上的lidar已经做过了运动补偿，因此这里就不做具体补偿了
    if (DISTORTION)//需要去除畸变
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;// s = 1.0说明全部补偿到点云结束的时刻
    // 所有点的操作方式都是一致的，相当于从结束时刻补偿到起始时刻，相当于是一个匀速模型的假设
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr; // 平移量插值
    Eigen::Vector3d point(pi->x, pi->y, pi->z);// 当前帧的点
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;//q_point_last ：为此帧在上一阵lider坐标系下位姿

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame
//转换所有的激光雷达指向下一帧的开始

void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    //
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}
// 之后的5个Handler函数为接受上游5个topic的回调函数，作用是将消息缓存到对应的queue中，以便后续处理。
// 操作都是送去各自的队列中，加了线程锁以避免线程数据冲突
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}
/*
laserOdometry这个节点订阅了5个话题：
        有序点云、极大边线点、次极大边线点、极小平面点、次极小平面点。

        发布了4个话题：
        有序点云、上一帧的平面点、上一帧的边线点、当前帧位姿粗估计。

        主要功能是前端的激光里程计和位姿粗估计。
*/
int main(int argc, char **argv)
{
    //1 初始化发布和订阅
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);//    设定里程计的帧率
    //if 1, do mapping 10 Hz, if 2, do mapping 5 Hz.
    printf("Mapping %d Hz \n", 10 / skipFrameNum);

    // 从scanRegistration节点订阅的消息话题，分别为极大边线点  次极大边线点   极小平面点  次极小平面点 全部点云点
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);

    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);

    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);

    // 注册发布上一帧的边线点话题
    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    // 注册发布上一帧的平面点话题
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    // 注册发布全部有序点云话题，就是从scanRegistration订阅来的点云，未经其他处理
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
    // 注册发布帧间的位姿变换话题
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    // 注册发布帧间的平移运动话题
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(100);// 循环频率
    //2 进行数据处理
/*
下面是主函数的主循环，主要是帧间位姿估计的过程，目
标是希望找到位姿变换T，使得第k帧点云左乘T得到第k+1帧点云，
或者说左乘T得到k+1帧点云的误差最小。
*/
    while (ros::ok())
    {
        ros::spinOnce();// 只触发一次回调，所以每次都要调用一次；等待回调函数执行完毕，执行一次后续代码，参考https://www.cnblogs.com/liu-fa/p/5925381.html
        //2.1 判断特征点是否有空的 // 首先确保订阅的五个消息都有，有一个队列为空都不行
        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            //2.1.1  获取特征点集合的首歌元素时间戳
            // 5个queue记录了最新的极大/次极大边线点，极小/次极小平面点，全部有序点云
            // 分别求出队列第一个时间，用来分配时间戳
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
            //2.1.2 如果关键点的时间戳和完整数据的时间戳有一个不相等，那么报错
            // 因为同一帧的时间戳都是相同的，这里比较是否是同一帧
            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync messeage!");
                ROS_BREAK();
            }
            //2.1.3 5个点云的时间同步
            // 分别将五个点云消息取出来，同时转成pcl的点云格式
            mBuf.lock();//数据多个线程使用，这里先进行加锁，避免线程冲突
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            mBuf.unlock();//数据取出来后进行解锁

            TicToc t_whole;//计算整个激光雷达里程计的时间

            // initializing 一个什么也不干的初始化，没有延迟时间，主要用来跳过第一帧数据，直接作为第二帧的上一帧
            //2.1.4 是否进行初始化
            if (!systemInited)// 第一帧不进行匹配，仅仅将 cornerPointsLessSharp 保存至 laserCloudCornerLast
                //                       将 surfPointsLessFlat    保存至 laserCloudSurfLast
                // 为下次匹配提供target
            {
                systemInited = true;
                std::cout << "Initialization finished \n";
            }
            else// 第二帧开始 特征点匹配、位姿估计
            {
                // 2.1.4.1 获取sharp和flat点的个数
                // 取出比较突出的特征点数量，极大边线点和极小平面点
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                //2.1.4.2 进行点到线以及点到面的ICP，迭代2次
                TicToc t_opt;//计算优化的时间
                // 点到线以及点到面的非线性优化，迭代2次（选择当前优化位姿的特征点匹配，并优化位姿（4次迭代），然后重新选择特征点匹配并优化）
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                {
                    //2.1.4.2.1 定义ceres优化器
                    corner_correspondence = 0;// 角点的误差项数量
                    plane_correspondence = 0;// 平面点的误差项数量

                    //ceres::LossFunction *loss_function = NULL;
                    // 定义一下ceres的核函数，使用Huber核函数来减少外点的影响，即去除outliers
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    // 由于旋转不满足一般意义的加法，因此这里使用ceres自带的local param
                    ceres::LocalParameterization *q_parameterization =
                            new ceres::EigenQuaternionParameterization();
                    ceres::Problem::Options problem_options;

                    ceres::Problem problem(problem_options);//实例化求解最优化问题
                    // 待优化的变量是帧间位姿，包括平移和旋转，这里旋转使用四元数来表示
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    TicToc t_data;//计算寻找关联点的时间

                    //2.1.4.2.2 基于最近邻原理建立corner特征点之间关联，find correspondence for corner features
                    /*
                    基于最近邻原理建立corner特征点（边线点）之间的关联，每一个极大边线点去上一帧的次极大边线点中找匹配；
                    采用边线点匹配方法:假如在第k+1帧中发现了边线点i，通过KD-tree查询在第k帧中的最近邻点j，查询j的附近扫描线上的最近邻点l，
                    j与l相连形成一条直线l-j，让点i与这条直线的距离最短。

                    构建一个非线性优化问题：以点i与直线lj的距离为代价函数，以位姿变换T(四元数表示旋转+位移t)为优化变量。下面是优化的程序。
                    */
                    // find correspondence for corner feature
                    for (int i = 0; i < cornerPointsSharpNum; ++i)//先进行线点的匹配
                    {
                        //2.1.4.2.2.1 变换sharp点，并寻找最近点
                        // 运动补偿去畸变
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);// 将当前帧的corner_sharp特征点O_cur，从当前帧的Lidar坐标系下变换到
                        //上一帧的Lidar坐标系下（记为点O，注意与前面的点O_cur不同），以利于寻找corner特征点的correspondence
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);// kdtree中的点云是上一帧的corner_less_sharp，所以这是在上一帧
                        // 的corner_less_sharp中寻找当前帧corner_sharp特征点O的最近邻点（记为A）

                        int closestPointInd = -1, minPointInd2 = -1;
                        //2.1.4.2.2.2 如果最近邻的 corner 特征点之间距离平方小于阈值，则最近邻点A有效
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            // 2.1.4.2.2.2.1 获取近邻点A的所属激光束
                            closestPointInd = pointSearchInd[0];// 目标点对应的最近距离点的索引取出来
                            // 找到其所在线束id，线束信息是intensity的整数部分
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            //2.1.4.2.2.2.2 在激光束增大的方向，寻找点O的另外一个最近邻的点（记为点B） in the direction of increasing scan line
                            // 在刚刚角点（次极大边线点）id附近扫描线分别继续寻找最邻近点
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)// laserCloudCornerLast 来自上一帧的corner_less_sharp特征点,由于提取特征时是
                            {                                                                                   // 按照scan的顺序提取的，所以laserCloudCornerLast中的点也是按照scanID递增的顺序存放的
                                //2.1.4.2.2.2.2.1 判断是否是同一个激光束上的点
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)// intensity整数部分存放的是scanID
                                    continue;

                                //2.1.4.2.2.2.2.2 判断该点是否是近邻点,即要求找到的线束距离当前线束不能太远
                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                //2.1.4.2.2.2.2.3 计算所得近邻点B与转换后的角点O之间的距离二次方
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                                //2.1.4.2.2.2.2.4 判断近邻点B与角点的距离平方是否小于阈值
                                if (pointSqDis < minPointSqDis2)// 第二个最近邻点有效,，更新点B
                                {
                                    // find nearer point
                                    //2.1.4.2.2.2.2.4.1 更新特征点之间的最小距离平方
                                    // find nearer point，寻找距离最小的角点（次极大边线点）及其索引，记录其索引
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            //2.1.4.2.2.2.3 在激光束减小的方向，寻找点O的另外一个最近邻的点B in the direction of decreasing scan line
                            // 同样另一个方向寻找对应角点（次极大边线点）
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)// 第二个最近邻点有效，更新点B
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }

                        //2.1.4.2.2.3 特征点O的两个最近邻点A和B都有效 构造误差项
                        // 如果特征点i的两个最近邻点j和m都有效，构建非线性优化问题
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        {
                            // 取出当前点和上一帧的两个角点
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);

                            double s;// 运动补偿系数， kitti 数据集的点云已经被补偿过，所以s = 1.0
                            if (DISTORTION)
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;
                            // 用点O，A，B构造点到线的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差 = 点O到直线AB的距离
                            // 具体到介绍lidarFactor.cpp时再说明该残差的具体计算方法
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;
                        }
                    }
                    //2.1.4.2.3 基于最近邻原理建立surf特征点之间关联
                    // 下面说的点符号与上述相同
                    // 与上面的建立corner特征点之间的关联类似，寻找平面特征点O的最近邻点ABC，即基于最近邻原理建立surf特征点之间的关联，find correspondence for plane features
                    /*
                    下面采用平面点匹配方法：
                            假如在第k+1帧中发现了平面点i，通过KD-tree查询在第k帧（上一帧）中的最近邻点j，
                            查询j的附近扫描线上的最近邻点l和同一条扫描线的最近邻点m，这三点确定一个平面，让点i与这个平面的距离最短；

                            构建一个非线性优化问题：以点i与平面lmj的距离为代价函数，以位姿变换T(四元数表示旋转+t)为优化变量。
                    */
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        // 先寻找上一帧距离这个面点最近的面点
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)// 找到的最近邻点A有效
                        {
                            closestPointInd = pointSearchInd[0];

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                    (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                    (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;// 找到的第2个最近邻点有效，更新点B，注意如果scanID准确的话，一般点A和点B的scanID相同
                                    minPointInd2 = j;
                                }
                                    // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;// 找到的第3个最近邻点有效，更新点C，注意如果scanID准确的话，一般点A和点B的scanID相同,且与点C的scanID不同，与LOAM的paper叙述一致
                                    minPointInd3 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                    (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                    (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)// 如果三个最近邻点都有效
                            {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                           surfPointsFlat->points[i].y,
                                                           surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                             laserCloudSurfLast->points[closestPointInd].y,
                                                             laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                             laserCloudSurfLast->points[minPointInd2].y,
                                                             laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                             laserCloudSurfLast->points[minPointInd3].y,
                                                             laserCloudSurfLast->points[minPointInd3].z);

                                double s;
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                // 用点O，A，B，C构造点到面的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差 = 点O到平面ABC的距离
                                // 同样的，具体到介绍lidarFactor.cpp时再说明该残差的具体计算方法
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;
                            }
                        }
                    }

                    // 输出寻找关联点消耗的时间
                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());
                    // 如果总的线约束和面约束太少，就打印一下
                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                    }
                    //2.1.4.2.4 使用ceres求解
                    // 调用ceres求解器求解 ，设定求解器类型，最大迭代次数，不输出过程信息，优化报告存入summary
                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 4;
                    options.minimizer_progress_to_stdout = false;
                    ceres::Solver::Summary summary;
                    // 基于构建的所有残差项，求解最优的当前帧位姿与上一帧位姿的位姿增量：para_q和para_t
                    ceres::Solve(options, &problem, &summary);
                    printf("solver time %f ms \n", t_solver.toc());
                }
                // 经过两次LM优化消耗的时间
                printf("optimization twice time %f \n", t_opt.toc());
                //2.1.4.3 新计算出的位姿增量
                // 用最新计算出的位姿增量，更新上一帧的位姿，得到当前帧的位姿，注意这里说的位姿都指的是世界坐标系下的位姿
                // 这里右式的w_curr 实际上是 w_last，即上一帧
                // 更新帧间匹配的结果，得到lidar odom位姿
                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;
            }

            TicToc t_pub;//计算发布运行时间
            // 发布lidar里程计结果
            //2.1.5 发布里程计odom话题和path
            // publish odometry
            // 创建nav_msgs::Odometry消息类型，把信息导入，并发布
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "/camera_init";//选择相机里程坐标系
            laserOdometry.child_frame_id = "/laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            // 以四元数和平移向量发出去
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);

            // geometry_msgs::PoseStamped消息是laserOdometry的部分内容
            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/camera_init";
            pubLaserPath.publish(laserPath);

            //2.1.6 将角特征和面特征变换到扫描端点
            // transform corner features and plane features to the scan end point
            // transform corner features and plane features to the scan end point
            //去畸变，没有调用
            if (0)
            {
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            //2.1.7 更新配准的source 当前帧变上一帧
            // 位姿估计完毕之后，当前边线点和平面点就变成了上一帧的边线点和平面点，把索引和数量都转移过去
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            // 使用上一帧的点云更新kd-tree，如果是第一帧的话是直接将其保存为这个的
            // kdtree设置当前帧，用来下一帧lidar odom使用
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);// 更新kdtree的点云
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

            //2.1.8 每5帧执行一次发布
            // 控制后端节点的执行频率，降频后给后端发送，只有整除时才发布结果
            if (frameCount % skipFrameNum == 0)//skipFrameNum = 2
            {
                frameCount = 0;

                // 发布上一帧的Corner点 // 发布边线点
                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                // 发布上一帧的Surf点 // 发布平面点
                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                // 发布全部完整点云 // 原封不动的转发当前帧点云
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            printf("publication time %f ms \n", t_pub.toc());
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;
        }
        rate.sleep();
    }
    return 0;
}
