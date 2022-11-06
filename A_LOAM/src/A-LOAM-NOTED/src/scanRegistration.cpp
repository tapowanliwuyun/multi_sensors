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
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;

const int systemDelay = 0;
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

//comp函数，比较两个点的曲率
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1;

//removeClosedPointCloud函数，丢弃一定距离以内的点
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres)
{
    // 假如输入输出点云不使用同一个变量，则需要将输出点云的时间戳和容器大小与输入点云同步
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    // 把点云距离小于给定阈值的去除掉
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    // 重新调整输出容器大小
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }
    // 这里是对每条扫描线上的点云进行直通滤波，因此设置点云的高度为1，宽度为数量，稠密点云
    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

//这个函数主要就是对接收到的点云数据进行处理，
//最终发布关键点（laser_cloud_sharp、laser_cloud_less_sharp、laser_cloud_flat、laser_cloud_less_flat），以方便后续使用。
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    //下面是做初始化，但是这里延时为0，相当于没有延时，只是留下了初始化的接口。
    if (!systemInited)
    {
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }
    //作者自己设计的计时类，以构造函数为起始时间，以toc()函数为终止时间，并返回时间间隔(ms)
    TicToc t_whole;//计算整个回调函数的时间
    TicToc t_prepare;//计算雷达点云有序化的时间

    //每条雷达扫描线上的可以计算曲率的点云点的起始索引和结束索引，分别用scanStartInd数组和scanEndInd数组记录
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    // 把点云从ros格式转到pcl的格式
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    // 首先对点云滤波，去除NaN值得无效点云
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    // 去除在 Lidar 坐标系原点 MINIMUM_RANGE 距离以内的点，这里调用作者写的函数
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);//MINIMUM_RANGE = 0.1

    // 计算起始点和终止点角度
    // 解释一下：atan2的范围是[-180,180]，atan的范围是[-90,90]
    /*
    下面要计算点云角度范围，是为了使点云有序，需要做到两件事：
        为每个点找到它所对应的扫描线（SCAN）；
        为每条扫描线上的点分配时间戳。
    要计算每个点的时间戳， 首先我们需要确定这个点的角度范围。
    可以使用<cmath>中的atan2( )函数计算点云点的水平角度。

    这块在后面的激光框架基本都不需要了，
    因为后面雷达的驱动将每个点的线号、角度、时间戳都发出来了。
    所以说这一块可以跳过不看。
    */
    // 计算起始点和结束点的角度，由于激光雷达是顺时针旋转，这里取反就相当于转成了逆时针
    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);// 第一个扫描点的角度
    /*
     * atan2()函数是atan(y， x)函数的增强版，不仅可以求取arctran(y/x)还能够确定象限
     * startOri和endOri分别为起始点和终止点的方位角
     * atan2范围是[-Pi,PI]，这里加上2PI是为了保证起始到结束相差2PI符合实际
     */
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,     // 最后一个扫描点的角度
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    // 但总有一些例外，比如这里大于3PI，和小于PI，就需要调整到合理范围
    // 最后一个点的角度 - 第一个点的角度> PI
    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);
    //printf("[scanRegistration] start Ori %f\n",startOri);
    //printf("[scanRegistration] end Ori %f\n", endOri);

    //对每一个点计算其 scanID 和该点对应的扫描到的时间，将结果放在intensity中。

    //为点云点找到对应的扫描线scanID，每条扫描线都有它固定的俯仰角，
    //我们可以根据点云点的垂直角度为其寻找对应的扫描线。
    bool halfPassed = false;
    int count = cloudSize;  // 所有的点数
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

    // 遍历每一个点
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        //通过计算垂直视场角确定激光点在哪个扫描线上（N_SCANS线激光雷达）
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            // 如果是16线激光雷达，结算出的angle应该在-15~15之间，
            //+-15°的垂直视场，垂直角度分辨率2°，则-15°时的scanID = 0。
            scanID = int((angle + 15) / 2 + 0.5);//+0.5是为了向上取整
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        // 获取每一个点对应的角度
        float ori = -atan2(point.y, point.x);// 每一个点的水平角度
        // 根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，
        //从而进行补偿，如果此时扫描没有过半，则halfPassed为false
        if (!halfPassed) // 判断当前点过没过一半（开始的时候是没过一半的）
        {
            // 确保-PI / 2 < ori - startOri < 3 / 2 * PI，
            //如果ori-startOri小于-0.5pi或大于1.5pi，则调整ori的角度
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }
            //扫描点过半则设定halfPassed为true，如果超过180度，就说明过了一半了
            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else// 后半段时间
        {
            // 确保-PI * 3 / 2 < ori - endOri < PI / 2
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        /*
         * relTime 是一个0~1之间的小数，代表占用一帧扫描时间的比例，乘以扫描时间得到真实扫描时刻，
         * scanPeriod 扫描时间默认为0.1s
         * 角度的计算是为了计算相对起始时刻的时间
         */
        float relTime = (ori - startOri) / (endOri - startOri);// 时间占比
        // 整数部分是scan线束的索引，小数部分是相对起始时刻的时间
        point.intensity = scanID + scanPeriod * relTime;// intensity=scanID+点对应的在线上的时间
        laserCloudScans[scanID].push_back(point); // 根据每条线的idx送入各自数组，表示这一条扫描线上的点
    }
    // cloudSize 是有效的点云的数目
    cloudSize = count;
    printf("points size %d \n", cloudSize);



    // 记录16个scan的每一个的 startInd，endInd
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    // 全部集合到一个点云里面去，但是使用两个数组标记起始和结果，
    //这里分别+5和-6是为了计算曲率方便
    for (int i = 0; i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;// 记录每个scan的开始index，忽略前5个点
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;// 记录每个scan的结束index，忽略后5个点，开始和结束处的点云scan容易产生不闭合的“接缝”，对提取edge feature不利
    }
    // 将一帧无序点云转换成有序点云消耗的时间，这里指的是前面处理雷达数据的时间
    printf("prepare time %f \n", t_prepare.toc());
    // 有序地计算曲率
    // 计算每一个点的曲率，这里的laserCloud是有序的点云，故可以直接这样计算
    // 但是在每条scan的交界处计算得到的曲率是不准确的，这可通过scanStartInd[i]、scanEndInd[i]来选取
    for (int i = 5; i < cloudSize - 5; i++)// 忽略前后的5个点
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        //存储每个点的曲率的索引
        /*
         * cloudSortInd[i] = i相当于所有点的初始自然序列，每个点得到它自己的序号（索引）
         * 对于每个点，选择了它附近的特征点数量初始化为0，
         * 每个点的点类型初始设置为0（次极小平面点）
         */
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;// 点有没有被选选择为feature点
        cloudLabel[i] = 0;// Label 2: corner_sharp
        // Label 1: corner_less_sharp, 包含Label 2
        // Label -1: surf_flat
        // Label 0: surf_less_flat， 包含Label -1，因为点太多，最后会降采样
    }


/*
曲率计算完成后进行特征分类，提取特征点有几点原则：

1.为了提高效率，每条扫描线分成6个扇区，在每个扇区内，寻找曲率最大的20个点，
    作为次极大边线点，其中最大的2个点，同时作为极大边线点；

2. 寻找曲率最小的4个点，作为极小平面点，剩下未被标记的点，全部作为次极小平面点。

3. 为了防止特征点过多聚堆，每提取一个特征点（极大/次极大边线点，极小平面点），
    都要将这个点和它附近的点全都标记为“已选中”，在下次提取特征点时，将会跳过这些点。
    对于次极小平面点，采取降采样的方法避免过多聚堆。
*/
    TicToc t_pts;//计算特征提取的时间
    pcl::PointCloud<PointType> cornerPointsSharp;// 极大边线点
    pcl::PointCloud<PointType> cornerPointsLessSharp;// 次极大边线点
    pcl::PointCloud<PointType> surfPointsFlat;// 极小平面点
    pcl::PointCloud<PointType> surfPointsLessFlat;// 次极小平面
    // 提取特征点
    float t_q_sort = 0;// 用来记录排序花费的总时间

    // 遍历每个scan，对每条线进行操作（曲率排序，选取对应特征点）
    for (int i = 0; i < N_SCANS; i++)// 按照scan的顺序提取4种特征点
    {
        // 如果最后一个可算曲率的点与第一个的数量差小于6，说明无法分成6个扇区，跳过
        if( scanEndInd[i] - scanStartInd[i] < 6)// 如果该scanx线的点数少于7个点，就跳过
            continue;
        // 用来存储次极小平面点，后面会进行降采样
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        //为了使特征点均匀分布，将一个scan线分成6个扇区
        for (int j = 0; j < 6; j++)// 将该 scan 线分成6小段执行特征检测
        {
            // 每个等分的起始和结束点
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;// subscan的起始index
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;// subscan的结束index

            TicToc t_tmp;//计算排序时间
            // 对点云按照曲率进行升序排序，小的在前，大的在后
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            // t_q_sort累计每个扇区曲率排序时间总和
            t_q_sort += t_tmp.toc();

            // 选取极大边线点（2个）和次极大边线点（20个）
            int largestPickedNum = 0;
            // 挑选曲率比较大的部分，从最大曲率往最小曲率遍历，寻找边线点，并要求大于0.1
            for (int k = ep; k >= sp; k--)// 从后往前，即从曲率大的点开始提取corner feature
            {
                // 排序后顺序就乱了，这个时候索引的作用就体现出来了
                int ind = cloudSortInd[k];

                // 看看这个点是否是有效点，同时曲率是否大于阈值，即没被选过 && 曲率 > 0.1
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)// 如果该点没有被选择过，并且曲率大于0.1
                {
                    largestPickedNum++;
                    // 每段选2个曲率大的点
                    if (largestPickedNum <= 2)// 该 subscan 中曲率最大的前2个点认为是 corner_sharp 特征点
                    {
                        // label 为2是曲率大的标记，即设置标签为极大边线点
                        cloudLabel[ind] = 2;
                        // cornerPointsSharp用来存放大曲率的点，既放入极大边线点，也放入次极大边线点
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                        // 以及20个曲率稍微大一些的点
                    else if (largestPickedNum <= 20)// 该subscan中曲率最大的前20个点认为是corner_less_sharp特征点
                    {
                        // label置1表示曲率稍微大一些，超过2个选择点以后，设置为次极大边线点，放入次极大边线点容器
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                        // 超过20个就跳过
                    else
                    {
                        break;
                    }
                    // 这个点被选中后pick标志位置1 ，记录这个点已经被选择了
                    cloudNeighborPicked[ind] = 1;// 标记该点被选择过了

                    // 与当前点距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布
                    // 为了保证特征点不过度集中，将选中的点周围5个点都置1,避免后续会选到
                    for (int l = 1; l <= 5; l++)
                    {
                        // 查看相邻点距离是否差异过大，如果差异过大说明点云在此不连续，
                        // 是特征边缘，就会是新的特征，因此就不置位了
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        // 相邻scan点距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        // 与临近点的距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布，越往上scanID越大越远，所以可以直接break
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // 下面开始挑选面点，选取极小平面点（4个）
            // 提取surf平面feature，与上述类似，选取该subscan曲率最小的前4个点为surf_flat
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++) // 寻找平的，曲率由小到大排序
            {
                int ind = cloudSortInd[k];
                // 判断是否满足平面点的条件
                // 确保这个点没有被pick且曲率小于阈值
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    // -1认为是平坦的点
                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);
                    // 这里不区分平坦和比较平坦，因为剩下的点label默认是0,就是比较平坦
                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    // 根据scanID往上查找，防止关键点密集
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }


            // 其他的非corner特征点与surf_flat特征点一起组成surf_less_flat特征点
            // 选取次极小平面点，除了极小平面点，剩下的都是次极小平面点
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        // 最后对该 scan 点云中提取的所有 surf_less_flat 特征点进行降采样，因为点太多了
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());


    sensor_msgs::PointCloud2 laserCloudOutMsg;// 创建publish msg实例
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);// 有序点云转化为msg
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;// 时间戳保持不变
    laserCloudOutMsg.header.frame_id = "/camera_init"; // frame_id名字，坐标系
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // pub each scam
    // 可以按照每个scan发出去，不过这里是false
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "/camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration"); //节点名称
    ros::NodeHandle nh; //注册ROS句柄

    //从launch文件参数服务器中获取多少线的激光雷达，如果没有则默认16线
    nh.param<int>("scan_line", N_SCANS, 16);

    // 从launch文件参数服务器中获取激光雷达的最小扫描距离MINIMUM_RANGE，
    //小于 MINIMUM_RANGE 的点将被滤除，单位为M，如果没有则默认0.1。
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    // 只有线束是16、32、64的才可以继续
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }
    // 订阅初始的激光雷达数据，并注册回调函数laserCloudHandler
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);// TODO_czy   nsh_indoor_outdoor.bag
    //ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 100, laserCloudHandler);// TODO_czy   nsh_indoor_outdoor.bag
    // 发布话题：有序点云（删除过近点、设置索引），极大边线点集合，次极大边线点集合，极小平面点集合，次极小平面点集合,删除的点云
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    if(PUB_EACH_LINE)//这个条件没有进去
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();// 循环执行回调函数

    return 0;
}



