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

//scanRegistration�Ĺ��ܾ�����ȡ�����㡣
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

//comp�������Ƚ������������
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

//removeClosedPointCloud����������һ���������ڵĵ�
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres)
{
    // ��������������Ʋ�ʹ��ͬһ������������Ҫ��������Ƶ�ʱ�����������С���������ͬ��
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    // �ѵ��ƾ���С�ڸ�����ֵ��ȥ����
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    // ���µ������������С
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }
    // �����Ƕ�ÿ��ɨ�����ϵĵ��ƽ���ֱͨ�˲���������õ��Ƶĸ߶�Ϊ1�����Ϊ���������ܵ���
    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

//���������Ҫ���ǶԽ��յ��ĵ������ݽ��д���
//���շ����ؼ��㣨laser_cloud_sharp��laser_cloud_less_sharp��laser_cloud_flat��laser_cloud_less_flat�����Է������ʹ�á�
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    //����������ʼ��������������ʱΪ0���൱��û����ʱ��ֻ�������˳�ʼ���Ľӿڡ�
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
    //�����Լ���Ƶļ�ʱ�࣬�Թ��캯��Ϊ��ʼʱ�䣬��toc()����Ϊ��ֹʱ�䣬������ʱ����(ms)
    TicToc t_whole;//���������ص�������ʱ��
    TicToc t_prepare;//�����״�������򻯵�ʱ��

    //ÿ���״�ɨ�����ϵĿ��Լ������ʵĵ��Ƶ����ʼ�����ͽ����������ֱ���scanStartInd�����scanEndInd�����¼
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    // �ѵ��ƴ�ros��ʽת��pcl�ĸ�ʽ
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    // ���ȶԵ����˲���ȥ��NaNֵ����Ч����
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    // ȥ���� Lidar ����ϵԭ�� MINIMUM_RANGE �������ڵĵ㣬�����������д�ĺ���
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);//MINIMUM_RANGE = 0.1

    // ������ʼ�����ֹ��Ƕ�
    // ����һ�£�atan2�ķ�Χ��[-180,180]��atan�ķ�Χ��[-90,90]
    /*
    ����Ҫ������ƽǶȷ�Χ����Ϊ��ʹ����������Ҫ���������£�
        Ϊÿ�����ҵ�������Ӧ��ɨ���ߣ�SCAN����
        Ϊÿ��ɨ�����ϵĵ����ʱ�����
    Ҫ����ÿ�����ʱ����� ����������Ҫȷ�������ĽǶȷ�Χ��
    ����ʹ��<cmath>�е�atan2( )����������Ƶ��ˮƽ�Ƕȡ�

    ����ں���ļ����ܻ���������Ҫ�ˣ�
    ��Ϊ�����״��������ÿ������ߺš��Ƕȡ�ʱ������������ˡ�
    ����˵��һ���������������
    */
    // ������ʼ��ͽ�����ĽǶȣ����ڼ����״���˳ʱ����ת������ȡ�����൱��ת������ʱ��
    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);// ��һ��ɨ���ĽǶ�
    /*
     * atan2()������atan(y�� x)��������ǿ�棬����������ȡarctran(y/x)���ܹ�ȷ������
     * startOri��endOri�ֱ�Ϊ��ʼ�����ֹ��ķ�λ��
     * atan2��Χ��[-Pi,PI]���������2PI��Ϊ�˱�֤��ʼ���������2PI����ʵ��
     */
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,     // ���һ��ɨ���ĽǶ�
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    // ������һЩ���⣬�����������3PI����С��PI������Ҫ����������Χ
    // ���һ����ĽǶ� - ��һ����ĽǶ�> PI
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

    //��ÿһ��������� scanID �͸õ��Ӧ��ɨ�赽��ʱ�䣬���������intensity�С�

    //Ϊ���Ƶ��ҵ���Ӧ��ɨ����scanID��ÿ��ɨ���߶������̶��ĸ����ǣ�
    //���ǿ��Ը��ݵ��Ƶ�Ĵ�ֱ�Ƕ�Ϊ��Ѱ�Ҷ�Ӧ��ɨ���ߡ�
    bool halfPassed = false;
    int count = cloudSize;  // ���еĵ���
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

    // ����ÿһ����
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        //ͨ�����㴹ֱ�ӳ���ȷ����������ĸ�ɨ�����ϣ�N_SCANS�߼����״
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            // �����16�߼����״�������angleӦ����-15~15֮�䣬
            //+-15��Ĵ�ֱ�ӳ�����ֱ�Ƕȷֱ���2�㣬��-15��ʱ��scanID = 0��
            scanID = int((angle + 15) / 2 + 0.5);//+0.5��Ϊ������ȡ��
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

        // ��ȡÿһ�����Ӧ�ĽǶ�
        float ori = -atan2(point.y, point.x);// ÿһ�����ˮƽ�Ƕ�
        // ����ɨ�����Ƿ���ת����ѡ������ʼλ�û�����ֹλ�ý��в�ֵ���㣬
        //�Ӷ����в����������ʱɨ��û�й��룬��halfPassedΪfalse
        if (!halfPassed) // �жϵ�ǰ���û��һ�루��ʼ��ʱ����û��һ��ģ�
        {
            // ȷ��-PI / 2 < ori - startOri < 3 / 2 * PI��
            //���ori-startOriС��-0.5pi�����1.5pi�������ori�ĽǶ�
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }
            //ɨ���������趨halfPassedΪtrue���������180�ȣ���˵������һ����
            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else// ����ʱ��
        {
            // ȷ��-PI * 3 / 2 < ori - endOri < PI / 2
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
         * relTime ��һ��0~1֮���С��������ռ��һ֡ɨ��ʱ��ı���������ɨ��ʱ��õ���ʵɨ��ʱ�̣�
         * scanPeriod ɨ��ʱ��Ĭ��Ϊ0.1s
         * �Ƕȵļ�����Ϊ�˼��������ʼʱ�̵�ʱ��
         */
        float relTime = (ori - startOri) / (endOri - startOri);// ʱ��ռ��
        // ����������scan������������С�������������ʼʱ�̵�ʱ��
        point.intensity = scanID + scanPeriod * relTime;// intensity=scanID+���Ӧ�������ϵ�ʱ��
        laserCloudScans[scanID].push_back(point); // ����ÿ���ߵ�idx����������飬��ʾ��һ��ɨ�����ϵĵ�
    }
    // cloudSize ����Ч�ĵ��Ƶ���Ŀ
    cloudSize = count;
    printf("points size %d \n", cloudSize);



    // ��¼16��scan��ÿһ���� startInd��endInd
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    // ȫ�����ϵ�һ����������ȥ������ʹ��������������ʼ�ͽ����
    //����ֱ�+5��-6��Ϊ�˼������ʷ���
    for (int i = 0; i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;// ��¼ÿ��scan�Ŀ�ʼindex������ǰ5����
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;// ��¼ÿ��scan�Ľ���index�����Ժ�5���㣬��ʼ�ͽ������ĵ���scan���ײ������պϵġ��ӷ족������ȡedge feature����
    }
    // ��һ֡�������ת��������������ĵ�ʱ�䣬����ָ����ǰ�洦���״����ݵ�ʱ��
    printf("prepare time %f \n", t_prepare.toc());
    // ����ؼ�������
    // ����ÿһ��������ʣ������laserCloud������ĵ��ƣ��ʿ���ֱ����������
    // ������ÿ��scan�Ľ��紦����õ��������ǲ�׼ȷ�ģ����ͨ��scanStartInd[i]��scanEndInd[i]��ѡȡ
    for (int i = 5; i < cloudSize - 5; i++)// ����ǰ���5����
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        //�洢ÿ��������ʵ�����
        /*
         * cloudSortInd[i] = i�൱�����е�ĳ�ʼ��Ȼ���У�ÿ����õ����Լ�����ţ�������
         * ����ÿ���㣬ѡ������������������������ʼ��Ϊ0��
         * ÿ����ĵ����ͳ�ʼ����Ϊ0���μ�Сƽ��㣩
         */
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;// ����û�б�ѡѡ��Ϊfeature��
        cloudLabel[i] = 0;// Label 2: corner_sharp
        // Label 1: corner_less_sharp, ����Label 2
        // Label -1: surf_flat
        // Label 0: surf_less_flat�� ����Label -1����Ϊ��̫�࣬���ή����
    }


/*
���ʼ�����ɺ�����������࣬��ȡ�������м���ԭ��

1.Ϊ�����Ч�ʣ�ÿ��ɨ���߷ֳ�6����������ÿ�������ڣ�Ѱ����������20���㣬
    ��Ϊ�μ�����ߵ㣬��������2���㣬ͬʱ��Ϊ������ߵ㣻

2. Ѱ��������С��4���㣬��Ϊ��Сƽ��㣬ʣ��δ����ǵĵ㣬ȫ����Ϊ�μ�Сƽ��㡣

3. Ϊ�˷�ֹ���������۶ѣ�ÿ��ȡһ�������㣨����/�μ�����ߵ㣬��Сƽ��㣩��
    ��Ҫ���������������ĵ�ȫ�����Ϊ����ѡ�С������´���ȡ������ʱ������������Щ�㡣
    ���ڴμ�Сƽ��㣬��ȡ�������ķ����������۶ѡ�
*/
    TicToc t_pts;//����������ȡ��ʱ��
    pcl::PointCloud<PointType> cornerPointsSharp;// ������ߵ�
    pcl::PointCloud<PointType> cornerPointsLessSharp;// �μ�����ߵ�
    pcl::PointCloud<PointType> surfPointsFlat;// ��Сƽ���
    pcl::PointCloud<PointType> surfPointsLessFlat;// �μ�Сƽ��
    // ��ȡ������
    float t_q_sort = 0;// ������¼���򻨷ѵ���ʱ��

    // ����ÿ��scan����ÿ���߽��в�������������ѡȡ��Ӧ�����㣩
    for (int i = 0; i < N_SCANS; i++)// ����scan��˳����ȡ4��������
    {
        // ������һ���������ʵĵ����һ����������С��6��˵���޷��ֳ�6������������
        if( scanEndInd[i] - scanStartInd[i] < 6)// �����scanx�ߵĵ�������7���㣬������
            continue;
        // �����洢�μ�Сƽ��㣬�������н�����
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        //Ϊ��ʹ��������ȷֲ�����һ��scan�߷ֳ�6������
        for (int j = 0; j < 6; j++)// ���� scan �߷ֳ�6С��ִ���������
        {
            // ÿ���ȷֵ���ʼ�ͽ�����
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;// subscan����ʼindex
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;// subscan�Ľ���index

            TicToc t_tmp;//��������ʱ��
            // �Ե��ư������ʽ�����������С����ǰ������ں�
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            // t_q_sort�ۼ�ÿ��������������ʱ���ܺ�
            t_q_sort += t_tmp.toc();

            // ѡȡ������ߵ㣨2�����ʹμ�����ߵ㣨20����
            int largestPickedNum = 0;
            // ��ѡ���ʱȽϴ�Ĳ��֣��������������С���ʱ�����Ѱ�ұ��ߵ㣬��Ҫ�����0.1
            for (int k = ep; k >= sp; k--)// �Ӻ���ǰ���������ʴ�ĵ㿪ʼ��ȡcorner feature
            {
                // �����˳������ˣ����ʱ�����������þ����ֳ�����
                int ind = cloudSortInd[k];

                // ����������Ƿ�����Ч�㣬ͬʱ�����Ƿ������ֵ����û��ѡ�� && ���� > 0.1
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)// ����õ�û�б�ѡ������������ʴ���0.1
                {
                    largestPickedNum++;
                    // ÿ��ѡ2�����ʴ�ĵ�
                    if (largestPickedNum <= 2)// �� subscan ����������ǰ2������Ϊ�� corner_sharp ������
                    {
                        // label Ϊ2�����ʴ�ı�ǣ������ñ�ǩΪ������ߵ�
                        cloudLabel[ind] = 2;
                        // cornerPointsSharp������Ŵ����ʵĵ㣬�ȷ��뼫����ߵ㣬Ҳ����μ�����ߵ�
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                        // �Լ�20��������΢��һЩ�ĵ�
                    else if (largestPickedNum <= 20)// ��subscan����������ǰ20������Ϊ��corner_less_sharp������
                    {
                        // label��1��ʾ������΢��һЩ������2��ѡ����Ժ�����Ϊ�μ�����ߵ㣬����μ�����ߵ�����
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                        // ����20��������
                    else
                    {
                        break;
                    }
                    // ����㱻ѡ�к�pick��־λ��1 ����¼������Ѿ���ѡ����
                    cloudNeighborPicked[ind] = 1;// ��Ǹõ㱻ѡ�����

                    // �뵱ǰ������ƽ�� <= 0.05�ĵ���Ϊѡ����������������ܼ��ֲ�
                    // Ϊ�˱�֤�����㲻���ȼ��У���ѡ�еĵ���Χ5���㶼��1,���������ѡ��
                    for (int l = 1; l <= 5; l++)
                    {
                        // �鿴���ڵ�����Ƿ�����������������˵�������ڴ˲�������
                        // ��������Ե���ͻ����µ���������˾Ͳ���λ��
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        // ����scan������ƽ�� <= 0.05�ĵ���Ϊѡ����������������ܼ��ֲ�
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        // ���ٽ���ľ����ƽ�� <= 0.05�ĵ���Ϊѡ����������������ܼ��ֲ���Խ����scanIDԽ��ԽԶ�����Կ���ֱ��break
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
            // ���濪ʼ��ѡ��㣬ѡȡ��Сƽ��㣨4����
            // ��ȡsurfƽ��feature�����������ƣ�ѡȡ��subscan������С��ǰ4����Ϊsurf_flat
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++) // Ѱ��ƽ�ģ�������С��������
            {
                int ind = cloudSortInd[k];
                // �ж��Ƿ�����ƽ��������
                // ȷ�������û�б�pick������С����ֵ
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    // -1��Ϊ��ƽ̹�ĵ�
                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);
                    // ���ﲻ����ƽ̹�ͱȽ�ƽ̹����Ϊʣ�µĵ�labelĬ����0,���ǱȽ�ƽ̹
                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    // ����scanID���ϲ��ң���ֹ�ؼ����ܼ�
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


            // �����ķ�corner��������surf_flat������һ�����surf_less_flat������
            // ѡȡ�μ�Сƽ��㣬���˼�Сƽ��㣬ʣ�µĶ��Ǵμ�Сƽ���
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        // ���Ը� scan ��������ȡ������ surf_less_flat ��������н���������Ϊ��̫����
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());


    sensor_msgs::PointCloud2 laserCloudOutMsg;// ����publish msgʵ��
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);// �������ת��Ϊmsg
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;// ʱ������ֲ���
    laserCloudOutMsg.header.frame_id = "/camera_init"; // frame_id���֣�����ϵ
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
    // ���԰���ÿ��scan����ȥ������������false
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
    ros::init(argc, argv, "scanRegistration"); //�ڵ�����
    ros::NodeHandle nh; //ע��ROS���

    //��launch�ļ������������л�ȡ�����ߵļ����״���û����Ĭ��16��
    nh.param<int>("scan_line", N_SCANS, 16);

    // ��launch�ļ������������л�ȡ�����״����Сɨ�����MINIMUM_RANGE��
    //С�� MINIMUM_RANGE �ĵ㽫���˳�����λΪM�����û����Ĭ��0.1��
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    // ֻ��������16��32��64�Ĳſ��Լ���
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }
    // ���ĳ�ʼ�ļ����״����ݣ���ע��ص�����laserCloudHandler
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);// TODO_czy   nsh_indoor_outdoor.bag
    //ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 100, laserCloudHandler);// TODO_czy   nsh_indoor_outdoor.bag
    // �������⣺������ƣ�ɾ�������㡢������������������ߵ㼯�ϣ��μ�����ߵ㼯�ϣ���Сƽ��㼯�ϣ��μ�Сƽ��㼯��,ɾ���ĵ���
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    if(PUB_EACH_LINE)//�������û�н�ȥ
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();// ѭ��ִ�лص�����

    return 0;
}



