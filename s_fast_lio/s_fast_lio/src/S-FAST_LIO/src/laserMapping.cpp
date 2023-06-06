#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#include "IMU_Processing.hpp"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
//add_point_size为添加点的数量，kdtree_delete_counter为删除点的数量
int    add_point_size = 0, kdtree_delete_counter = 0;
//pcd_save_en是否保存pcd文件，time_sync_en是否同步时间
bool   pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};//残差，点到面距离平方和
float DET_RANGE = 300.0f;//设置的当前雷达系中心到各个地图边缘的距离
const float MOV_THRESHOLD = 1.5f; //设置的当前雷达系中心到各个地图边缘的权重
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;// 互斥锁
condition_variable sig_buffer;// 条件变量

string root_dir = ROOT_DIR;  //设置根目录
string map_file_path, lid_topic, imu_topic;//设置地图文件路径，雷达topic，imu topic

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;  //设置雷达时间戳，imu时间戳
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;   //设置imu的角速度协方差，加速度协方差，角速度协方差偏置，加速度协方差偏置
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;//设置滤波器的最小尺寸，地图的最小尺寸，视野角度

//设置立方体长度，雷达结束时间，雷达初始时间
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;

// scan_count：接收到的激光雷达Msg的总数，publish_count：接收到的IMU的Msg的总数
int    scan_count = 0, publish_count = 0;

//下采样的点数，最大迭代次数
int    feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;

// lidar_pushed：用于判断激光雷达数据是否从缓存队列中拿到meas中的数据,  flg_EKF_inited 用于判断EKF是否初始化完成
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;

//设置是否发布激光雷达数据，是否发布稠密数据，是否发布激光雷达数据的身体数据
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<BoxPointType> cub_needrm;   // ikd-tree中，地图需要移除的包围盒序列
vector<PointVector>  Nearest_Points;     //每个点的最近点序列
vector<double>       extrinT(3, 0.0);    //雷达相对于IMU的外参T
vector<double>       extrinR(9, 0.0);    //雷达相对于IMU的外参R
deque<double>                     time_buffer;           // 激光雷达数据时间戳缓存队列
deque<PointCloudXYZI::Ptr>        lidar_buffer;  //记录特征提取或间隔采样后的lidar（特征）数据
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;// IMU数据缓存队列

//一些点云变量
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI()); //提取地图中的特征点，IKD-tree获得
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());   //去畸变的特征
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());          //畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());         //畸变纠正后降采样的单帧点云，W系

//下采样的体素点云
pcl::VoxelGrid<PointType> downSizeFilterSurf;//单帧内降采样使用voxel grid
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree; // ikd-tree类

V3D Lidar_T_wrt_IMU(Zero3d);  // T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);       // R lidar to imu (imu = r * lidar + t)

/*** EKF inputs and output ***/
// ESEKF操作
MeasureGroup Measures;

esekfom::esekf kf;

state_ikfom state_point;        // 状态
Eigen::Vector3d pos_lid;    // world系下lidar坐标

//输出的路径参数
nav_msgs::Path path;//包含了一系列位姿
nav_msgs::Odometry odomAftMapped; //只包含了一个位姿
geometry_msgs::PoseStamped msg_body_pose;//位姿

//激光和imu处理操作
shared_ptr<Preprocess> p_pre(new Preprocess());// 定义指向激光雷达数据的预处理类Preprocess的智能指针

//按下ctrl+c后唤醒所有线程
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();//  会唤醒所有等待队列中阻塞的线程 线程被唤醒后，会通过轮询方式获得锁，获得锁前也一直处理运行状态，不会被再次阻塞。
}

// 除了AVIA类型之外的雷达点云回调函数，将数据引入到buffer当中
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();//加锁
    scan_count ++;// wzl没用到
    double preprocess_start_time = omp_get_wtime();//记录时间 wzl没用到
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);// 点云预处理
    lidar_buffer.push_back(ptr);   //将点云放入缓冲区
    time_buffer.push_back(msg->header.stamp.toSec());    //将时间放入缓冲区
    last_timestamp_lidar = msg->header.stamp.toSec();   //记录最后一个时间
    mtx_buffer.unlock();
    sig_buffer.notify_all();// 唤醒所有线程
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;// 时间同步flag，false表示未进行时间同步，true表示已经进行过时间同步


// 订阅器 sub_pcl 的回调函数：接收Livox激光雷达的点云数据，对点云数据进行预处理（特征提取、降采样、滤波），
// 并将处理后的数据保存到激光雷达数据队列中
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();    // 互斥锁
    double preprocess_start_time = omp_get_wtime();//它返回一个双精度浮点值，表示从过去某个时间点开始到现在的经过的秒数。这个时间点是任意的，但是在程序执行期间保证不会改变。这个函数通常用于测量代码块的执行时间
    scan_count ++;    // 激光雷达扫描的总次数
    // 如果当前扫描的激光雷达数据的时间戳比上一次扫描的激光雷达数据的时间戳早，需要将激光雷达数据缓存队列清空
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    // 如果不需要进行时间同步，而imu时间戳和雷达时间戳相差大于10s，则输出错误信息
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }
    // time_sync_en为true时，当imu时间戳和雷达时间戳相差大于1s时，进行时间同步
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;// 标记已经进行时间同步
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    // 用pcl点云格式保存接收到的激光雷达数据
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    // 对激光雷达数据进行预处理（特征提取或者降采样），其中p_pre是Preprocess类的智能指针
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    mtx_buffer.unlock();
    sig_buffer.notify_all();// 唤醒所有线程
}
// 订阅器sub_imu的回调函数：接收IMU数据，将IMU数据保存到IMU数据缓存队列中
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    // 将IMU和激光雷达点云的时间戳对齐
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());//将IMU时间戳对齐到激光雷达时间戳
    }

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);// 系统默认都需要进行的时间校准

    double timestamp = msg->header.stamp.toSec();// IMU时间戳

    mtx_buffer.lock();
    // 如果当前IMU的时间戳小于上一个时刻IMU的时间戳，则IMU数据有误，将IMU数据缓存队列清空
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    // 将当前的IMU数据保存到IMU数据缓存队列中
    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();  //解锁
    sig_buffer.notify_all();// 唤醒阻塞的线程，当持有锁的线程释放锁时，这些线程中的一个会获得锁。而其余的会接着尝试获得锁
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
//把当前要处理的LIDAR和IMU数据打包到meas
//这部分主要处理了buffer中的数据，将两帧激光雷达点云数据时间内的IMU数据从缓存队列中取出，进行时间对齐，并保存到meas中
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) //如果缓存队列中没有数据，则返回false
    {
        return false;
    }

    /*** push a lidar scan ***/
    //如果还没有把雷达数据放到meas中的话，就执行一下操作
    if(!lidar_pushed)
    {
        // 从激光雷达点云缓存队列中取出点云数据，放到meas中
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();        // 该lidar测量起始的时间戳

        if (meas.lidar->points.size() <= 5) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; //此次雷达点云结束时刻
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; //此次雷达点云结束时刻
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); //此次雷达点云结束时刻
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;
        // 成功提取到lidar测量的标志
        lidar_pushed = true;
    }
    // 最新的IMU时间戳(也就是队尾的)不能早于雷达的end时间戳，因为last_timestamp_imu比较时是加了0.1的，
    // 所以要比较大于雷达的end时间戳
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    // 压入imu数据，并从imu缓冲区弹出
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    // 拿出lidar_beg_time到lidar_end_time之间的所有IMU数据
    //如果imu缓存队列中的数据时间戳小于雷达结束时间戳，则将该数据放到meas中,代表了这一帧中的imu数据
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();//获取imu数据的时间戳
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front()); //将imu数据放到meas中
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();//将lidar数据弹出
    time_buffer.pop_front();//将时间戳弹出
    lidar_pushed = false;  //将lidar_pushed置为false，代表lidar数据已经被放到meas中了
    return true;
}

//把点从body系转到world系，通过ikfom的位置和姿态
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    //下面式子最里面的括号是从雷达到IMU坐标系 然后从IMU转到世界坐标系
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix()*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
//把点从body系转到world系
template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix()*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

// 在拿到eskf前馈结果后。动态调整地图区域，防止地图过大而内存溢出，类似LOAM中提取局部地图的方法
BoxPointType LocalMap_Points;           // ikd-tree地图立方体的2个角点
bool Localmap_Initialized = false;      // 局部地图是否初始化
void lasermap_fov_segment()
{
    cub_needrm.clear();     // 清空需要移除的区域
    kdtree_delete_counter = 0;

    V3D pos_LiD = pos_lid;      // global系下lidar位置
    
    //初始化局部地图范围，以 pos_LiD 为中心,长宽高均为 cube_len 在 launch 文件里面会进行设置，默认是 200  ，但是设置为 1000 
    //初始化局部地图包围盒角点，以为w系下 lidar 位置为中心,得到长宽高 200*200*200 的局部地图
    if (!Localmap_Initialized)
    { // 系统起始需要初始化局部地图的大小和位置
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    //各个方向上 pos_LiD 与局部地图边界的距离，或者说是lidar与立方体盒子六个面的距离
    float dist_to_map_edge[3][2];
    bool need_move = false;
    // 当前雷达系中心到各个地图边缘的距离
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        // 与某个方向上的边界距离（1.5*300m）太小，标记需要移除 need_move(FAST-LIO2论文Fig.3)
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    //std::cout <<  need_move << std::endl;
    if (!need_move) return;  //如果不需要，直接返回，不更改局部地图
    // 否则需要计算移动的距离
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    // 新的局部地图盒子边界点
    New_LocalMap_Points = LocalMap_Points;
    //  max(（1000  - 2 * 1.5 * 450 ）* 0.5 * 0.9  ,   450  *（1.5 -1 ）) =  max（ 45  ，225  ） = 225
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));

    // std::cout <<  "MOV_THRESHOLD = " <<  MOV_THRESHOLD << std::endl;
    // std::cout <<  "DET_RANGE = " <<  DET_RANGE << std::endl;
    // std::cout <<  "cube_len = " <<  cube_len << std::endl;
    // std::cout <<  "mov_dist = " <<  mov_dist << std::endl;
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        //与包围盒最小值边界点距离
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints); // 移除较远包围盒
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    PointVector points_history;
    ikdtree.acquire_removed_points(points_history); //返回被剔除的点

    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm); //删除指定范围内的点
}
// 含有RGB的点云从Lidar系转到IMU系
void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I.matrix()*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

//根据最新估计位姿  增量添加点云到map
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        //转换到世界坐标系
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType mid_point;   //点所在体素的中心
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]); //如果距离最近的点都在体素外，则该点不需要Downsample
                continue;
            }
            for (int j = 0; j < NUM_MATCH_POINTS; j ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[j], mid_point) < dist)  //如果近邻点距离 < 当前点距离，不添加该点
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1)); //创建一个点云用于存储等待发布的点云
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());  //创建一个点云用于存储等待保存的点云
void publish_frame_world(const ros::Publisher & pubLaserCloudFull_)
{
    if(scan_pub_en) // 设置是否发布激光雷达数据
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);//判断是否需要降采样
        int size = laserCloudFullRes->points.size(); //获取待转换点云的大小
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));//创建一个点云用于存储转换到世界坐标系的点云

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);//从激光雷达坐标系转换到世界坐标系
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull_.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;// TODO czy 没有理解这是干什么的，前面注释说是接受到IMU的msg的总数，但是发布之后减去一个固定的数
    }
    //把结果压入到pcd中
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);//转换到世界坐标系
        }
        *pcl_wait_save += *laserCloudWorld;//把结果压入到pcd中

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}
//把去畸变的雷达系下的点云转到IMU系 
void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));//创建一个点云用于存储转换到IMU系的点云

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);//转换到IMU坐标系
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}
// 发布ikd-tree地图
void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}
// 设置输出的t,q，在publish_odometry，publish_path调用
template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);//将eskf求得的位置传入
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);

    auto q_ = Eigen::Quaterniond(state_point.rot.matrix());//将eskf求得的姿态传入
    out.pose.orientation.x = q_.coeffs()[0];
    out.pose.orientation.y = q_.coeffs()[1];
    out.pose.orientation.z = q_.coeffs()[2];
    out.pose.orientation.w = q_.coeffs()[3];   
}
//发布里程计
void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);

    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        //设置协方差 P里面先是旋转后是位置 这个POSE里面先是位置后是旋转 所以对应的协方差要对调一下
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );//发布tf变换
}
//每隔10个发布一下位姿
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

// FAST_LIO2主函数
int main(int argc, char** argv)
{
    /*****************************初始化：读取参数、定义变量以及赋值*************************/
    // 初始化ros节点，节点名为laserMapping
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    // 从参数服务器读取参数值赋给变量（包括launch文件和launch读取的yaml文件中的参数）
    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);                // 是否发布当前正在扫描的点云的topic
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);              // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic 
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);     // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic，需要该变量和上一个变量同时为true才发布
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);                        // 卡尔曼滤波的最大迭代次数
    nh.param<string>("map_file_path",map_file_path,"");                         // 地图保存路径
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");              // 雷达点云topic名称
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");               // IMU的topic名称
    nh.param<bool>("common/time_sync_en", time_sync_en, false);                 // 是否需要时间同步，只有当外部未进行时间同步时设为true
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);          // VoxelGrid降采样时的体素大小
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);    // VoxelGrid降采样时的体素大小
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);             // VoxelGrid降采样时的体素大小
    nh.param<double>("cube_side_length",cube_len,200);                          // 地图的局部区域的长度（FastLio2论文中有解释）
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);                       // 激光雷达的最大探测范围
    nh.param<double>("mapping/fov_degree",fov_deg,180);                     // 激光雷达的视场角
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);                            // IMU陀螺仪的协方差
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);                            // IMU加速度计的协方差
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);                     // IMU陀螺仪偏置的协方差
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);                     // IMU加速度计偏置的协方差
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);                   // 最小距离阈值，即过滤掉0～blind范围内的点云
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);            // 激光雷达的类型
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);                  // 激光雷达扫描的线数（livox avia为6线）
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);              // 采样间隔，即每隔point_filter_num个点取1个点
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);    // 是否提取特征点（FAST_LIO2默认不进行特征点提取）
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);                 // 是否将点云地图保存到PCD文件
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>()); // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>()); // 雷达相对于IMU的外参R
    
    cout<<"Lidar_type: "<<p_pre->lidar_type<<endl;
    // 初始化path的header（包括时间戳和帧id），path用于保存odemetry的路径
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** ROS subscribe initialization ***/
    // ROS订阅器和发布器的定义和初始化

    // 雷达点云的订阅器sub_pcl，订阅点云的topic
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);

    // IMU的订阅器sub_imu，订阅IMU的topic
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    // 发布当前正在扫描的点云，topic名字为/cloud_registered
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);

    // 发布经过运动畸变校正到IMU坐标系的点云，topic名字为/cloud_registered_body
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);

    // 后面的代码中没有用到
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);

    // 后面的代码中没有用到
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);

    // 发布当前里程计信息，topic名字为/Odometry
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/Odometry", 100000);

    // 发布里程计总的路径，topic名字为/path
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> ("/path", 100000);

    // VoxelGrid滤波器参数，即进行滤波时的创建的体素边长为filter_size_surf_min
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    // VoxelGrid滤波器参数，即进行滤波时的创建的体素边长为filter_size_map_min
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);


    shared_ptr<ImuProcess> p_imu1(new ImuProcess());
    // 从雷达到IMU的外参R和T
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);// 相对IMU的外参
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    // 设置IMU的参数，对p_imu进行初始化，其中p_imu为ImuProcess的智能指针（ImuProcess是进行IMU处理的类）
    p_imu1->set_param(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov), 
                        V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    //------------------------------------------------------------------------------------------------------
    // 中断处理函数，如果有中断信号（比如Ctrl+C），则执行第二个参数里面的SigHandle函数
    signal(SIGINT, SigHandle);

    // 设置ROS程序主循环每次运行的时间至少为0.0002秒（5000Hz）
    ros::Rate rate(5000);

    while (ros::ok())
    {
        // 如果有中断产生， 则结束主循环
        if (flg_exit) break;
        // ROS消息回调处理函数， 放在ROS的主循环中
        ros::spinOnce();
        // 将激光雷达点云数据和IMU数据从缓存队列中取出，进行时间对齐，并保存到Measures中
        if(sync_packages(Measures))  //把一次的IMU和LIDAR数据打包到Measures
        {
            double t00 = omp_get_wtime();
            // 激光雷达第一次扫描
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu1->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }
            // 对IMU数据进行预处理，其中包含了点云畸变处理 前向传播 反向传播
            p_imu1->Process(Measures, kf, feats_undistort);  

            //如果feats_undistort为空 ROS_WARN
            //如果点云数据为空，则代表了激光雷达没有完成去畸变，此时还不能初始化成功
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            // 获取kf预测的全局状态（imu）
            state_point = kf.get_x();
            //世界系下雷达坐标系的位置
            //下面式子的意义是W^p_L = W^p_I + W^R_I * I^t_L
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;
            //判断是否初始化完成，需要满足第一次扫描的时间和第一个点云时间的差值大于INIT_TIME
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            /*** Segment the map in lidar FOV ***/
            // 动态调整局部地图,在拿到eskf前馈结果后
            lasermap_fov_segment();     //更新localmap边界，然后降采样当前帧点云

            //点云下采样
            downSizeFilterSurf.setInputCloud(feats_undistort);//获得去畸变后的点云数据
            downSizeFilterSurf.filter(*feats_down_body);  //滤波降采样后的点云数据
            feats_down_size = feats_down_body->points.size(); //记录滤波后的点云数量
 
            // std::cout << "feats_down_size :" << feats_down_size << std::endl;
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            //初始化ikdtree(ikdtree为空时)   // 构建kd树
            if(ikdtree.Root_Node == nullptr)
            {
                  // 设置ikd tree的降采样参数    
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);//将下采样得到的地图点大小于body系大小一致
                for(int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));  //lidar坐标系转到世界坐标系
                }
                // 组织 ikd tree 
                ikdtree.Build(feats_down_world->points);        //根据世界坐标系下的点构建ikdtree 
                continue;
            }
            
            if(0) // If you need to see map point, change to "if(1)"
            {                // 释放 PCL_Storage 的内存
                PointVector ().swap(ikdtree.PCL_Storage);
                // 把树展平用于展示
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            /*** iterated state estimation ***/
            Nearest_Points.resize(feats_down_size);         //存储近邻点的vector   //将降采样处理后的点云用于搜索最近点

            //迭代卡尔曼滤波更新，更新地图信息
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, ikdtree, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_en);

            state_point = kf.get_x();
            //世界系下雷达坐标系的位置
            //下面式子的意义是W^p_L = W^p_I + W^R_I * I^t_L
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            /******* Publish odometry *******/
            /******* 发布里程计 *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            /*** 向映射kdtree添加特性点 ***/
            feats_down_world->resize(feats_down_size);
            map_incremental();
            
            /******* Publish points *******/
            /******* 发布轨迹和点 *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_map(pubLaserCloudMap);

            double t11 = omp_get_wtime();
            // std::cout << "feats_down_size: " <<  feats_down_size << "  Whole mapping time(ms):  " << (t11 - t00)*1000 << std::endl<< std::endl;
        }

        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 1; i <= pcd_index; i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(i) + string(".pcd"));
            pcl::PCDReader reader;
            reader.read(all_points_dir,*cloud_temp);
            *cloud = *cloud + *cloud_temp;
        }
    
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *cloud);
    }

    return 0;
}
