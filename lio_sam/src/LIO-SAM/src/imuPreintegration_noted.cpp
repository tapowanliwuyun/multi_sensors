#include "utility.h"
 
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
 
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
 
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
 
 
// 订阅激光里程计（来自MapOptimization）和IMU里程计，
//根据前一时刻激光里程计，和该时刻到当前时刻的IMU里程计变换增量，
//计算当前时刻IMU里程计；
//rviz展示IMU里程计轨迹（局部）。
class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;
 
    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;
 
    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;
 
    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;
 
    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;
 
    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;
 
    TransformFusion()
    {
        // 如果lidar系与baselink系不同（激光系和载体系），需要外部提供二者之间的变换关系
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                // 等待3s
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                // lidar系到baselink系的变换
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }
        // 订阅激光里程计，来自mapOptimization
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 订阅imu里程计，来自IMUPreintegration(IMUPreintegration.cpp中的类IMUPreintegration)
        //topic name: odometry/imu_incremental
        //注意区分lio_sam/mapping/odometry_incremental
        //目前可以明确的一点，odometry/imu_incremental是增量内容，即两帧激光里程计之间的预积分内容,（加上开始的激光里程计本身有的位姿）
        //imuIntegratorImu_本身是个积分器，只有两帧之间的预积分，但是发布的时候发布的实际是结合了前述里程计本身有的位姿
        //如这个predict里的prevStateOdom:
        //currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());
        // 发布imu里程计，用于rviz展示
        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        // 发布imu里程计轨迹
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1);
    }
 
 
    /**
     * 里程计对应变换矩阵
    */
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }
 
    /**
     * 订阅激光里程计的回调函数，来自mapOptimization
    */
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        // 激光里程计对应变换矩阵
        lidarOdomAffine = odom2affine(*odomMsg);
        // 激光里程计时间戳
        lidarOdomTime = odomMsg->header.stamp.toSec();
        //这二者里面保存的都是最近的一个雷达激光里程计的变换和时间戳（不再是用一个vector之类的东西保存起来）
    }
 
    /**
     * 订阅imu里程计，来自IMUPreintegration
     * 1、以最近一帧激光里程计位姿为基础，计算该时刻与当前时刻间imu里程计增量位姿变换，相乘得到当前时刻imu里程计位姿
     * 2、发布当前时刻里程计位姿，用于rviz展示；发布imu里程计路径，注：只是最近一帧激光里程计时刻与当前时刻之间的一段
    */
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // 发布tf，map与odom系设为同一个系
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));
 
        std::lock_guard<std::mutex> lock(mtx);
        // 添加imu里程计到队列，注：imu里程计由本cpp中的另一个类imuPreintegration来发布
        imuOdomQueue.push_back(*odomMsg);
 
        // get latest odometry (at current IMU stamp)
        // 从imu里程计队列中删除当前（最近的一帧）激光里程计时刻之前的数据
        // lidarOdomTime初始化为-1，在收到lidar里程计数据后，在回调函数lidarOdometryHandler中被赋值时间戳
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        // 最近的一帧激光里程计时刻对应imu里程计位姿
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        // 当前时刻imu里程计位姿
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        // imu里程计增量位姿变换
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        //  当前时刻imu里程计位姿=最近的一帧激光里程计位姿 * imu里程计增量位姿变换 
        //lidarOdomAffine在本类的lidarOdometryHandler回调函数中被赋值，消息来源于mapOptimization.cpp发布,是激光里程计
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // publish latest odometry
        //发布名为"odometry/imu"的话题
        //也就是说，这个函数先监听了类IMUPreintegration中发布的odometry/imu_incremental，
        //然后计算imu二者之间的增量，然后在激光的基础上加上增量，重新发布
        //可以看出发布的并非odometry/imu_incremental的一部分子数据，而是把x，y，z，roll，pitch，yaw都经过了激光里程计的修正，才发布
        //可以看出这次发布的内容，是当前时刻里程计位姿.发布名称为"odometry/imu"
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back(); 
        laserOdometry.pose.pose.position.x = x;  //赋上新的值
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);
 
        // 发布tf，当前时刻odom与baselink系变换关系
        //由于之前把map和odom坐标系固定了，因此这里我认为发布的就是真正的最终位姿关系
        //map优化提供激光，预积分提供imu，imu之间变换再乘以激光里程计得到各个时刻精确位姿
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);
 
        // publish IMU path
        // 发布imu里程计路径，注：只是最近一帧激光里程计时刻与当前时刻之间的一段
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        // 每隔0.1s添加一个
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame; //就叫"odom"
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            // 删除最近一帧激光里程计时刻之前的imu里程计
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};
 
class IMUPreintegration : public ParamServer
{
public:
 
    std::mutex mtx;
 
    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;
 
    bool systemInitialized = false;
    // 噪声协方差
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;
 
    // imu预积分器
 
    // imuIntegratorOpt_ 负责预积分两个激光里程计之间的imu数据，作为约束加入因子图，并且优化出bias
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    // imuIntegratorImu_ 用来根据新的激光里程计到达后已经优化好的 bias ，预测从当前帧开始，下一帧激光里程计到达之前的imu里程计增量
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
 
    // imu数据队列
    // imuQueOpt 用来给 imuIntegratorOpt_ 提供数据来源，不要的就弹出(从队头开始出发，比当前激光里程计数据早的imu通通积分，用一个扔一个)；
    std::deque<sensor_msgs::Imu> imuQueOpt;
    // imuQueImu 用来给 imuIntegratorImu_ 提供数据来源，不要的就弹出(弹出当前激光里程计之前的 imu 数据,预积分用完一个弹一个)；
    std::deque<sensor_msgs::Imu> imuQueImu;
 
    // imu因子图优化过程中的状态变量
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
 
    // imu状态
    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;
 
    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
 
    // ISAM2优化器
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;  //总的因子图模型
    gtsam::Values graphValues;  //因子图模型中的值
 
    const double delta_t = 0;
 
    int key = 1;
 
    // imu-lidar位姿变换
    //这点要注意，tixiaoshan这里命名的很垃圾，这只是一个平移变换，
    //同样头文件的 imuConverter 中，也只有一个旋转变换。这里绝对不可以理解为把 imu 数据转到 lidar 下的变换矩阵。
    //事实上，作者后续是把imu数据先用 imuConverter 旋转到雷达系下（但其实还差了个平移）。
    //作者真正是把雷达数据又根据 lidar2Imu 反向平移了一下，和转换以后差了个平移的 imu 数据在“中间系”对齐，
    //之后算完又从中间系通过 imu2Lidar 挪回了雷达系进行 publish 。
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));
 
    IMUPreintegration()
    {
        //订阅imu原始数据，用下面因子图优化的结果，施加两帧之间的imu预计分量，预测每一时刻（imu频率）的imu里程计
        //imuTopic name: "imu_correct"
        subImu = nh.subscribe<sensor_msgs::Imu>  (imuTopic, 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay());
        // 订阅激光里程计，来自 mapOptimization ，用两帧之间的imu预计分量构建因子图，
        // 优化当前帧位姿（这个位姿仅用于更新每时刻的imu里程计，以及下一次因子图优化）
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5 ,&IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        
        //发布imu里程计: odometry/imu_incremental
        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
 
        // imu预积分的噪声协方差
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
 
        //imuAccNoise 和 imuGyrNoise 都是定义在头文件中的高斯白噪声，由配置文件中写入
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        //对于速度的积分误差？这块暂时不太理解
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        //假设没有初始的bias
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
        
        // 噪声先验
        // Diagonal 对角线矩阵
        // 发现 diagonal 型一般调用 .finished() ,注释中说 finished() 意为设置完所有系数后返回构建的矩阵
        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        // 激光里程计 scan-to-map 优化过程中发生退化，则选择一个较大的协方差
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        //imu预积分器，用于预测每一时刻（imu频率）的imu里程计（转到lidar系了，与激光里程计同一个系）
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        //imu预积分器，用于因子图优化
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }
 
    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);
 
        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;
 
        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }
 
    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }
    // 订阅的是激光里程计,"lio_sam/mapping/odometry_incremental"
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        // 当前帧激光里程计时间戳
        double currentCorrectionTime = ROS_TIME(odomMsg);
 
        // 确保 imu 优化队列中有 imu 数据进行预积分
        if (imuQueOpt.empty())
            return;
 
        // 当前帧激光位姿，来自scan-to-map匹配、因子图优化后的位姿
        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));
 
 
        // 0. initialize system
        // 0. 系统初始化，第一帧
        if (systemInitialized == false)
        {
            // 重置 ISAM2 优化器
            resetOptimization();
 
            // pop old IMU message
            // 从imu优化队列中删除当前帧激光里程计时刻之前的imu数据,delta_t=0
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // initial pose
            // 添加里程计位姿先验因子
            //lidarPose 为本回调函数收到的激光里程计数据，重组成 gtsam 的 pose 格式
            //并转到 imu 坐标系下,我猜测 compose 可能类似于左乘之类的含义吧
            prevPose_ = lidarPose.compose(lidar2Imu);
            //X可能是固定搭配（当使用Pose时），如果是速度则是V，bias则是B
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            //通过调用总的因子图模型的add方式，添加第一个因子
            //PriorFactor 概念可看gtsam  包括了位姿 速度  bias 
            //加入PriorFactor在图优化中基本都是必需的前提
            //各种noise都定义在构造函数当中
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            // 变量节点赋初值
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            // 优化一次
            optimizer.update(graphFactors, graphValues);
            //图和节点均清零  为什么要清零不能继续用吗?
            //是因为节点信息保存在 gtsam::ISAM2 optimizer，所以要清理后才能继续使用
            graphFactors.resize(0);
            graphValues.clear();
 
            //积分器重置,重置优化之后的偏置
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            return;
        }
 
 
        // reset graph for speed
        // 每隔 100 帧激光里程计，重置ISAM2优化器，保证优化效率
        if (key == 100)
        {
            // get updated noise before reset
            // 前一帧的位姿、速度、偏置噪声模型
            //保存最后的噪声值
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            // 重置ISAM2优化器
            resetOptimization();
            // add pose
            // 添加位姿先验因子，用前一帧的值初始化
            // 重置之后还有类似与初始化的过程 区别在于噪声值不同
            // prevPose_ 等三项，也是上一时刻得到的，
            // （初始时刻是 lidar 里程计的 pose 直接用 lidar2IMU 变量转到 imu 坐标系下，而此处则是通过上一时刻，即接下来的后续优化中得到）
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();
 
            key = 1;
        }
 
 
        // 1. integrate imu data and optimize
        // 1. 计算前一帧与当前帧之间的imu预积分量，用前一帧状态施加预积分量得到当前帧初始状态估计，
        //  添加来自mapOptimization的当前帧位姿，进行因子图优化，更新当前帧状态
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            // 提取前一帧与当前帧之间的imu数据，计算预积分
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            //currentCorrectionTime 是当前回调函数收到的激光里程计数据的时间
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                // imu预积分数据输入： 加速度、角速度、dt
                // 加入的是这个用来因子图优化的预积分器 imuIntegratorOpt_ ,注意加入了上一步算出的 dt
                //作者要求的9轴imu数据中欧拉角在本程序文件中没有任何用到,全在地图优化里用到的
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                //在推出一次数据前保存上一个数据的时间戳
 
                lastImuT_opt = imuTime;
                // 从队列中删除已经处理的imu数据
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // add imu factor to graph
        //利用两帧之间的IMU数据完成了预积分后增加imu因子到因子图中,
        //注意后面容易被遮挡， imuIntegratorOpt_ 的值经过格式转换被传入 preint_imu，
        //因此可以推测 imuIntegratorOpt_ 中的integrateMeasurement 函数应该就是一个简单的积分轮子，
        //传入数据和 dt ，得到一个积分量,数据会被存放在 imuIntegratorOpt_ 中
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        // 参数：前一帧位姿，前一帧速度，当前帧位姿，当前帧速度，前一帧偏置，预计分量
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        // 添加 imu 偏置因子，前一帧偏置 B(key - 1) ，当前帧偏置 B(key) ，观测值，噪声协方差；deltaTij() 是积分段的时间
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        // 添加位姿因子
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        // 用前一帧的状态、偏置，施加imu预计分量，得到当前帧的状态
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        // 变量节点赋初值
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
         // 优化结果
        gtsam::Values result = optimizer.calculateEstimate();
        // 更新当前帧位姿、速度
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        // 更新当前帧状态
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        // 更新当前帧imu偏置
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        //重置预积分器，设置新的偏置，这样下一帧激光里程计进来的时候，预积分量就是两帧之间的增量
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        // imu因子图优化结果，速度或者偏置过大，认为失败
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }
 
 
        // 2. after optiization, re-propagate imu odometry preintegration
        // 2. 优化之后，执行重传播；优化更新了imu的偏置，
        //用最新的偏置重新计算当前激光里程计时刻之后的imu预积分，这个预积分用于计算每时刻位姿
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        // 从imu队列中删除当前激光里程计时刻之前的imu数据
        double lastImuQT = -1;
        //注意，这里是要“删除”当前帧“之前”的imu数据，是想根据当前帧“之后”的累积递推。
        //而前面imuIntegratorOpt_做的事情是，“提取”当前帧“之前”的imu数据，用两帧之间的imu数据进行积分。处理过的就弹出来。
        //因此，新到一帧激光帧里程计数据时，imuQueOpt队列变化如下：
        //当前帧之前的数据被提出来做积分，用一个删一个（这样下一帧到达后，队列中就不会有现在这帧之前的数据了）
        //那么在更新完以后，imuQueOpt队列不再变化，剩下的原始imu数据用作下一次优化时的数据。
        //而imuQueImu队列则是把当前帧之前的imu数据都给直接剔除掉，仅保留当前帧之后的imu数据，
        //用作两帧lidar里程计到达时刻之间发布的imu增量式里程计的预测。
        //imuQueImu和imuQueOpt的区别要明确,imuIntegratorImu_和imuIntegratorOpt_的区别也要明确,见imuhandler中的注释
 
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        // 对剩余的imu数据计算预积分
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            // 传入状态,重置预积分器和最新的偏置
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            // 计算预积分
            //利用imuQueImu中的数据进行预积分 主要区别旧在于上一行的更新了bias
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);
                // 注意:加入的是这个用于传播的的预积分器imuIntegratorImu_,(之前用来计算的是imuIntegratorOpt_,）
                //注意加入了上一步算出的dt
                //结果被存放在imuIntegratorImu_中
                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }
 
        ++key;
        //设置成True，用来通知另一个负责发布imu里程计的回调函数imuHandler“可以发布了”
        doneFirstOpt = true;
    }
 
    /**
     * imu因子图优化结果，速度或者偏置过大，认为失败
    */
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }
 
        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }
 
        return false;
    }
 
    /**
     * 订阅imu原始数据
     * 1、用上一帧激光里程计时刻对应的状态、偏置，
     * 施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态，也就是imu里程计
     * 2、imu里程计位姿转到lidar系，发布里程计
    */
 
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);
        // imu原始测量数据转换到lidar系，加速度、角速度、RPY
        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);
 
        // 添加当前帧imu数据到队列
        // 两个双端队列分别装着优化前后的imu数据
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);
 
        // 要求上一次imu因子图优化执行成功，确保更新了上一帧（激光里程计帧）的状态、偏置，预积分已经被重新计算
        // 这里需要先在 odomhandler 中优化一次后再进行该函数后续的工作
        if (doneFirstOpt == false)
            return;
 
        double imuTime = ROS_TIME(&thisImu);
        //lastImuT_imu 变量初始被赋值为 -1
        // 获得时间间隔, 第一次为 1/500 , 之后是两次 imuTime 间的差
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;
 
        // integrate this single imu message
        // imu 预积分器添加一帧 imu 数据，注：这个预积分器的起始时刻是上一帧激光里程计时刻
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);
 
        // predict odometry
        // 用上一帧激光里程计时刻对应的状态、偏置，施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
 
        // publish odometry
        // 发布imu里程计（转到lidar系，与激光里程计同一个系）
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame; //"odom"
        odometry.child_frame_id = "odom_imu";
 
        // transform imu pose to ldiar
        //预测值 currentState 获得 imu 位姿, 再由 imu 到雷达变换, 获得雷达位姿
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);
        // 这里留疑问，本cpp读完后补充：
        // 为什么 currentState 获得的是imu的位姿？原始imu数据难道不是先转换成雷达坐标系下的数据（this imu）才再送到imu预积分器中吗？
        //答：在之前的优化函数 odometryHandler 中， thisIMU 是直接从 imuQueOpt 中取值.
        //而 imuQueOpt 中的内容，是已经从 imu 原始测量数据转换到了 lidar "中间系"系下（在本函数第二行）。离真正的雷达系还差了一个平移
        // odometryHandler 函数中根据 prevPose_ = lidarPose.compose(lidar2Imu) 得到激光帧先验位姿 (lidarPose) 转换到 imu 系下,（相当于从真正的雷达系扭到上面的中间系中）
        //作为初值构建了因子进行优化；
        //在其 imuIntegratorOpt_->integrateMeasurement 中得到的应该是dt之间的预积分量，
        //由于其处在循环中，因此是在递推累积计算两帧之间的预积分量。
        //（相当于是每到一帧，就把二者之间的预积分量计算一遍，并且优化一遍，存储进 imuIntegratorOpt_ ）中。
 
        //因为本函数为imu回调函数，每收到一个imu数据，当之前因子图算完的情况下，
        //在 imuIntegratorImu_ 的基础上继续递推新收到的imu数据，并且进行预测。
        //最后把imu再转回lidar系下进行发布。
        //注意：这里发布的是两帧之间的“增量”imu里程计信息，
        // imuIntegratorImu_ 本身是个积分器，只有两帧之间的预积分，但是发布的时候发布的实际是结合了前述里程计本身有的位姿
        //如这个 predict 里的 prevStateOdom :
        //currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
 
        //关于 imuIntegratorImu_ 在两个回调函数中都进行 integrateMeasurement 操作，之间是否会有冲突呢？
        //我觉得关键在于 odometryHandler 中有一句： imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom)，
        //在 imuIntegratorOpt_ 优化完两帧 imu 数据之后， imuIntegratorImu_ 直接把积分和 bias 给 reset 掉，
        //然后开始根据 imuIntegratorOpt_ 优化出的 bias 来更新 imuIntegratorImu_ 。
 
        // imuIntegratorImu_ 和 imuIntegratorOpt_ 的区别在于， opt 存储的是新到一帧，和上一帧之间的预积分量，作为约束，执行优化。
        // 优化后， imuIntegratorImu_ 利用新的 bias ，在新到的这一帧的基础上，递推“之后”的预积分量。
        //（绝对不能把 imuIntegratorOpt_ 和 imuIntegratorImu_ 理解为同一批 imu 数据在优化前后的不同值）
 
 
 
        //在更新的过程中不用担心会被 imuHandler 中的 imuIntegratorImu_->integrateMeasurement 给顶掉，
        //这是因为 imuHandler 要根据 doneFirstOpt 来检查 odometryHandler 是不是已经更新完 bias 了。
        //因为更新并不是实时的，而是一帧激光数据到了才更新。
        //可是下一帧激光并没有到，但是在此期间imu增量式里程计还是得照常发布，
        //如果当前帧已经更新完 bias 了，然后就可以直接利用这个 bias 计算后面新到的 imuIntegratorImu_ ，
 
        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }
};
//还有两个问题：
//1.第一个是，为什么 imu 原始数据先要根据 imuConverter 变到 lidar 系，
//那么之后 imuintegrator->integrateMeasurement 算到的预积分数据不就是lidar系下的吗？
//在处理的时候又是把 lidar 里程计的坐标系根据 compose 函数变到 imu 系？这难道不是不对应了吗:
 
//2.变量 gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
//gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
//这里为什么不用配置文件中的 extRot，extQRPY 之类的内容呢，而只是用了 extTrans 数据？
 
//关于这点，我在 github 中找到了解释: https://github.com/TixiaoShan/LIO-SAM/issues/30 
//imuConverter() to align the axis of the two coordinates , 并没有涉及到平移,
//lidar2Imu or imu2Lidar 却只有平移的内容
//因此收到 imu 后，先用 imuConverter() 转换到雷达系下，（但其实和雷达之间仍然差了一个平移），
//因此又把雷达的内容用只含有平移的 lidar2Imu 和原本差了一个平移的 imu 数据真正对齐
//（相当于是imu旋转到雷达系下以后不平移，然后把雷达倒着平移过来,在一个“中间系”对齐）。
//在算完以后，等发布的时候，又用 imu2Lidar 又倒回到了正儿八经的雷达系。
 
//那么 tixiaoshan 为什么在默认里把平移参数设置为0，0，0？
//他在github中的解释为: 我在不同的数据集中改变了几次IMU的安装位置。但是位置总是靠近激光雷达。
//所以每次测试不同的数据集时，我都不必费心去修改这个参数。严格地说，我的方法并不理想。需要提供此参数以获得更好的性能。
int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");
    
    IMUPreintegration ImuP;
 
    TransformFusion TF;
 
    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}