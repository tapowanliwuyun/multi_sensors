//运行不了，没有相关的输入文件
//IMU 尝试使用IMU预计分与3d相机量联合优化位姿，用于处理无特征的帧
//假设 相机与IMU之间的坐标系转换是已知的
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using  symbol_shorthand::X;
using  symbol_shorthand::V;
using  symbol_shorthand::B;

struct IMUhelper{
    IMUhelper(){
        {
            gtsam::Vector Vector6(6);
            Vector6 <<  5.0e-2,5.0e-2,5.0e-2,5.0e-3,5.0e-3,5.0e-3;
            auto gaussian=noiseModel::Diagonal::Sigmas(Vector6);
            auto huber=noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345),gaussian);
            biasNoiseModel=huber;
        }
        {
            auto gassian=noiseModel::Isotropic::Sigma(3,0.01);
            auto huber=noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345),gassian);
            velocityNoiseModel = huber;
        }
    //定义初始位置重力常量
    auto p = boost::make_shared<PreintegratedCombinedMeasurements::Params>(
        Vector3(0.0, 9.8, 0.0));

    //连续加速度白噪声
    p->accelerometerCovariance=I_3x3 *pow(0.0565,2.0);

    //积分连续不确定性
    p->integrationCovariance=I_3x3*1e-9;

    //陀螺仪连续白噪声
    p->gyroscopeCovariance=I_3x3*pow(4.0e-5,2.0);

    //加速度残差 连续白噪声
    p->biasAccCovariance=I_3x3*pow(0.00002,2.0);

    //陀螺仪残差 连续白噪声
    p->biasAccOmegaInt=Matrix::Identity(6,6)*1e-5;

    //Imu imu自身坐标与导航坐标系的旋转
    Rot3 iRb(0.036129, -0.998727, 0.035207,
             0.045417, -0.033553, -0.998404,
             0.998315, 0.037670, 0.044147);
    //Imu imu自身坐标与导航坐标系的位置误差
    Point3 iTb(0.03,-0.025,-0.06);

    //左边相机与imu的位姿变换
    p->body_P_sensor = Pose3(iRb, iTb);

    //初始位姿
    Rot3 prior_rotation = Rot3(I_3x3);
    Pose3 prior_pose(prior_rotation, Point3(0, 0, 0));

    //相机坐标系下 imu矫正的残差
    Vector3 acc_bias(0.0, -0.0942015, 0.0);
    Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);
    priorImuBias = imuBias::ConstantBias(acc_bias, gyro_bias);

    //初始化导航向量
    prevState=NavState(prior_pose,Vector3::Zero());
    propState=prevState;
    preintegrated=new PreintegratedCombinedMeasurements(p,priorImuBias);
    }
    imuBias::ConstantBias priorImuBias;                 //初始误差
    noiseModel::Robust::shared_ptr velocityNoiseModel;  //速度噪声
    noiseModel::Robust::shared_ptr biasNoiseModel;      //残差噪声模型
    NavState prevState;                                 //之前的导航状态
    NavState propState;                                 //当前导航状态
    imuBias::ConstantBias prevBias;                     //之前的残差
    PreintegratedCombinedMeasurements* preintegrated;   //之前的预积分
};

int main(int argc, const char** argv) {
    string file="/home/n1/notes/gtsam/ISAM_IMU_CAMERA/ISAM2_SmartFactorStereo_IMU.txt";
    ifstream in(file);
    //相机内参
    double fx = 822.37;
    double fy = 822.37;
    double cx = 538.73;
    double cy = 579.10;
    double baseline = 0.372;  

    //初始化相机
    Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline));
    ISAM2Params parameters;
    parameters.relinearizeThreshold=0.1;
    ISAM2 isam(parameters);

    //创建因子图
    std::map<size_t,SmartStereoProjectionPoseFactor::shared_ptr>smartFactors;//构造容器
    NonlinearFactorGraph graph;
    Values InitialEstimate;
    IMUhelper imu;

    //初始值处理
    //Pose初始值
            gtsam::Vector Vector6(6);
            Vector6 <<  0.1,0.1,0.1,0.1,0.1,0.1;
    auto priorPoseNoise=noiseModel::Diagonal::Sigmas(
            Vector6);
       
    graph.emplace_shared<PriorFactor<Pose3> >(X(1), Pose3::identity(),priorPoseNoise);
    //从0开始
    InitialEstimate.insert(X(0),Pose3::identity());

    //imu初始值和噪声模型
    graph.add(PriorFactor<imuBias::ConstantBias>(B(1),imu.priorImuBias,imu.biasNoiseModel));
    InitialEstimate.insert(B(0),imu.priorImuBias);

    //加速度 假设为恒定的
    graph.add(PriorFactor<Vector3>(V(1),Vector3(0,0,0),imu.velocityNoiseModel));
    InitialEstimate.insert(V(0),Vector3(0, 0, 0));

    int lastframe=1;
    int frame;
    while(1){
        char line[1024];
        in.getline(line,sizeof(line));
        stringstream ss(line);
        char type;
        ss>> type >> frame;
        if(frame !=lastframe||in.eof()){
            cout<<"当前帧:"<<lastframe<<endl;
            InitialEstimate.insert(X(lastframe),Pose3::identity());
            InitialEstimate.insert(V(lastframe),Vector3(0,0,0));
            InitialEstimate.insert(B(lastframe),imu.priorImuBias);

            CombinedImuFactor imuFactor(X(lastframe-1),V(lastframe - 1),
                                        X(lastframe),V(lastframe),B(lastframe-1),
                                        B(lastframe),*imu.preintegrated);
            graph.add(imuFactor);//添加新的的变量
            isam.update(graph,InitialEstimate);//更新
            Values currentEstimate=isam.calculateEstimate();//当前值
            //获得当前估计值 predict（当前状态，当前残差）
            imu.propState=imu.preintegrated->predict(imu.prevState,imu.prevBias);//
            //更新状态
            imu.prevState=NavState(currentEstimate.at<Pose3>(X(lastframe)),currentEstimate.at<Vector3>(V(lastframe)));
            //更新残差
            imu.prevBias=currentEstimate.at<imuBias::ConstantBias>(B(lastframe));
            //更新当前预计分值
            imu.preintegrated->resetIntegrationAndSetBias(imu.prevBias);
            //重塑大小，删除所有已存在的。
            //当新的size小于原始size，先删除新的值，大于添加null
            graph.resize(0);
            InitialEstimate.print();
            InitialEstimate.clear();
            if(in.eof()){
                break;
            }
        }
        if(type=='i'){//Imu数据
            double ax,ay,az;
            double gx,gy,gz;
            double dt=1/800.0;//Imu 800Hz
            ss>>ax>>ay>>az;
            ss>>gx>>gy>>gz;
            Vector3 acc(ax,ay,az);
            Vector3 gyro(gx,gy,gz);
            //测量值
            imu.preintegrated->integrateMeasurement(acc,gyro,dt);
        }
        else if(type=='s'){//双目相机
            int landmark;
            double xl,xr,y;
            ss>>landmark>>xl>>xr>>y;
            if (smartFactors.count(landmark) == 0) {
                auto gaussian = noiseModel::Isotropic::Sigma(3, 1.0);
                
                //SmartProjectionParams 双目相机的smartFactor
                //  该类实现了相机类贝叶斯树的线性化和degeneracy
                // 默认参数：
                //      LinearizationMode linMode = HESSIAN, HESSIAN：2阶海森矩阵线性化
                //      DegeneracyMode degMode = IGNORE_DEGENERACY,：degeneracy（退化）模式
                //                          IGNORE_DEGENERACY, ZERO_ON_DEGENERACY, HANDLE_INFINITY
                //      bool throwCheirality = false, 如果为真重新抛出Cheirality（与景深相关）异常
                //      bool verboseCheirality = false, 输出Cheirality异常
                //      double retriangulationTh = 1e-5 重新三角化范围
                SmartProjectionParams params(HESSIAN, ZERO_ON_DEGENERACY);

                //SmartStereoProjectionPoseFactor
                //          假设每一个相机内参都有自己独立参数且已经被矫正
                //参数：
                //          const SharedNoiseModel& sharedNoiseModel：噪声模型
                //          const SmartStereoProjectionParams& params = SmartStereoProjectionParams(),参数
                //          const boost::optional<Pose3> body_P_sensor = boost::none 初始位姿
                smartFactors[landmark] = SmartStereoProjectionPoseFactor::shared_ptr(
                    new SmartStereoProjectionPoseFactor(gaussian, params));
                graph.push_back(smartFactors[landmark]);
            }
            //xl 左边相机的像素x
            //xr 右边相机对应像素的x
            //y  矫正后的y
            smartFactors[landmark]->add(StereoPoint2(xl,xr,y),X(frame),K);
        }
        else {
                
         throw runtime_error("读取错误: " + string(1, type));
        }
        lastframe = frame;
    

    }
    return 0;
};
