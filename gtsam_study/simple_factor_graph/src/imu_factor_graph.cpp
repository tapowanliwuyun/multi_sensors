#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;//用作表示    姿态(x,y,z,r,p,y)
using symbol_shorthand::V;//用表示      速度导数(xdot,ydot,zdot)
using symbol_shorthand::B;//陀螺仪残差  (ax,ay,az,gx,gy,gz)
string inputfile="../data/imu/imuAndGPSdata.csv";
string outputfile="../data/imu/result1.csv";
PreintegrationType *imu_preintegrated_;
//    Matrix3 biasAccCovariance;     3*3矩阵 加速度计的协防差矩阵，（可以根据残差计算加速度雅克比矩阵逐步更新）
//    Matrix3 biasOmegaCovariance;   3*3矩阵 陀螺仪的协防差矩阵， （可以根据残差计算雅克比矩阵递归更新预计分值，防止从头计算）
//    Matrix6 biasAccOmegaInt;       6*6矩阵 位置残关于加速度和速度残差的协防差，用作更新预计分
int main(int argc, const char** argv) {
    FILE* fp=fopen(outputfile.c_str(),"w+");
    //输出
    fprintf(fp,"#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,gt_qy,gt_qz,gt_qw\n");

    //解析 CSV
    ifstream file(inputfile.c_str());
    string value;

    Eigen::Matrix<double,10,1> initial_state=Eigen::Matrix<double,10,1>::Zero();
    //N,E,D,qx,qy,qz,qw,VelN,VelE,VelD
    getline(file,value,',');
    for(int i=0;i<9;i++){
        getline(file,value,',');
        initial_state(i)=atof(value.c_str());//转为浮点型
    }
    getline(file,value,'\n');//换行
    initial_state(9) = atof(value.c_str());
    Rot3 prior_rotation=Rot3::Quaternion(initial_state(6),initial_state(3),initial_state(4),initial_state(5));
    Point3 prior_point(initial_state(0),initial_state(1),initial_state(2));
    Pose3 prior_pose(prior_rotation,prior_point);                               //初始位姿
    Vector3 prior_velocity(initial_state(7),initial_state(8),initial_state(9)); //初始速度
    imuBias::ConstantBias prior_imu_bias;//残差，默认设为0

    //初始化值
    Values initial_values;
    int correction_count=0;
    //位姿
    initial_values.insert(X(correction_count),prior_pose);
    //速度
    initial_values.insert(V(correction_count),prior_velocity);
    //残差
    initial_values.insert(B(correction_count),prior_imu_bias);
    cout << "initial state:\n" << initial_state.transpose() <<endl;
    //设置噪声模型
    //一般为设置为对角噪声
        gtsam::Vector Vector6(6);
    Vector6 << 0.01,0.01,0.01,0.5,0.5,0.5;
    noiseModel::Diagonal::shared_ptr pose_noise_model=noiseModel::Diagonal::Sigmas(Vector6);
    noiseModel::Diagonal::shared_ptr velocity_noise_model=noiseModel::Isotropic::Sigma(3,0.1);
    noiseModel::Diagonal::shared_ptr bias_noise_model=noiseModel::Isotropic::Sigma(6,0.001);
    NonlinearFactorGraph *graph = new NonlinearFactorGraph();
    graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
    graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
    graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));
    //使用传感器信息构建IMU的噪声模型
    double accel_noise_sigma = 0.0003924;
    double gyro_noise_sigma = 0.000205689024915;
    double accel_bias_rw_sigma = 0.004905;
    double gyro_bias_rw_sigma = 0.000001454441043;

    Matrix33 measured_acc_cov=Matrix33::Identity(3,3)*pow(accel_noise_sigma,2);
    Matrix33 measured_omega_cov=Matrix33::Identity(3,3)*pow(gyro_bias_rw_sigma,2);
    Matrix33 integration_error_cov=Matrix33::Identity(3,3)*1e-8;        //速度积分误差
    Matrix33 bias_acc_cov=Matrix33::Identity(3,3)*pow(accel_bias_rw_sigma,2);
    Matrix33 bias_omega_cov=Matrix33::Identity(3,3)*pow(gyro_bias_rw_sigma,2);
    Matrix66 bias_acc_omega_int=Matrix66::Identity(6,6)*1e-5;           //积分骗到误差

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p=PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    //MakeSharedD:NED坐标系，g默认为 9.81，这里设置为0
    //MakeSharedU：NEU坐标系，g默认为 9.81
    
    // 设置预积分分参数
    p->accelerometerCovariance=measured_acc_cov;
    p->integrationCovariance=integration_error_cov;
    p->gyroscopeCovariance=measured_omega_cov;

    //预计分测量值
    p->biasAccCovariance=bias_acc_cov;
    p->biasAccOmegaInt=bias_acc_omega_int;
    p->biasOmegaCovariance=bias_omega_cov;
#ifdef USE_COMBINED
  imu_preintegrated_ = new PreintegratedCombinedMeasurements(p, prior_imu_bias);
#else
  imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);
#endif
    //保存上一次的imu积分值和结果
    NavState prev_state(prior_pose,prior_velocity);
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias=prior_imu_bias;//

    //记录总体误差
    double current_position_error = 0.0, current_orientation_error = 0.0;

    double output_time=0;
    double dt=0.005;    //积分时间

    //使用数据进行迭代
    while(file.good()){
        getline(file,value,',');
        int type=atoi(value.c_str());//字符转为整形
        if (type == 0) { // IMU 测量数据
            Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
            //读取imu数据
            for (int i=0; i<5; ++i) {
                getline(file, value, ',');
                imu(i) = atof(value.c_str());
            }
            getline(file, value, '\n');
            imu(5) = atof(value.c_str());
            // 检测测量值加入预计分
            imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

        }else if(type ==1){//Gps测量数据
            Eigen::Matrix<double,7,1> gps=Eigen::Matrix<double,7,1>::Zero();
            for(int i=0;i<6;i++){
                getline(file,value,',');
                gps(i)=atof(value.c_str());
            }
            getline(file, value, '\n');
            gps(6)=atof(value.c_str());
            correction_count++;
        
#ifdef USE_COMBINED
    //预计分测量值
        PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_);
    //IMU 因子
    //typedef NoiseModelFactor6<Pose3, Vector3, Pose3, Vector3,imuBias::ConstantBias, imuBias::ConstantBias>
        CombinedImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                                   X(correction_count  ), V(correction_count  ),
                                   B(correction_count-1), B(correction_count  ),
                                   *preint_imu_combined);
        graph->add(imu_factor);
#else
    //预计分测量值
        PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
        ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                           X(correction_count  ), V(correction_count  ),
                           B(correction_count-1),
                           *preint_imu);
        graph->add(imu_factor);
        imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        graph->add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1),
                                                      B(correction_count  ),
                                                      zero_bias, bias_noise_model));
#endif
        noiseModel::Diagonal::shared_ptr correction_noise=noiseModel::Isotropic::Sigma(3,1.0);
        GPSFactor gps_factor(X(correction_count),
                            Point3(gps(0),gps(1),gps(2)),//(N,E,D)
                            correction_noise);
        graph->add(gps_factor);
        //迭代更新求解imu预测值
        prop_state=imu_preintegrated_->predict(prev_state,prev_bias);
        initial_values.insert(X(correction_count), prop_state.pose());
        initial_values.insert(V(correction_count), prop_state.v());
        initial_values.insert(B(correction_count), prev_bias);
        //求解
        LevenbergMarquardtOptimizer optimizer(*graph,initial_values);
        Values result=optimizer.optimize();

        //更新下一步预计分初始值
        //导航状态
        prev_state=NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
        //偏导数
        prev_bias=result.at<imuBias::ConstantBias>(B(correction_count));
        //更新预计分值
        imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
        
        //计算角度误差和误差
        Vector3 gtsam_position=prev_state.pose().translation();
        //位置误差
        Vector3 position_error=gtsam_position-gps.head<3>();
        //误差的范数
        current_position_error=position_error.norm();//归一化
        
        //姿态误差
        Quaternion gtsam_quat=prev_state.pose().rotation().toQuaternion();
        Quaternion gps_quat(gps(6),gps(3),gps(4),gps(5));
        Quaternion quat_error=gtsam_quat*gps_quat.inverse();
        quat_error.normalized();//归一化
        Vector3 euler_angle_error(quat_error.x()*2,quat_error.y()*2,quat_error.z()*2);//转换为欧拉角误差
        current_orientation_error=euler_angle_error.norm();

        //输出误差
        cout << "Position error:" << current_position_error << "\t " << "Angular error:" << current_orientation_error << "\n";
              fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
              gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
              gps(0), gps(1), gps(2),
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

        output_time += 1.0;
        }
        else{
            cerr << "ERROR parsing file\n";
            return 1;
        }
    }
    fclose(fp);
    cout << "完成,结果见：" <<outputfile  << endl;
    return 0;
}
