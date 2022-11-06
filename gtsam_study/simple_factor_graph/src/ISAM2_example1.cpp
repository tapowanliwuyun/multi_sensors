#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/inference/Key.h>
#include <iostream>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::P;

// Cal3_S2 相机内参结构体

typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;
//SmartProjectionFactor 专门用来实现相机位姿优化的一类
//  三角化 适用于单目相机，同时矫正标定相机和位姿
//  如果相机已经标定使用：SmartProjectionPoseFactor ：只矫正位姿

int main(int argc,char **argv){
  //初始化相机内参:fx_(1), fy_(1), s_(0), u0_(0), v0_(0)
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0,50.0,0,50,50));
  auto measurementNoise=noiseModel::Isotropic::Sigma(2,1.0);//1单位像素的误差值
  
  Vector6 sigmas;
  sigmas<<0.3,0.3,0.3,0.1,0.1,0.1;
  auto noise=noiseModel::Diagonal::Sigmas(sigmas);

  ISAM2Params parameters;
  parameters.relinearizeThreshold=0.01;      //差值大于0.1需要重新线性化
  parameters.relinearizeSkip=1;             // 每当有1个值需要重新线性化时， 对贝叶斯树进行更新
  parameters.enableRelinearization=true;    //是否可以重新线性化任意变量
  parameters.evaluateNonlinearError=false;  //是否计算线性化误差默认false
  parameters.cacheLinearizedFactors = false;  //default: true 是否保存保存线性化结果，可以优化性能，但是当线性化容易计算时会造成相反效果
  parameters.factorization=ISAM2Params::Factorization::QR;//默认为QR分级，还可以选用CHOESKY分解，但CHOESKY求解数值不稳定
  parameters.keyFormatter=DefaultKeyFormatter;//  debug时key值形式默认
  parameters.enableDetailedResults=true;     //是否计算返回 ISAM2Result::detailedResults，会增加计算量
  parameters.enablePartialRelinearizationCheck=false; //是否只进行部分更新功能
  parameters.findUnusedFactorSlots=false;//当要移除许多因子时，比如ISAM2做固定平滑时，启用此选项第一值进行插入时
                                          //避免出现NULL值，但是会造成每当有新值插入时必须查找

  ISAM2 isam(parameters);//贝叶斯树

  NonlinearFactorGraph graph;
  Values initialEstimate;
  Point3 point(0.0,0.0,1.0);
  //Rodrigues 将RPY转为悬装向量罗德里格旋转公式
  Pose3 delta(Rot3::Rodrigues(0.0,0.0,0.0),Point3(0.05,-0.10,0.20));
  Pose3 pose1(Rot3(), Point3(0.0, 0.0, 0.0));
  Pose3 pose2(Rot3(), Point3(0.0, 0.2, 0.0));
  Pose3 pose3(Rot3(), Point3(0.0, 0.4, 0.0));
  Pose3 pose4(Rot3(), Point3(0.0, 0.5, 0.0));
  Pose3 pose5(Rot3(), Point3(0.0, 0.6, 0.0));
  vector<Pose3> poses = {pose1, pose2, pose3, pose4, pose5};

  //构造图
  
  //起始点
  graph.emplace_shared<PriorFactor<Pose3>>(X(0), poses[0], noise);
  initialEstimate.insert(X(0), poses[0].compose(delta));

  SmartFactor::shared_ptr smartFactor(new SmartFactor(measurementNoise, K));
  smartFactor->add(PinholePose<Cal3_S2>(poses[0], K).project(point), X(0));
  graph.push_back(smartFactor);

  for(size_t i=1;i<5;i++){
      cout << "****************************************************" << endl;
      cout << "i = " << i << endl;
      graph.emplace_shared<PriorFactor<Pose3>>(X(i), poses[i], noise);
      initialEstimate.insert(X(i), poses[i].compose(delta));//SE compose 偏移小量
      //添加噪声模型
      PinholePose<Cal3_S2> camera(poses[i],K);  //仿真相机 位姿+内参
      Point2 measurement=camera.project(point); //point在5个位置投影
      cout << "Measurement " << i << "" << measurement << endl;

      //添加测量值
      smartFactor->add(measurement,X(i));
      
      //更新
      ISAM2Result result=isam.update(graph,initialEstimate);
      result.print();

      cout << "Detailed results:" << endl;
      
      //遍历键值信息
      for(auto keyedStatus:result.detail->variableStatus){
          const auto& status=keyedStatus.second;
          PrintKey(keyedStatus.first);
           cout << " {" << endl;
           cout<<"变量是否被被重新限制（重新线性化、添加新值、或者在被更新的根目录路径上）:"<<status.isReeliminated<<endl;
           cout<<"是否超过阈值(在平滑线性时超过是否阈值):"<<status.isAboveRelinThreshold<<endl; 
           cout<<"是否被设计被重新线性化:"<<status.isRelinearized<<endl;
           cout<<"是否被观测到（仅仅与添加的新元素相关):"<< status.isObserved << endl;
           cout<<"新值:"<<status.isNew<<endl;
           cout<<"是否为根团："<<status.inRootClique<<endl;
           cout << " }" << endl;
      }
      Values currentEstimate=isam.calculateBestEstimate();//calculateBestEstimate使用所有值进行回带
      currentEstimate.print("Current estimate:");
      boost::optional<Point3> pointEstimate=smartFactor->point(currentEstimate);
          if (pointEstimate) {
      cout << *pointEstimate << endl;
      }
      graph.resize(0);
      initialEstimate.clear();
  }
  return 0;
}
