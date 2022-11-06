#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include  <gtsam/slam/dataset.h>
#include  <gtsam/slam/InitializePose3.h>
#include  <fstream>
#include <vector>
#include  <stdint.h>
/**
*@brief 创建8个点
*@return 返回vector<Point3> points 长度为8 
*/ 
std::vector<gtsam::Point3> createPoints() {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

  return points;
}
/** 
 *@brief    
 *@param init  初始化pose，队列的第一个值，之后每一个都会在pose之上旋转delta(Pose3)
 *@param delta 旋转角度
 *@param steps 默认为8 返回Pose3的长度
 *@return vector<Pose3>
 */
std::vector<gtsam::Pose3> createPoses(
            const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,-M_PI/2), gtsam::Point3(30, 0, 0)),
            const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0,-M_PI/4,0), gtsam::Point3(sin(M_PI/4)*30, 0, 30*(1-sin(M_PI/4)))),
            int steps = 8) {

  // Create the set of ground-truth poses
  // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
  std::vector<gtsam::Pose3> poses;
  int i = 1;
  poses.push_back(init);
  for(; i < steps; ++i) {
    poses.push_back(poses[i-1].compose(delta));
  }

  return poses;
};
using namespace std;
using namespace gtsam;
int main(int argc, const char** argv) {
    //step1： 构造相机内参
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0,50.0,0.0,50.0,50.0));
    
    //step2: 构造相关参数
    noiseModel::Isotropic::shared_ptr mewsurementNoise=noiseModel::Isotropic::Sigma(2,1.0);
    vector<Point3> points=createPoints();
    vector<Pose3> poses=createPoses();

    //step3：构造因子图结构
    NonlinearFactorGraph graph;

    gtsam::Vector Vector6(6);
    Vector6 <<  0.3,0.3,0.3,0.1,0.1,0.1;
    noiseModel::Diagonal::shared_ptr prriornoise=noiseModel::Diagonal::Sigmas(Vector6);

    graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x',0),poses[0],prriornoise);
    for(size_t i=0;i<poses.size();i++){
        SimpleCamera camera(poses[i],*K);//初始化相机内参和姿态
        for(size_t j=0;j<points.size();j++){
            Point2 meansurement=camera.project(points[j]);//计算points[j]在当前相机的投影,z作为测量值
            graph.emplace_shared<GenericProjectionFactor<Pose3,Point3,Cal3_S2> >(meansurement,mewsurementNoise,symbol('x',i),symbol('l',j),K);
        }
    }
    //step4:添加数据,在原始数据上加入误差
    noiseModel::Isotropic::shared_ptr pointNoise=noiseModel::Isotropic::Sigma(3,0.1);
    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l',0),points[0],pointNoise);
    graph.print("Graph \n");

    Values initialEstimate;
    for(size_t i=0;i<poses.size();i++){
        initialEstimate.insert<Pose3>(symbol('x',i),poses[i].compose(Pose3(Rot3::Rodrigues(-0.1,0.2,0.25),Point3(0.05,-0.10,0.20))));
    };
    for(size_t j=0;j<points.size();++j){
        initialEstimate.insert<Point3>(Symbol('l',j),points[j]+Point3(-0.25,0.20,0.15));
        // cout<<"j"<<points[j].matrix()<<endl;
    }   
    initialEstimate.print("Initial Estimate :\n");
    //step5:执行性优化
    Values result=DoglegOptimizer(graph,initialEstimate).optimize();
    result.print("result \n:");
    cout << "起始误差= " << graph.error(initialEstimate) << endl;
    cout << "优化后误差 = " << graph.error(result) << endl;
    writeG2o(graph,initialEstimate,"../sfmtest/init.g2o");
    writeG2o(graph,result,"../sfmtest/result.g2o");
    return 0;
}



