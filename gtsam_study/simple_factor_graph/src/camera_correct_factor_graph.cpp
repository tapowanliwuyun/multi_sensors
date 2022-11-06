#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Cal3DS2.h>
// 用来做相机矫正
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <iostream>
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
}
using namespace gtsam;
using namespace std;
int main(int argc,char **argv){
    //创建点

    vector<Point3> points=createPoints();
    vector<Pose3> poses=createPoses();
    NonlinearFactorGraph graph;

    //1:添加起始点
    gtsam::Vector Vector6(6);
    Vector6 <<  0.3,0.3,0.3,0.1,0.1,0.1;
    noiseModel::Diagonal::shared_ptr prrior_error=noiseModel::Diagonal::Sigmas(Vector6);
    graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x',0),poses[0],prrior_error);

    //Cal3_S2:相机内参类 

    Cal3_S2 K(50.0,50.0,0.0,50.0,50.0);//fx_, fy_, s_, u0_, v0_;
    //Cal3DS2 K();//fx, fy, s, u0, v0, k1, k2, p1, p2
 
    //噪声 Isotropic 等方噪声
    noiseModel::Isotropic::shared_ptr measurementNoise=noiseModel::Isotropic::Sigma(2,1.0);
    for(size_t i=0;i<poses.size();i++){
        for(size_t j=0;j<points.size();j++){
            SimpleCamera camera(poses[i],K);
            //project 将世界坐标系，映射到图像坐标系
            Point2 measurement=camera.project(points[j]);
            //GeneralSFMFactor2 用作相机矫正
            graph.emplace_shared<GeneralSFMFactor2<Cal3_S2> >(measurement,measurementNoise,Symbol('x',i),Symbol('l',j),symbol('K',0));
        }
    }
    //路标的第一个点
    noiseModel::Isotropic::shared_ptr pointNoise=noiseModel::Isotropic::Sigma(3,0.1);
    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l',0),points[0],pointNoise);
    //相机的第一点
    noiseModel::Diagonal::shared_ptr calNoise = noiseModel::Diagonal::Sigmas((Vector(5) << 500, 500, 0.1, 100, 100).finished());
    graph.emplace_shared<PriorFactor<Cal3_S2> >(Symbol('K', 0), K, calNoise);

    //插入值
    Values initialEstimate;
    initialEstimate.insert(Symbol('K', 0), Cal3_S2(60.0, 60.0, 0.0, 45.0, 45.0));
    for (size_t i = 0; i < poses.size(); ++i)
      initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
    for (size_t j = 0; j < points.size(); ++j)
      initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));

    /* Optimize the graph and print results */
    Values result = DoglegOptimizer(graph, initialEstimate).optimize();
    result.print("Final results:\n");
      return 0;
}
