#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <vector>
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
    //相机内参K，无畸变
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0,50.0,0,50.0,50.0));

    //模拟数据
    auto measurementnoise=noiseModel::Isotropic::Sigma(2,1.0);
    vector<Point3> points=createPoints();
    vector<Pose3>  poses=createPoses();

    //与ISAM1不同，ISAM1进行周期性批量处理，ISAM2进行部分平滑线性
    ISAM2Params parameters;
    parameters.relinearizeThreshold=0.01;   //重新线性化平滑值
    parameters.relinearizeSkip=1;
    ISAM2 isam(parameters);
    
    NonlinearFactorGraph graph;
    Values initialEstimate;

    for(size_t i=0;i<poses.size();i++){
        //测量值
        for(size_t j=0;j<points.size();j++){
            SimpleCamera camera(poses[i],*K);
            Point2 measurement=camera.project(points[j]);
            graph.emplace_shared<GenericProjectionFactor<Pose3,Point3,Cal3_S2> >(measurement,measurementnoise,Symbol('x',i),Symbol('l',j),K);

        }
        Pose3 kDeltaPose(Rot3::Rodrigues(-0.1,0.2,0.25),Point3(0.05,-0.10,0.20));
        initialEstimate.insert(symbol('x',i),poses[i]*kDeltaPose);
        if(i==0){
            gtsam::Vector Vector6(6);
            Vector6 <<  0.3,0.3,0.3,0.1,0.1,0.1;
            auto kPosePrior=noiseModel::Diagonal::Sigmas(Vector6);
            graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x',0),poses[0],kPosePrior);

            noiseModel::Isotropic::shared_ptr kPointPrior=noiseModel::Isotropic::Sigma(3,0.1);
            graph.emplace_shared<PriorFactor<Point3> >(Symbol('l',0),points[0],kPointPrior);
            Point3 kDeltaPoints(-0.25,0.20,0.15);
            for(size_t j=0;j<points.size();j++){
                initialEstimate.insert<Point3>(Symbol('l',j),points[j]+kDeltaPoints);
            }
        }
        else{
            //每次update 都会进行一次非线性迭代求解，如果需要更加准确的数据，可以调用多个，但会消耗额外的时间
            isam.update(graph,initialEstimate);
            isam.update();
            Values currentEstimate=isam.calculateBestEstimate();
            cout << "****************************************************" << endl;
            cout << "Frame " << i << ": " << endl;
            currentEstimate.print("Current estimate: ");
            graph.resize(0);
            initialEstimate.clear();
        }

    }
    return 0;
}

