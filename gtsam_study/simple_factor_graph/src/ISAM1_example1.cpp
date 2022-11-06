#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Cal3DS2.h>
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
    //相机内参，无畸变
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0,50.0,0.0,50.0,50.0));
    //模拟数据
    //相机观测噪声
    noiseModel::Isotropic::shared_ptr noise=noiseModel::Isotropic::Sigma(2,1.0);
    //创建路标点，位姿
    vector<Pose3> poses=createPoses();
    vector<Point3> points=createPoints();

    //创建ISAM：周期性重新平滑线性化和排序
    int relinearizeInterval=3;//新添加多少个之后更新，
    NonlinearISAM isam(relinearizeInterval);

    //创建图模型
    NonlinearFactorGraph graph;
    Values initialEstimate;

    //添加观则量
    for(int i=0;i<poses.size();i++){
        for(int j=0;j<points.size();j++){
            SimpleCamera camera(poses[i],*K);
            Point2 measurement=camera.project(points[j]);
            graph.emplace_shared<GenericProjectionFactor<Pose3,Point3,Cal3_S2> >(measurement,noise,Symbol('x',i),Symbol('l',j),K);
        }
        //初始化 路边真实点
        Pose3 noise(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
        Pose3 initial_xi=poses[i].compose(noise);//初始旋转值
        initialEstimate.insert(symbol('x',i),initial_xi);
        
        // 设置起始值同时设置当前帧的坐标系，并在第一个路标点上设置尺度，ISAM使用增量求解，至少需要两个值
        if(i==0){
         // Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
            gtsam::Vector Vector6(6);
            Vector6 <<  0.3,0.3,0.3,0.1,0.1,0.1;
            noiseModel::Diagonal::shared_ptr poseNoise=noiseModel::Diagonal::Sigmas(Vector6);
            graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x',0),poses[0],poseNoise);
            
            //添加prior l0
            noiseModel::Isotropic::shared_ptr pointnoise=noiseModel::Isotropic::Sigma(3,0.1);
            graph.emplace_shared<PriorFactor<Point3> >(Symbol('l',0),points[0],pointnoise);
            //初始化噪声
            Point3 noise(-0.25, 0.20, 0.15);
            //插入值
            for(int j=0;j<points.size();j++){
                Point3 initial_lj=points[j]+noise;
                initialEstimate.insert(Symbol('l',j),initial_lj);
            }
        }
        else{
            isam.update(graph,initialEstimate);
            Values currentEsimate=isam.estimate();
            cout << "****************************************************" << endl;
            cout << "Frame " << i << ": " << endl;
            currentEsimate.print("Current estimate: ");
            graph.resize(0);
            initialEstimate.clear();
        }
    }
    return 0;
}

