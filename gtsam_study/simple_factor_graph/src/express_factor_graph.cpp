
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
using namespace std;
using namespace gtsam;
//使用express 使用
typedef BearingRange<Pose2, Point2> BearingRange2D;
int main(int argc, char** argv) {

  ExpressionFactorGraph graph;

  //express 中Pose2相应类型为 Pose2_ (factor id)
  //即在相应类型后加一个_
  Pose2_ x1(1),x2(2),x3(3);
  Point2_ l1(4),l2(5);
  Pose2 prior(0,0,0);
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  //添加先验值 更加简单
//  graph.emplace_shared<PriorFactor<Pose2> >(x1,prior,priorNoise);
   graph.addExpressionFactor(x1, Pose2(0, 0, 0), priorNoise);

  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  Rot2 bearing11=Rot2::fromDegrees(45);//l1,x1,x2,构成的夹角
  Rot2 bearing21=Rot2::fromDegrees(90);//x1,x2,l1,构成的夹角
  Rot2 bearing32=Rot2::fromDegrees(90);//x1,x2,l2,构成的夹角
  double range11=sqrt(8),range21=2.0,range32=2.0;
  Pose2 odometry1(2.0, 0.0, 0.0);
  Pose2 odometry2(2, 0.0, 0.0);
  //构造因子图关系图
  graph.addExpressionFactor(between(x1,x2),odometry1,model);
  graph.addExpressionFactor(between(x2,x3),odometry2,model);
  noiseModel::Diagonal::shared_ptr measurementNoise=noiseModel::Diagonal::Sigmas(Vector2(0.1,0.2));
  //节点关系表达式
  auto pre_x1l1=Expression<BearingRange2D>(BearingRange2D::Measure,x1,l1);
  //两个节点之间的位姿关系
  auto measure_x1l1=BearingRange2D(bearing11, range11);
  auto pre_x2l1=Expression<BearingRange2D>(BearingRange2D::Measure,x2,l1);
  auto measure_x2l1=BearingRange2D(bearing21, range21);
  auto pre_x3l2=Expression<BearingRange2D>(BearingRange2D::Measure,x3,l2);
  auto measure_x3l2=BearingRange2D(bearing32, range32);
  graph.addExpressionFactor(pre_x1l1, measure_x1l1, measurementNoise);
  graph.addExpressionFactor(pre_x2l1, measure_x2l1, measurementNoise);
  graph.addExpressionFactor(pre_x3l2, measure_x3l2, measurementNoise);

  Values initialEstimate;
  
  	//插入实际值
    initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(2, Pose2(2.3, 0.1,-0.2));
    initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
    initialEstimate.insert(4, Point2(1.8, 2.1));
    initialEstimate.insert(5, Point2(4.1, 1.8));

    //求解
    GaussNewtonOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    //优化后的结果
    result.print("Final Result:\n");

    Marginals marginals(graph, result);
    cout<<"marginals x1"<<marginals.marginalCovariance(1)<<endl;
    cout<<"marginals x2"<<marginals.marginalCovariance(2)<<endl;
    cout<<"marginals x3"<<marginals.marginalCovariance(3)<<endl;
    cout<<"marginals l1"<<marginals.marginalCovariance(4)<<endl;
    cout<<"marginals l2"<<marginals.marginalCovariance(5)<<endl;

  return 0;
}

