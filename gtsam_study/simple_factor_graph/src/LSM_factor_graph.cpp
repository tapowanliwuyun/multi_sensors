//BetweenFactor 包含描述两个因子之间相关操作类
//              和初始状态概率矩阵
#include <gtsam/slam/BetweenFactor.h>//
//误差
#include <gtsam/slam/PriorFactor.h> 
//非线性
#include <gtsam/nonlinear/NonlinearFactorGraph.h>//
//列文伯格求解方程组
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>//
//Pose2 类库 把3d点用作(x,y,theta)表示
#include <gtsam/geometry/Pose2.h>//
#include <iostream>//
//计算边缘分布
#include <gtsam/nonlinear/Marginals.h>//

//指定列文伯格迭代的初始值
#include <gtsam/nonlinear/Values.h>//
#include <gtsam/base/Vector.h>//
#include <CppUnitLite/TestHarness.h>// 单元测试库
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>

using namespace gtsam;
using namespace std;
//必须实现计算误差

class UnaryFactor: public NoiseModelFactor1<Pose2> {
//NoiseModelFactor1 gtsam 可以实现单边NoiseModelFactor
//                  使用时需要实现evaluateError()
//                  模板为VALUE,可以支持vector,Rot3,Pose3,包括gtsam 内的李群
  double mx_,my_;
  //mx_,my_:测量值
  public:
    typedef boost::shared_ptr<UnaryFactor> Ptr;
    //构造函数
    UnaryFactor(Key i,double x,double y,const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, i), mx_(x), my_(y) {;}
    //typedef noiseModel::Base::shared_ptr SharedNoiseModel;
    //noiseModel::Base 一个虚基类
    
    virtual ~UnaryFactor(){}


    //计算误差并且当值无效时，返回无效的向量，有时还需要计算雅克比矩阵
    Vector evaluateError(const Pose2& q,boost::optional<Matrix&> H=boost::none) const{
      if(H)(*H)=(Matrix(2,3)<<1.0,0.0,0.0,0.0,1.0,0.0).finished();
      return (Vector(2)<<q.x()-mx_,q.y()-my_).finished();
    }

    //克隆函数
    virtual gtsam::NonlinearFactor::shared_ptr clone() const{
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this)));
    ;}


};

int main(int argc, char const *argv[])
{
  //步骤1：创建图
  NonlinearFactorGraph graph;
  
  //步骤2：添加odometry 因子，构造有连接因子之间的关系
  noiseModel::Diagonal::shared_ptr odometriyNose=noiseModel::Diagonal::Sigmas(Vector3(0.2,0.2,0.1));
  graph.emplace_shared<BetweenFactor<Pose2> >(1,2,Pose2(2.0,0,0),odometriyNose);
  graph.emplace_shared<BetweenFactor<Pose2> >(2,3,Pose2(2.0,0,0),odometriyNose);

  //步骤2：添加观测值：“GPS”信息 vector(x,y)
  noiseModel::Diagonal::shared_ptr unaryNosie=noiseModel::Diagonal::Sigmas((Vector2(0.1,0.1)));
  graph.emplace_shared<UnaryFactor>(1,0.0,0.0,unaryNosie);
  graph.emplace_shared<UnaryFactor>(2,2.0,0.0,unaryNosie);
  graph.emplace_shared<UnaryFactor>(3,4,0.0,unaryNosie);

  //步骤3：根据数据结构插入数据，为了有效验证这些值均有误差
  Values InitialEstimate;
  InitialEstimate.insert(1,Pose2(0.5,0.0,0.2));
  InitialEstimate.insert(2,Pose2(2.3,0.1,-0.2));
  InitialEstimate.insert(3,Pose2(4.1,0.1,0.1));
  InitialEstimate.print("\nInitial Estimate:\n");

  //步骤4：选择方法进行迭代计算，可以设置收敛条件，如果不设置则使用默认参数
  LevenbergMarquardtOptimizer optimizer(graph,InitialEstimate);
  Values result=optimizer.optimize();
  result.print("Final Result:\n");

  
  cout<<"result 1:\n"<<result.at<Pose2>(1).matrix()<<endl;
  cout<<"result 2:\n"<<result.at<Pose2>(2).matrix()<<endl;
  cout<<"result 3:\n"<<result.at<Pose2>(3).matrix()<<endl;
  // result.at<Pose2>(1).print();
  // cout<<"result0:"<<Pose2(-1.5424e-14,1.34169e-15,-1.38879e-16).matrix()<<endl;
  //这里最后结果表示最优值

  //计算边缘概率密度,表示不确定度
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  return 0;
}


