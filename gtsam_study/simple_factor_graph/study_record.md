# 0 简介

# １gtsam的学习记录
## 1.0 定义因子图
### 1.0.1 定义高斯因子图
1. 头文件
```cpp
//非线性
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
```
2. 使用
```cpp
    NonlinearFactorGraph graph;//高斯因子图
```
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

或者
定义了一个因子图指针
```cpp
    NonlinearFactorGraph::shared_ptr graph;
```
如[loopclosure_factor_graph](./src/loopclosure_factor_graph.cpp)
### 1.0.2 定义因子变量
1. 头文件
```cpp
//每个元必须key，在gtsam中
//可以使用整数int，或者symbol自定义key
#include <gtsam/inference/Symbol.h>
```
2. 使用
```cpp
     Symbol x1('x',1),x2('x',2),x3('x',3);
     Symbol l1('l',1),l2('l',2);
    // cout<<"x1"<<x1.key()<<endl;
    // cout<<"x2"<<x2.key()<<endl;
    // cout<<"x3"<<x3.key()<<endl;
    // cout<<"l1"<<l1.key()<<endl;
    // cout<<"l2"<<l2.key()<<endl;
```
输出：
```cpp
x18646911284551352321
x28646911284551352322
x38646911284551352323
l17782220156096217089
l27782220156096217090
```
如4[symbol_factor_graph](./src/symbol_factor_graph.cpp)

## 1.1 往因子图中添加因子约束

```cpp
    //3d点
    Pose2 priorMean(0,0,0);//先验值 x,y,theta

    //添加噪声，替他还有gussian,Robust、Unit 噪声
    noiseModel::Diagonal::shared_ptr priorNoise=noiseModel::Diagonal::Sigmas(Vector3(0.3,0.3,0.1));

    //在因子图中添加因子（//初始化）
    //graph.add(PriorFactor<Pose2>(1,priorMean,odometryNoise));//添加单个
    graph.emplace_shared<PriorFactor<Pose2> >(1,priorMean,priorNoise);//可以添加多个
    //graph.emplace_shared<PriorFactor<Pose2> >(x1,priorMean,priorNoise);//可以添加多个
```
* **1**表示：第一个因子
* **priorMean**表示：估计值
* **priorNoise**表示：噪声
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

## 1.2 初始化观测矩阵
```cpp
    Values initial;
    //初始化观测矩阵
    initial.insert(1, Pose2(0.5, 0.0, 0.2));//插入观测数据
    initial.insert(2, Pose2(2.3, 0.1, -0.2));
    initial.insert(3, Pose2(4.1, 0.1, 0.1));
```
* **1**表示：第1个因子
* **Pose2(0.5, 0.0, 0.2)**表示：该因子的观测数据
* 这里是在构建好因子约束之后为因子添加观测值
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

## 1.3 开始执行优化
### 1.3.1 使用列文伯格马夸尔特迭代求解
1. 头文件
```cpp
//信任区域法
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
```
2. 使用
```cpp
    //使用列文伯格迭代求解最小二乘:min(z-gussan（x+w)-gussan(y+v)+gussan(z+e))
    Values result=LevenbergMarquardtOptimizer(graph,initial).optimize();//经过三次迭代后的预测值
```
* **graph**表示：1.1中构建的因子约束
* **initial**表示：1.2中赋予的初值
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

或者
```cpp
  //步骤4：选择方法进行迭代计算，可以设置收敛条件，如果不设置则使用默认参数
  LevenbergMarquardtOptimizer optimizer(graph,InitialEstimate);
  Values result=optimizer.optimize();
```
如2 [LSM_factor_graph](./src/LSM_factor_graph.cpp)

### 1.3.2 lago方法初始化因子图
1. 头文件
```cpp
#include <gtsam/slam/lago.h>
```
2. 使用
* 查看3.3小节
### 1.3.3 高斯牛顿求解
查看4.5小节

### 1.3.4 Dogleg求解
查看5.9小节
## 1.4 计算非线性因子图中变量的高斯边缘概率
```cpp
    //计算非线性因子图中变量的高斯边缘概率
    Marginals marginals(graph, result);
    
    cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;   
    cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
    cout << "x1 marginalInformation:\n" << marginals.marginalInformation(1) << endl;
    cout << "x2 marginalInformation:\n" << marginals.marginalInformation(2) << endl;
    cout << "x3 marginalInformation:\n" << marginals.marginalInformation(3) << endl;
```
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

## 1.5  查询优化结果
```cpp
    //查询优化结果
    for (const gtsam::Values::ConstKeyValuePair& key_value: result )
    {   
        cout<<"key \n"<<key_value.key<<endl;
        Pose2 pose = key_value.value.cast<gtsam::Pose2>();//类型转换
        cout<<"data:\n"<<pose.matrix()<<endl;
    }
```
* **result**表示：1.3节定义的```Values result```
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

## 1.6 遍历因子图节点
```cpp
    //遍历
    for ( gtsam::NonlinearFactor::shared_ptr factor: graph )//遍历因子图节点
    {
         gtsam::BetweenFactor<gtsam::Pose2>::shared_ptr f = dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose2>>( factor );//提取边
         if (f){//如果有边
                 gtsam::SharedNoiseModel model = f->noiseModel();//提取这个边的噪声模型
                 gtsam::noiseModel::Diagonal::shared_ptr DiagModel = dynamic_pointer_cast<gtsam::noiseModel::Diagonal>( model );//从噪声模型提取高斯模型
                 if ( DiagModel )//如果顺利提取高斯模型
                {       
                    gtsam::Matrix info;   
                    gtsam::Vector DiagModel_sigma=DiagModel->sigmas();//也就是自己构建数据的时候构建的噪声模型
                    cout<<"DiagModel_sigma:"<<DiagModel_sigma<<endl;//输出误差
                    Pose2 pose=f->measured();
                    cout<<"data:"<<pose.matrix()<<endl;//显示测量
                }
         }
    }
```
* 这个应该主要是用来查询因子图中的东西的
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

## 1.7 gtsam的可视化
### 1.7.1 因子图的查看
```cpp
NonlinearFactorGraph graph;//高斯因子图
...
graph.print("Graph \n");   
```
如5.2  [sfm_factor_graph](./src/sfm_factor_graph.cpp)

### 1.7.2 初值的查看
```cpp
Values initial;
...
initial.print("\nInitial Estimate:\n"); // print
```
如5.2 [sfm_factor_graph](./src/sfm_factor_graph.cpp)

### 1.7.3 查看当前节点值在因子图中的误差
* 查看5.10小节

# 2 构建因子
## 2.1 官方因子库
### 2.1.1 一元边 位姿先验因子 可以理解为固定点
#### 2.1.1.1 PriorFactor因子
1. 头文件
```cpp
#include <gtsam/slam/PriorFactor.h> 
```
2. 使用
```cpp
    Pose2 priorMean(0,0,0);//先验值 x,y,theta

    //添加噪声，替他还有gussian,Robust、Unit 噪声
    noiseModel::Diagonal::shared_ptr priorNoise=noiseModel::Diagonal::Sigmas(Vector3(0.3,0.3,0.1));

    //在因子图中添加因子（//初始化）
    //graph.add(PriorFactor<Pose2>(1,priorMean,odometryNoise));//添加单个
    graph.emplace_shared<PriorFactor<Pose2> >(1,priorMean,priorNoise);//可以添加多个
```
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

#### 2.1.1.2 NonlinearEquality因子
1. 头文件
```cpp
#include <gtsam/nonlinear/NonlinearEquality.h>
```

2. 使用
```cpp
  //graph.emplace_shared<NonlinearEquality<Pose3> >(1, Pose3());
  graph.emplace_shared<PriorFactor<Pose3>>(1, Pose3());
```
如[StereoVOExample.cpp](../gtsam_study_cvlife/cap7_visual_odometry_in_GTSAM/src/StereoVOExample.cpp)

### 2.1.2  二元边 odom因子 BetweenFactor
1. 头文件
```cpp
#include <gtsam/slam/BetweenFactor.h>
```

2. 使用
```cpp
    //Eigen::Vector3
    //sigma 函数 :1/(1+e^(-z))
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

    Pose2 odometry(2.0,0,0);

    graph.emplace_shared<BetweenFactor<Pose2> >(1,2,odometry,odometryNoise);
    //graph.emplace_shared<BetweenFactor<Pose2> >(x1,x2,odometry,odometryNoise);
```
如1.1 [simple_factor_graph](./src/simple_factor_graph.cpp)

### 2.1.3 二元边 观测因子 BearingRangeFactor
1. 头文件
```cpp
#include <gtsam/sam/BearingRangeFactor.h>
```
2. 使用
```cpp
    //添加Range-Bearing 到两俩个不同的landmarks
    noiseModel::Diagonal::shared_ptr measurementNoise=noiseModel::Diagonal::Sigmas(Vector2(0.1,0.2));

    Rot2 bearing11=Rot2::fromDegrees(45);//l1,x1,x2,构成的夹角

    double range11=sqrt(8);

    graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x1, l1, bearing11, range11, measurementNoise);
    //    initialEstimate.insert(x1, Pose2(0.5, 0.0, 0.2));
    //    initialEstimate.insert(l1, Point2(1.8, 2.1));
```
如4[symbol_factor_graph](./src/symbol_factor_graph.cpp)
### 2.1.4 三元边 重投影因子  GeneralSFMFactor2
* 参考5.6.1小节
### 2.1.5 二元边 重投影因子  GenericProjectionFactor
* 参考5.6.2 小节

## 2.2 自定义因子
### 2.2.1 示例1 构建一元边  可用于GPS约束等
1. 定义
```cpp
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
```
2. 使用
```cpp
  noiseModel::Diagonal::shared_ptr unaryNosie=noiseModel::Diagonal::Sigmas((Vector2(0.1,0.1)));
  graph.emplace_shared<UnaryFactor>(1,0.0,0.0,unaryNosie);
```
如2 [simple_factor_graph](./src/simple_factor_graph.cpp)


# 3 gtsam与g2o文件的联合使用
* x.x.1为第一个案例，主要是2d的点
* x.x.2为第二个案例，主要是3d的点
* 头文件
```cpp
#include <gtsam/slam/lago.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <fstream>

```
```cpp
#include <iostream>//
#include  <gtsam/slam/InitializePose3.h>
#include  <gtsam/slam/dataset.h>
#include  <gtsam/slam/BetweenFactor.h>
#include  <gtsam/slam/PriorFactor.h>
#include  <fstream>
#include  <string>
```

## 3.1 读取g2o文件中的约束和初值
### 3.1.1 
```cpp
    string g2oFile="../src/loop_closure.g2o";
    NonlinearFactorGraph::shared_ptr graph;
    Values::shared_ptr initial;

//boost::tie:将两个内部参数赋值（最多可以有10个），并将其合并为一个类型范围
//  template<class T0, class T1>
//  inline typename detail::tie_mapper<T0, T1>::type
//  tie(T0& t0, T1& t1) {
//  typedef typename detail::tie_mapper<T0, T1>::type t;
//  return t(t0, t1);
// }
    boost::tie(graph,initial)=readG2o(g2oFile);
    //readG2o(file,is3D,kernelFuntionType)
    //  file:文件路径
    //  is3D：默认false 是否为3d问题
    //  kernelFuntionType：采用何种核函数
    NonlinearFactorGraph graphWithPrior=*graph;
```
如3.1 [loopclosure_factor_graph](./src/loopclosure_factor_graph.cpp)
### 3.1.2 
```cpp
    string g2ofile="../src/sphere.g2o";
    NonlinearFactorGraph::shared_ptr graph;
    Values::shared_ptr initial;
    bool is3D=true;
    boost::tie(graph,initial)=readG2o(g2ofile,is3D);
    NonlinearFactorGraph graphPrior=*graph;
```
如3.2 [sphere_factor_graph](./src/sphere_factor_graph.cpp)
## 3.2 添加起始点
### 3.2.1 
```cpp
    noiseModel::Diagonal::shared_ptr priorModel=noiseModel::Diagonal::Variances(gtsam::Vector3(1e-6,1e-6,1e-8));
    graphWithPrior.add(PriorFactor<Pose2>(0,Pose2(),priorModel));//添加起始点
```
如3.1 [loopclosure_factor_graph](./src/loopclosure_factor_graph.cpp)

### 3.2.2 
```cpp
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6,1e-6,1e-6,1e-4,1e-4,1e-4;
    noiseModel::Diagonal::shared_ptr priormpdel=noiseModel::Diagonal::Variances(Vector6);
    Key firstKey=0;
    for(const Values::ConstKeyValuePair& key_value:*initial){
        std::cout<<"添加 起始因子 到g2o"<<std::endl;
        firstKey=key_value.key;//获取其第一个因子键值(因为不一定起始为0)
        graphPrior.add(PriorFactor<Pose3>(firstKey,Pose3(),priormpdel));
        break;
    }
```
如3.2 [sphere_factor_graph](./src/sphere_factor_graph.cpp)
## 3.3 利用lago快速初始化因子图
### 3.3.1 
```cpp
    //使用lago 快速初始化因子图
    Values estimate=lago::initialize(graphWithPrior);//返回因子图中的值
```
如3.1 [loopclosure_factor_graph](./src/loopclosure_factor_graph.cpp)
### 3.3.2 
```cpp
    Values initialzation=InitializePose3::initialize(graphPrior);//初始化
```
如3.2 [sphere_factor_graph](./src/sphere_factor_graph.cpp)

## 3.4 输出初始化后的估计值
### 3.4.1 
```cpp
    estimate.print("estimateLago");
```
如3.1 [loopclosure_factor_graph](./src/loopclosure_factor_graph.cpp)
### 3.4.2 
```cpp
    initialzation.print("initialzatoin");
```
如3.2 [sphere_factor_graph](./src/sphere_factor_graph.cpp)
## 3.5 将结果写入g2o文件中
### 3.5.1 
```cpp
    string outputfile="../src/loop_closure_optimized.g2o";
    writeG2o(*graph,estimate,outputfile);
    //graph:图结构
    //estimate：估计值
    //outputfile：保存路径
```
如3.1 [loopclosure_factor_graph](./src/loopclosure_factor_graph.cpp)
### 3.5.2 
 ```cpp
    string output="../src/correctsphere.g2o";
    writeG2o(*graph,initialzation,output);
 ```
如3.2 [sphere_factor_graph](./src/sphere_factor_graph.cpp)

### 3.5.3 
```cpp
    writeG2o(graph,result,"/home/bupo/my_study/multi_sensors/gtsam_study/symbol_factor_graph/src/landmark.g2o");
```
如4[symbol_factor_graph](./src/symbol_factor_graph.cpp)

# 4 express框架
头文件
```cpp
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
```
如 [express_factor_graph](./src/express_factor_graph.cpp)

## 4.1 定义express框架因子图
1. 头文件
```cpp
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
```
2. 使用
```cpp
  ExpressionFactorGraph graph;
```

## 4.2 定义express表达式因子
1. 头文件
```cpp
#include <gtsam/slam/expressions.h>
```
2. 使用
```cpp
  //express 中Pose2相应类型为 Pose2_ (factor id)
  //即在相应类型后加一个_
  Pose2_ x1(1),x2(2),x3(3);
  Point2_ l1(4),l2(5);
```
## 4.3 添加因子约束
```cpp

Pose2 prior(0,0,0);
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  //添加先验值 更加简单
//  graph.emplace_shared<PriorFactor<Pose2> >(x1,prior,priorNoise);
   graph.addExpressionFactor(x1, Pose2(0, 0, 0), priorNoise);
```

```cpp
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  Pose2 odometry1(2.0, 0.0, 0.0);
  Pose2 odometry2(2, 0.0, 0.0);

  //构造因子图关系图
  graph.addExpressionFactor(between(x1,x2),odometry1,model);
  graph.addExpressionFactor(between(x2,x3),odometry2,model);
```
```cpp
  Rot2 bearing11=Rot2::fromDegrees(45);//l1,x1,x2,构成的夹角
  Rot2 bearing21=Rot2::fromDegrees(90);//x1,x2,l1,构成的夹角
  Rot2 bearing32=Rot2::fromDegrees(90);//x1,x2,l2,构成的夹角
  double range11=sqrt(8),range21=2.0,range32=2.0;

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
```
## 4.4 插入实际值

```cpp
  Values initialEstimate;
  
  	//插入实际值
    initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(2, Pose2(2.3, 0.1,-0.2));
    initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
    initialEstimate.insert(4, Point2(1.8, 2.1));
    initialEstimate.insert(5, Point2(4.1, 1.8));
```
## 4.5 高斯牛顿求解
1. 头文件
```cpp
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
```
2. 使用
```cpp
    //求解
    GaussNewtonOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
```
## 4.6 可视化边缘化结果
```cpp
    Marginals marginals(graph, result);
    cout<<"marginals x1"<<marginals.marginalCovariance(1)<<endl;
    cout<<"marginals x2"<<marginals.marginalCovariance(2)<<endl;
    cout<<"marginals x3"<<marginals.marginalCovariance(3)<<endl;
    cout<<"marginals l1"<<marginals.marginalCovariance(4)<<endl;
    cout<<"marginals l2"<<marginals.marginalCovariance(5)<<endl;
```

# 5 相机矫正
1. 头文件
```cpp
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
```
该小节内未说明就是来自如[camera_correct_factor_graph](./src/camera_correct_factor_graph.cpp)
1. 头文件
```cpp

```
2. 使用
```cpp

```

## 5.1 自定义节点（位姿和路标）数据
```cpp
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
```
```cpp
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
```
## 5.2 使用自定义的节点
```cpp
    //创建点

    vector<Point3> points=createPoints();
    vector<Pose3> poses=createPoses();
```
## 5.3 定义因子图
```cpp
    NonlinearFactorGraph graph;
```
## 5.4 位姿添加初始点
```cpp
    //1:添加起始点
    gtsam::Vector Vector6(6);
    Vector6 <<  0.3,0.3,0.3,0.1,0.1,0.1;
    noiseModel::Diagonal::shared_ptr prrior_error=noiseModel::Diagonal::Sigmas(Vector6);
    graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x',0),poses[0],prrior_error);
```
## 5.5 定义相机类
### 5.5.1 非指针类型相机内参
```cpp
    //Cal3_S2:相机内参类 

    Cal3_S2 K(50.0,50.0,0.0,50.0,50.0);//fx_, fy_, s_, u0_, v0_;
    //Cal3DS2 K();//fx, fy, s, u0, v0, k1, k2, p1, p2
```


### 5.5.2 非指针类型相机内参
```cpp
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0,50.0,0.0,50.0,50.0));

```
如[sfm_factor_graph](./src/sfm_factor_graph.cpp)

## 5.6 定义重投影约束 二元边/三元边
### 5.6.1 重投影约束，三元边，优化位姿、路标、相机内参
1. 头文件
```cpp
// 用来做相机矫正
#include <gtsam/slam/GeneralSFMFactor.h>
```

2. 使用
```cpp
    //噪声 Isotropic 等方噪声
    noiseModel::Isotropic::shared_ptr measurementNoise=noiseModel::Isotropic::Sigma(2,1.0);
    for(size_t i=0;i<poses.size();i++){
        for(size_t j=0;j<points.size();j++){
            SimpleCamera camera(poses[i],K);
            //project 将世界坐标系，映射到图像坐标系
            Point2 measurement=camera.project(points[j]);
            //GeneralSFMFactor2 用作相机矫正
            graph.emplace_shared<GeneralSFMFactor2<Cal3_S2> >(measurement,measurementNoise,Symbol('x',i),Symbol('l',j),symbol('K',0));//不仅优化位姿、路标点、还会优化相机的内参K
        }
    }
```

### 5.6.2 重投影约束，二元边，优化位姿、路标
1. 头文件
```cpp
#include <gtsam/slam/ProjectionFactor.h>
```
2. 使用
```cpp
    noiseModel::Isotropic::shared_ptr mewsurementNoise=noiseModel::Isotropic::Sigma(2,1.0);
    for(size_t i=0;i<poses.size();i++){
        SimpleCamera camera(poses[i],*K);//初始化相机内参和姿态
        for(size_t j=0;j<points.size();j++){
            Point2 meansurement=camera.project(points[j]);//计算points[j]在当前相机的投影,z作为测量值
            graph.emplace_shared<GenericProjectionFactor<Pose3,Point3,Cal3_S2> >(meansurement,mewsurementNoise,symbol('x',i),symbol('l',j),K);
        }
    }
```
如[sfm_factor_graph](./src/sfm_factor_graph.cpp)

## 5.7 添加路标、相机初始点
```cpp
   //路标的第一个点
    noiseModel::Isotropic::shared_ptr pointNoise=noiseModel::Isotropic::Sigma(3,0.1);
    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l',0),points[0],pointNoise);
    //相机的第一点，这里在使用二元边重投影约束的时候不需要
    noiseModel::Diagonal::shared_ptr calNoise = noiseModel::Diagonal::Sigmas((Vector(5) << 500, 500, 0.1, 100, 100).finished());
    graph.emplace_shared<PriorFactor<Cal3_S2> >(Symbol('K', 0), K, calNoise);
```
## 5.8 添加初始值
```cpp
    //插入值
    Values initialEstimate;
    initialEstimate.insert(Symbol('K', 0), Cal3_S2(60.0, 60.0, 0.0, 45.0, 45.0));//这里在使用二元边重投影约束的时候不需要
    for (size_t i = 0; i < poses.size(); ++i)
      initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
    for (size_t j = 0; j < points.size(); ++j)
      initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));
```
## 5.9 Dogleg优化
1. 头文件
```cpp
#include <gtsam/nonlinear/DoglegOptimizer.h>
```
1. 使用
```cpp
    /* Optimize the graph and print results */
    Values result = DoglegOptimizer(graph, initialEstimate).optimize();
```

## 5.10 可视化误差
```cpp
    result.print("result \n:");
    cout << "起始误差= " << graph.error(initialEstimate) << endl;
    cout << "优化后误差 = " << graph.error(result) << endl;
```
## 5.11 保存文件
```cpp
    writeG2o(graph,initialEstimate,"../sfmtest/init.g2o");
    writeG2o(graph,result,"../sfmtest/result.g2o");
```
# 6 IMU预积分因子和GPS因子
1. 头文件
```cpp
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
```
* 如[imu_factor_graph](./src/imu_factor_graph.cpp)

1. 头文件
```cpp

```
2. 使用
```cpp

```


## 6.1 定义symbol因子变量和.csv数据文件
```cpp
using namespace std;
using namespace gtsam;
using symbol_shorthand::X;//用作表示    姿态(x,y,z,r,p,y)
using symbol_shorthand::V;//用表示      速度导数(xdot,ydot,zdot)
using symbol_shorthand::B;//陀螺仪残差  (ax,ay,az,gx,gy,gz)
string inputfile="../data/imu/imuAndGPSdata.csv";   
string outputfile="../data/imu/result1.csv";
```
## 6.2 定义预积分
1. 头文件
```cpp
#include <gtsam/navigation/CombinedImuFactor.h>
```
2. 使用
```cpp
PreintegrationType *imu_preintegrated_;
//    Matrix3 biasAccCovariance;     3*3矩阵 加速度计的协防差矩阵，（可以根据残差计算加速度雅克比矩阵逐步更新）
//    Matrix3 biasOmegaCovariance;   3*3矩阵 陀螺仪的协防差矩阵， （可以根据残差计算雅克比矩阵递归更新预计分值，防止从头计算）
//    Matrix6 biasAccOmegaInt;       6*6矩阵 位置残关于加速度和速度残差的协防差，用作更新预计分
```
## 6.3 向csv文件中写入和读取
1. 写入
```cpp
    FILE* fp=fopen(outputfile.c_str(),"w+");
    //输出
    fprintf(fp,"#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,gt_qy,gt_qz,gt_qw\n");

```
2. 读取第一行数据
```cpp
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
```
## 6.4 构造旋转、位置、偏差初值
```cpp
    Rot3 prior_rotation=Rot3::Quaternion(initial_state(6),initial_state(3),initial_state(4),initial_state(5));//初始位姿
    Point3 prior_point(initial_state(0),initial_state(1),initial_state(2));//初始位置
    Pose3 prior_pose(prior_rotation,prior_point);                               //初始位姿
    Vector3 prior_velocity(initial_state(7),initial_state(8),initial_state(9)); //初始速度
    imuBias::ConstantBias prior_imu_bias;//残差，默认设为0
```
## 6.5 定义因子图初始点并且赋初值
```cpp
    //初始化值，定义初始点并且为赋予初始值
    Values initial_values;
    int correction_count=0;
    //位姿
    initial_values.insert(X(correction_count),prior_pose);
    //速度
    initial_values.insert(V(correction_count),prior_velocity);
    //残差
    initial_values.insert(B(correction_count),prior_imu_bias);
    cout << "initial state:\n" << initial_state.transpose() <<endl;
```
## 6.6 在因子图中添加初值约束
```cpp
    //设置噪声模型
    //一般为设置为对角噪声
    //在因子图中添加初值约束
    gtsam::Vector Vector6(6);
    Vector6 << 0.01,0.01,0.01,0.5,0.5,0.5;
    noiseModel::Diagonal::shared_ptr pose_noise_model=noiseModel::Diagonal::Sigmas(Vector6);
    noiseModel::Diagonal::shared_ptr velocity_noise_model=noiseModel::Isotropic::Sigma(3,0.1);
    noiseModel::Diagonal::shared_ptr bias_noise_model=noiseModel::Isotropic::Sigma(6,0.001);
    NonlinearFactorGraph *graph = new NonlinearFactorGraph();//定义因子图
    graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
    graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
    graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));
```
## 6.7 使用传感器信息构建IMU的噪声模型
```cpp
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
    p->accelerometerCovariance=measured_acc_cov;//加速度计
    p->integrationCovariance=integration_error_cov;//积分
    p->gyroscopeCovariance=measured_omega_cov;//陀螺仪

    //预计分测量值
    p->biasAccCovariance=bias_acc_cov;
    p->biasAccOmegaInt=bias_acc_omega_int;
    p->biasOmegaCovariance=bias_omega_cov;

      imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);
```
## 6.8 保存上一次的imu积分值和结果
```cpp
    //保存上一次的imu积分值和结果
    NavState prev_state(prior_pose,prior_velocity);
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias=prior_imu_bias;//

    //记录总体误差
    double current_position_error = 0.0, current_orientation_error = 0.0;

    double output_time=0;
    double dt=0.005;    //积分时间
```
## 6.9 处理IMU数据
```cpp
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
```
## 6.10 处理GPS数据
### 6.10.1 读取GPS数据
```cpp
//Gps测量数据
            Eigen::Matrix<double,7,1> gps=Eigen::Matrix<double,7,1>::Zero();
            for(int i=0;i<6;i++){
                getline(file,value,',');
                gps(i)=atof(value.c_str());
            }
            getline(file, value, '\n');
            gps(6)=atof(value.c_str());
            correction_count++;
```
### 6.10.2 预积分测量值/构建预积分因子
```cpp
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
```
### 6.10.3 构建GPS因子
```cpp
        noiseModel::Diagonal::shared_ptr correction_noise=noiseModel::Isotropic::Sigma(3,1.0);
        GPSFactor gps_factor(X(correction_count),
                            Point3(gps(0),gps(1),gps(2)),//(N,E,D)
                            correction_noise);
        graph->add(gps_factor);
```

### 6.10.4 迭代更新求解IMU预测值
```cpp
        //迭代更新求解imu预测值
        prop_state=imu_preintegrated_->predict(prev_state,prev_bias);
        initial_values.insert(X(correction_count), prop_state.pose());
        initial_values.insert(V(correction_count), prop_state.v());
        initial_values.insert(B(correction_count), prev_bias);
```
## 6.11 求解
```cpp
        //求解
        LevenbergMarquardtOptimizer optimizer(*graph,initial_values);
        Values result=optimizer.optimize();
```
## 6.12 更新下一步预积分初值
```cpp
        //更新下一步预计分初始值
        //导航状态
        prev_state=NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
        //偏导数
        prev_bias=result.at<imuBias::ConstantBias>(B(correction_count));
        //更新预计分值
        imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
```
## 6.13 计算误差和可视化误差
### 6.13.1 位置误差
```cpp
        //计算角度误差和误差
        Vector3 gtsam_position=prev_state.pose().translation();
        //位置误差
        Vector3 position_error=gtsam_position-gps.head<3>();
        //误差的范数
        current_position_error=position_error.norm();//归一化
```
### 6.13.2 姿态误差
```cpp
        //姿态误差
        Quaternion gtsam_quat=prev_state.pose().rotation().toQuaternion();
        Quaternion gps_quat(gps(6),gps(3),gps(4),gps(5));
        Quaternion quat_error=gtsam_quat*gps_quat.inverse();
        quat_error.normalized();//归一化
        Vector3 euler_angle_error(quat_error.x()*2,quat_error.y()*2,quat_error.z()*2);//转换为欧拉角误差
        current_orientation_error=euler_angle_error.norm();
```
### 6.13.3 输出误差
```cpp

        //输出误差
        cout << "Position error:" << current_position_error << "\t " << "Angular error:" << current_orientation_error << "\n";
              fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
              gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
              gps(0), gps(1), gps(2),
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

        output_time += 1.0;
```
# 7 ISAM1模板案例
1. 头文件
```cpp
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
```
* 如[ISAM1_example1](./src/ISAM1_example1.cpp)

## 7.1 代码
```cpp
...
    //创建ISAM：周期性重新平滑线性化和排序
    int relinearizeInterval=3;//新添加多少个之后更新，
    NonlinearISAM isam(relinearizeInterval);

    //创建图模型
    NonlinearFactorGraph graph;
    Values initialEstimate;
...
        if(i==0){
...
        }
        else{
            isam.update(graph,initialEstimate);
            Values currentEsimate=isam.estimate();
...
            currentEsimate.print("Current estimate: ");
            graph.resize(0);
            initialEstimate.clear();
        }
```

# 8 ISAM2模板案例
## 8.1 案例一
1. 头文件
```cpp
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
```
* 如 [ISAM2_example1](./src/ISAM2_example1.cpp)
## 8.1.1   代码
```cpp

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
  ...

  for(size_t i=1;i<5;i++){
...
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
      Values currentEstimate=isam.calculateBestEstimate();//calculateBestEstimate 使用所有值进行回带
      currentEstimate.print("Current estimate:");
      boost::optional<Point3> pointEstimate=smartFactor->point(currentEstimate);
          if (pointEstimate) {
      cout << *pointEstimate << endl;
      }
      graph.resize(0);
      initialEstimate.clear();
  }
  return 0;

```
## 8.2 案例二
1. 头文件
```cpp
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
```
### 8.2.1 代码
```cpp
...
    //与ISAM1不同，ISAM1进行周期性批量处理，ISAM2进行部分平滑线性
    ISAM2Params parameters;
    parameters.relinearizeThreshold=0.01;   //重新线性化平滑值
    parameters.relinearizeSkip=1;
    ISAM2 isam(parameters);
    
    NonlinearFactorGraph graph;
    Values initialEstimate;

    for(size_t i=0;i<poses.size();i++){
...
        if(i==0){
            ...
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
```

