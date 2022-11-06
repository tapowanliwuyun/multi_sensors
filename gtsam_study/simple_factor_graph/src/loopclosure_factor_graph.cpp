#include <gtsam/slam/lago.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <fstream>

using namespace std;
using namespace gtsam;

int main(int argc,char** argv){
    //read data
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
    noiseModel::Diagonal::shared_ptr priorModel=noiseModel::Diagonal::Variances(gtsam::Vector3(1e-6,1e-6,1e-8));
    graphWithPrior.add(PriorFactor<Pose2>(0,Pose2(),priorModel));//添加起始点
    std::cout << "Computing LAGO estimate ..." << std::endl;
    //使用lago 快速初始化因子图
    Values estimate=lago::initialize(graphWithPrior);//返回因子图中的值
    std::cout << "done" << std::endl;
    estimate.print("estimateLago");
    string outputfile="../src/loop_closure_optimized.g2o";
    std::cout << "Writing results to file: " << outputfile << std::endl;
    writeG2o(*graph,estimate,outputfile);
    //graph:图结构
    //estimate：估计值
    //outputfile：保存路径
    std::cout << "done! " << std::endl;
}

