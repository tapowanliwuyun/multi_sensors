
#include <iostream>//
#include  <gtsam/slam/InitializePose3.h>
#include  <gtsam/slam/dataset.h>
#include  <gtsam/slam/BetweenFactor.h>
#include  <gtsam/slam/PriorFactor.h>
#include  <fstream>
#include  <string>


using namespace std;
using namespace gtsam;
int main(int argc,char** argv){
    string g2ofile="../src/sphere.g2o";
    NonlinearFactorGraph::shared_ptr graph;
    Values::shared_ptr initial;
    bool is3D=true;
    boost::tie(graph,initial)=readG2o(g2ofile,is3D);
    NonlinearFactorGraph graphPrior=*graph;

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
    std::cout<<"Initialzing Pose3"<<std::endl;
    Values initialzation=InitializePose3::initialize(graphPrior);//初始化
    std::cout<<"done"<<endl;
    initialzation.print("initialzatoin");
    string output="../src/correctsphere.g2o";
    std::cout<<"wirte results to file:"<<output<<std::endl;
    writeG2o(*graph,initialzation,output);
    std::cout<<"done"<<std::endl;
    return 0;
}




