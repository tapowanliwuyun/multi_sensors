#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point2.h>

using namespace std;
using namespace gtsam;

typedef Point2 LinearMeasurement;
// F:状态转移矩阵
// B:输入矩阵
// Q:高斯白噪声 在由的模型中Q=G*w*G^T 一般为对角阵
//  控制方程 
//  X_k=F*x_k_1+B*u_k+Q
//  Z_k=H*X_t+V
// F = [1 0 ; 0 1], B = [1 0 ; 0 1]
// u = [1 ; 0]   , Q = [0.1 0 ; 0 0.1]
int main(int argc, char const *argv[])
{
    Point2 x_initial(0,0);
    SharedDiagonal P_initial=noiseModel::Diagonal::Sigmas(Vector2(0.1,0.1));
    //step1:x初始值
    Symbol x0('x',0);
    //step2:构建ekf
    ExtendedKalmanFilter<Point2> ekf(x0,x_initial,P_initial);
    //u:输入值
    Vector u=Vector2(1.0,0.0);
    SharedDiagonal Q=noiseModel::Diagonal::Sigmas(Vector2(0.1,0.1),true);
    //step3:创建x1
    Symbol x1('x',1);
    //输入的预测值
    Point2 difference(1,0);
    //ekf:预测
    BetweenFactor<Point2> factor1(x0,x1,difference,Q);
    Point2 x1_predict=ekf.predict(factor1);
    traits<Point2>::Print(x1_predict, "X1 预测");
    SharedDiagonal R=noiseModel::Diagonal::Sigmas(Vector2(0.25,0.25),true);
    //ekf:更新
    Point2 z1(1,0);
    PriorFactor<Point2> factor2(x1,z1,R);
    Point2 x1_update=ekf.update(factor2);
    traits<Point2>::Print(x1_update,"X1 更新");

      // Predict
    Symbol x2('x',2);
    difference = Point2(1,0);
    BetweenFactor<Point2> factor3(x1, x2, difference, Q);
    Point2 x2_predict = ekf.predict(factor1);
    traits<Point2>::Print(x2_predict, "X2 预测");
    // Update
    Point2 z2(2.0, 0.0);
    PriorFactor<Point2> factor4(x2, z2, R);
    Point2 x2_update = ekf.update(factor4);
    traits<Point2>::Print(x2_update, "X2 更新");
     // Do the same thing one more time...
    // Predict
    Symbol x3('x',3);
    difference = Point2(1,0);
    BetweenFactor<Point2> factor5(x2, x3, difference, Q);
    Point2 x3_predict = ekf.predict(factor5);
    traits<Point2>::Print(x3_predict, "X3 预测");

  // Update
    Point2 z3(3.0, 0.0);
    PriorFactor<Point2> factor6(x3, z3, R);
    Point2 x3_update = ekf.update(factor6);
    traits<Point2>::Print(x3_update, "X3 更新");
    ekf.print("ekf all \n");
    return 0;
}




