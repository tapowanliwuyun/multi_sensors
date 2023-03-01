#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/Vector.h>//
#include <stdlib.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>


using namespace std;
using namespace gtsam;

//自定义因子
class CurveFitFactor : public gtsam::NoiseModelFactor1<Vector2>
{
private:
  double _mx, _my;

public:
  CurveFitFactor(Key j, double x, double y, const SharedNoiseModel &model) : NoiseModelFactor1<Vector2>(model, j), _mx(x), _my(y) {}
  virtual ~CurveFitFactor() {}

  Vector evaluateError(const Vector2 &q, boost::optional<Matrix&> H=boost::none) const 
  {
    if (H)
      (*H) = (Matrix(1, 2) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1])).finished();
    return (Vector(1) << _my - exp(q[0] * _mx + q[1])).finished();
    // if (H)
    //   (*H) = (Matrix(2, 2) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1]), 0, 0).finished();
    // return (Vector(2) << _my - exp(q[0] * _mx + q[1]), 0).finished();
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new CurveFitFactor(*this)));
  }
};
int main(int argc, char **argv)
{
    using symbol_shorthand::X;
    NonlinearFactorGraph graph;

    srand(time(nullptr)); //设置随机数种子

    // f = exp(ax + b)
    double a = 0.03;
    double b = 0.5;

    auto CurveNoise = noiseModel::Diagonal::Sigmas(Vector1(0.1)); // 10cm std on x,y
    for (size_t i = 0; i < 100; i++)
    {
        double randoxNoise = (rand() % (1000)) * 0.0001;
        graph.emplace_shared<CurveFitFactor>(X(0), 0.1 * i, exp(a * 0.1 * i + b) + randoxNoise, CurveNoise);
        // cout << "i: " << i << " exp(a * i + b): " << exp(a * i + b) << " randoxNoise: " << randoxNoise<< endl;
    }

    Values initialEstimate;
    initialEstimate.insert(X(0), Vector2(0.0, 0.0));
    initialEstimate.print("\nInitial Estimate:\n"); // print

    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("LM Final Result:\n");

    ISAM2 isam;
    isam.update(graph, initialEstimate);
    auto result1 = isam.calculateEstimate();
    graph = NonlinearFactorGraph();
    result1.print("ISAM2 Final Result:\n");

}