
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <vector>

using namespace std;
using namespace gtsam;

// Shorthand for velocity and pose variables
using symbol_shorthand::V;
using symbol_shorthand::X;

const double kGravity = 9.81;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  auto params = PreintegrationParams::MakeSharedU(kGravity);// 设置重力分量（0，0，-g）
  params->setAccelerometerCovariance(I_3x3 * 0.1);//加速度计方差
  params->setGyroscopeCovariance(I_3x3 * 0.1);//陀螺仪方差
  params->setIntegrationCovariance(I_3x3 * 0.1);//积分方差
  params->setUse2ndOrderCoriolis(false);//是否使用二阶科氏
  params->setOmegaCoriolis(Vector3(0, 0, 0));//角速度科氏

  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));

  // Start with a camera on x-axis looking at origin
  double radius = 30;
  const Point3 up(0, 0, 1), target(0, 0, 0);
  const Point3 position(radius, 0, 0);
  const SimpleCamera camera = SimpleCamera::Lookat(position, target, up);//在这里面会根据三个向量之间的关系生成相机的位姿和位置
  const auto pose_0 = camera.pose();

  // Now, create a constant-twist scenario that makes the camera orbit the
  //创建一个恒定的旋转场景，使相机绕着原点旋转
  // origin
  double angular_velocity = M_PI,  // rad/sec
      delta_t = 1.0 / 18;          // makes for 10 degrees per step
  Vector3 angular_velocity_vector(0, -angular_velocity, 0);//角速度
  Vector3 linear_velocity_vector(radius * angular_velocity, 0, 0);//线速度
  auto scenario = ConstantTwistScenario(angular_velocity_vector,//这里就是相当于之前生成的相机位姿绕着原点逆时针旋转，pose_0为初始位姿
                                        linear_velocity_vector, pose_0);

  // Create a factor graph  创建因子图
  NonlinearFactorGraph newgraph;

  // Create (incremental) ISAM2 solver 创建ISAM2求解器
  ISAM2 isam;

  // Create the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth 故意使初始值远离真实值
  Values initialEstimate, totalEstimate, result;

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 0.1 rad std on roll, pitch, yaw, 30cm std on x,y,z.
  auto noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
  newgraph.push_back(PriorFactor<Pose3>(X(0), pose_0, noise));//对x0添加一个先验值  即初始位置

  // Add imu priors 添加IMU bias 的先验
  Key biasKey = Symbol('b', 0);
  auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
  PriorFactor<imuBias::ConstantBias> biasprior(biasKey, imuBias::ConstantBias(),
                                               biasnoise);
  newgraph.push_back(biasprior);
  initialEstimate.insert(biasKey, imuBias::ConstantBias());//添加b0到初始估计值  initialstimate
  auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  Vector n_velocity(3);
  n_velocity << 0, angular_velocity * radius, 0;
  PriorFactor<Vector> velprior(V(0), n_velocity, velnoise);
  newgraph.push_back(velprior);//对V0添加一个先验值 初始速度

  initialEstimate.insert(V(0), n_velocity);

  // IMU preintegrator
  PreintegratedImuMeasurements accum(params);

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 0; i < 18; ++i) {
    double t = i * delta_t;
    if (i == 0) {  // First time add two poses  当i==0时  添加x0和x1的先验
      auto pose_1 = scenario.pose(delta_t);
      initialEstimate.insert(X(0), pose_0.compose(delta));  // 这里的先验都加上了delta的误差量
      initialEstimate.insert(X(1), pose_1.compose(delta));
    } else if (i >= 2) {  // Add more poses as necessary  当i>=2时 添加更多的初始值
      auto pose_i = scenario.pose(t);
      initialEstimate.insert(X(i), pose_i.compose(delta));
    }

    if (i > 0) {
      // Add Bias variables periodically   周期性的添加bias
      if (i % 5 == 0) {
        biasKey++;
        Symbol b1 = biasKey - 1;
        Symbol b2 = biasKey;
        Vector6 covvec;
        covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        auto cov = noiseModel::Diagonal::Variances(covvec);
        auto f = boost::make_shared<BetweenFactor<imuBias::ConstantBias> >(
            b1, b2, imuBias::ConstantBias(), cov);
        newgraph.add(f);
        initialEstimate.insert(biasKey, imuBias::ConstantBias());
      }
      // Predict acceleration and gyro measurements in (actual) body frame
      // 把加速度转到body系下 （减去重力分量）
      Vector3 measuredAcc = scenario.acceleration_b(t) -
                            scenario.rotation(t).transpose() * params->n_gravity;
      Vector3 measuredOmega = scenario.omega_b(t);
      // 预积分量
      accum.integrateMeasurement(measuredAcc, measuredOmega, delta_t);

      // Add Imu Factor  添加IMU因子
      ImuFactor imufac(X(i - 1), V(i - 1), X(i), V(i), biasKey, accum);
      newgraph.add(imufac);

      // insert new velocity, which is wrong 加入新的速度值（错误的）
      initialEstimate.insert(V(i), n_velocity);
      accum.resetIntegration();//把积分量reset掉，便于下一次积分使用
    }

    // Incremental solution 求解
    isam.update(newgraph, initialEstimate);
    result = isam.calculateEstimate();
    newgraph = NonlinearFactorGraph();
    initialEstimate.clear();
  }
  GTSAM_PRINT(result);
  return 0;
}
/* ************************************************************************* */
