#include <gtsam/nonlinear/DoglegOptimizer.h>// dogleg
#include <gtsam/nonlinear/GaussNewtonOptimizer.h> // GN
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>// symbol_shorthand

#include <random>

// y = ax^2 + bx
double funct(const gtsam::Vector2& param, const double x) {
    return (param(0)*x*x + param(1)*x);
}

// NoiseModelFactor1表示一元边, NoiseModelFactor2表示2元边,一次类推一直到6
class curvfitFactor : public gtsam::NoiseModelFactor1<gtsam::Vector2> {
    public:
        curvfitFactor(gtsam::Key key, const gtsam::SharedNoiseModel& noise, double x, double y)
        : gtsam::NoiseModelFactor1<gtsam::Vector2>(noise, key), x_(x), y_(y){
        }

        gtsam::Vector evaluateError(const gtsam::Vector2& param, boost::optional<gtsam::Matrix&> H = boost::none) const {
            auto val = funct(param, x_);
            if (H) {
                gtsam::Matrix Jac = gtsam::Matrix::Zero(1, 2);
                Jac << x_ * x_, x_;
                (*H) = Jac;
            }
            return gtsam::Vector1(val - y_);
        }

    private:
        double x_, y_;
};

int main() {
    using gtsam::symbol_shorthand::X;// symbol_shorthand仅用于表示状态变量(char + num)
    const double sig = 0.05;
    std::random_device rd;// 非确定性种子
    std::default_random_engine generator(rd());// 构建随即生成器
    std::normal_distribution<double> noise(0, sig);// 生成服从正态分布的噪声(均值,方差)
    gtsam::NonlinearFactorGraph graph;
    const gtsam::Vector2 para(2, 3);// 曲线参数a b

    for (int i = 0; i < 100; ++i) {
        // 构造观测值mx my
        double mx = i;
        auto my = funct(para, mx) + noise(generator);

        // Isotropic: 各项同性噪声模型,相当于在协方差对角线作放缩
        // 对比ceres,无需设置噪声模型,但构建残差时可以设置信息矩阵
        auto noiseM = gtsam::noiseModel::Isotropic::Sigma(1, sig);
        // 添加因子: 曲线拟合问题是一堆观测,一个状态量,因此只需要X(0)
        graph.emplace_shared<curvfitFactor>(X(0), noiseM, mx, my);
    }

    // 设置初始值
    gtsam::Values intial;
    intial.insert<gtsam::Vector2>(X(0), gtsam::Vector2(1.5, 2.2));

    // 选择优化方法
    // gtsam::GaussNewtonOptimizer opt(graph, intial);
    gtsam::LevenbergMarquardtOptimizer opt(graph, intial);
    // gtsam::DoglegOptimizer opt(graph, intial);
    std::cout << "\ninitial error=" << graph.error(intial) << std::endl;
    auto res = opt.optimize();
    std::cout << "\nfinal error=" << graph.error(res) << std::endl;
    gtsam::Vector2 matX0 = res.at<gtsam::Vector2>(X(0));
    std::cout << "a = " << matX0[0] << ", b = " << matX0[1] << "\n";

    return 0;
}
