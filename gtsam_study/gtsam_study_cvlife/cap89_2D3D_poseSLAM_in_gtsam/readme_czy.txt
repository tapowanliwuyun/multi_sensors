1. 编译
cd build
cmake ..
make


2. 运行
2.1 
./Pose2SLAMExample
    使用 Prioractor、 BetweenFactor 因子 、使用高斯牛顿法 、emplace_shared方法添加

2.2
./Pose2SLAMExample_graphviz
    使用 Prioractor、 BetweenFactor 因子 、使用LM法 、emplace_shared方法添加

2.3
./Pose2SLAMwSPCG
    使用 Prioractor、 BetweenFactor 因子 、使用LM法并配置LevenbergMarquardtParams参数 、emplace_shared方法添加

2.4
./Pose2SLAMExampleExpressions
    使用 ExpressionFactor 因子、 使用高斯牛顿法 、 addExpressionFactor 方法添加

2.5
./Pose2SLAMExample_g2o
或者
./Pose2SLAMExample_g2o ../src/noisyToyGragh.txt
    读取.g2o文件里面的 顶点 和 边 来进行优化，使用NonlinearFactorGraph 和高斯牛顿法求解。

2.6
./Pose2SLAMExample_lago
或者
./Pose2SLAMExample_lago ../src/noisyToyGragh.txt

读取 g2x2o 文件数据后,同样构建 NonliearFactorGraph,但是使用线性逼近(Linear Approximation for Graph Optimization)是同时定位与地图构建(的方法求解。

2.7 
./Pose2SLAMExample_graph
读取 100 个 2x2D 位姿的数据,构建 NonlinearFactorGraph 并使用 LM 法求解。

2.8
./Pose3SLAMExample_g2o ../src/pose3example.txt
Gauss-Newton 优化器比 LM 优化器更快但更不稳定。Gauss-Newton 只在当前估
计值是优化目标函数的局部极小值点时才能保证快速收敛。而当当前估计值不在局部
极小值点附近时,Gauss-Newton 可能会发散。
相比之下,LM 优化器的性能更加稳定,因为它使用一种自适应的方式来控制步长
大小,从而可以在不同位置的优化问题中获得更好的收敛性能。
因此,如果计算速度是最重要的因素,可以选择 Gauss-Newton 优化器;如果稳
定性更重要,可以选择 LM 优化器。当然,具体选择哪种优化器还取决于问题的特性
和优化的目标。

2.9
./Pose3SLAMExample_initializePose3Gradient ../src/pose3example.txt

黎曼梯度法可以用于处理优化问题,其中因子类型和变量类型可以是几何、李群、
李代数或其他非欧几里德类型。在 gtsam 中,可以使用带有 Riemannian(黎曼)是同时定位与地图构建(后 缀
的因子类型和变量类型,例如 Rot3 和 Rot3::Jacobian。这些类型支持黎曼梯度法,
并且在 LevenbergMarquardtOptimizer、GaussNewtonOptimizer 和
DoglegOptimizer 中均可用。

2.10 
./Pose3SLAMExample_initializePose3Chordal ../src/pose3example.txt
GTSAM 库中提供了弦松弛算法(Chordal Relaxation)是同时定位与地图构建(来处理大规模稀疏矩阵,
该算法基于稀疏图的特殊结构,可以在不牺牲优化精度的情况下大幅降低计算成本。

接下来,可以使用 gtsam 提供的 CholeskyFactor 类和 NonlinearFactorGraph 类定
义优化问题的因子图。CholeskyFactor 类可以自动检测稀疏性,根据稀疏性采用不同
的算法来计算分解因子。

在优化完成后,可以使用 Marginals 类计算各个变量的边缘协方差矩阵。

2.11
./Pose3SLAMExampleExpressions_BearingRangeWithTransform

该 cpp 文件使用 ExpressionFactorGraph 表达式因子图,创建机器人的运
动轨迹和一些点的位置,同时指定一些噪声模型,添加包括初始姿态先验、每个位姿
的 BearingRange 因子、位姿之间的因子。

