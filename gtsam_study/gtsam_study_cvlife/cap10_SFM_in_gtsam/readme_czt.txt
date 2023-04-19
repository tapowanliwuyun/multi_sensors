1. 编译
cd build
cmake ..
make


2. 运行
2.1 
./SFMExample

2.2
./SFMExampleExpressions

2.3
./SFMExample_SmartFactor

2.4 
./SFMExample_SmartFactorPCG

    2.1-2.4 都是使用 SFMdata.h 创建的数据

2.5
./SFMExample_bal

2.6
./SFMExample_bal_COLAMD_METIS

2.7
./SFMExample_bal_COLAMD_METIS

2.8
./SFMExampleExpressions_bal

矩阵排序算法:

METIS 算法(Multilevel Partitioning of Irregular Networks)通过将图形分解为小的连
通子图来工作,然后在子图中应用递归二分划分。 METIS 算法的主要思想是将矩阵分
解为多个子矩阵,这些子矩阵可以使用较少的存储器和更快的算法进行处理。

COLAMD 算法(Column Approximate Minimum Degree)则使用一种基于对称因式分
解的技术,通过对矩阵的列进行排序,使其具有更好的稀疏性。 COLAMD 算法的主
要目的是减少高斯消元算法的计算时间和内存使用,从而提高矩阵求解的效率。
