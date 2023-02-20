1. 创建一个简单因子图	
./simple_factor_graph

1.1 边的类继承与NoiseModelFactorX就说明其是几元边



2. 因子图进行最小乘优化
./LSM_factor_graph 

2.1 这里使用自己定义的一种一元边来表示GPS约束

3. 添加一个回环约束
./loopclosure_factor_graph
./sphere_factor_graph

3.1 使用生成的.g2o文件作为输入来进行优化

4. symbol
4.1 
./symbol_factor_graph 

4.2 另一种添加约束的方式
express
./express_factor_graph



5. 使用相机矫正，简单的SFM
5.1 
./camera_correct_factor_graph

自己的构建不同时刻的位姿和路标
该程序优化的变量包括：K的四个参数、8个路标点、8个位姿

5.2 
./sfm_factor_graph
如何查看自己构造的因子图：graph.print("Graph \n");

