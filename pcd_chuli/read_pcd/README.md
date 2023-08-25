# read_pcd
https://blog.csdn.net/weixin_42174523/article/details/122217637
#### 介绍
ROS环境下采用PCL点云库对PCD格式三维点云进行加载、滤波、旋转和平移等处理

#### 安装环境
操作系统: Ubuntu 18.04
ROS版本: ROS Melodic

说明: 作者我仅仅在上述环境下进行了验证调试，其它版本应该也是可用的。

#### 安装教程
1.  安装好ubuntu系统和ROS环境
2.  打开终端，并在终端命令行输入以下指令:
3.  git clone https://gitee.com/wccworld/read_pcd.git
4.  cd read_pcd/catkin_ws
5.  catkin_make
6.  source devel/setup.bash
7.  roslaunch read_pcd read_pcd.launch

#### 使用说明
找到功能包里的read_pcd.launch文件，只需对相应参数进行修改，便可实现PCL点云的滤波、体素降采样、移除离群点、旋转以及平移等操作，具体参数代表的内容如下所示。

<launch>
  <param name="output_frame_id" value = "map" />                   # 设置frame_id
  <param name="pointCloud_pubTopic" value = "/handle_point" />     # 设置处理后点云的输出话题名
  <param name="pcd_doc_path" value = "/home/bupo/my_study/read_pcd/my_data/rgb_pt.pcd" />       # 设置所要加载的PCD文件路径，此处需要用自己的PCD文件路径替换此路径
  <param name="output_pcd_path" value = "/home/bupo/my_study/read_pcd/my_data/rgb_pt_xiugai.pcd" />   # 设置处理完之后点云输出的PCD文件路径，此处需根据自己的路径进行修改

  <param name="pass_x_min" value = "-100.0" />      # 设置直通滤波器进行X轴滤波时X的最小值
  <param name="pass_x_max" value = "100.0" />       # 设置直通滤波器进行X轴滤波时X的最大值
  <param name="pass_y_min" value = "-100.0" />      # 设置直通滤波器进行Y轴滤波时Y的最小值
  <param name="pass_y_max" value = "100.0" />       # 设置直通滤波器进行Y轴滤波时Y的最大值
  <param name="pass_z_min" value = "-1.0" />        # 设置直通滤波器进行Z轴滤波时Z的最小值
  <param name="pass_z_max" value = "30.0" />        # 设置直通滤波器进行Z轴滤波时Z的最大值
  <param name="voxel_size" value = "0.1" />         # 设置滤波时创建的体素体积值，单位(M)
  <param name="sor_nearby_number" value = "10" />   # 设置在进行统计时考虑查询点临近点数
  <param name="sor_thresh_value" value = "5.0" />   # 设置判断是否为离群点的阀值
  <param name="x_rotate_value" value = "0.0" />     # 设置整体点云绕X轴旋转的角度值
  <param name="y_rotate_value" value = "0.0" />     # 设置整体点云绕y轴旋转的角度值
  <param name="z_rotate_value" value = "0.0" />     # 设置整体点云绕z轴旋转的角度值
  <param name="x_trans_value" value = "0.0" />      # 设置整体点云绕x轴平移的距离值
  <param name="y_trans_value" value = "20.0" />     # 设置整体点云绕y轴平移的距离值
  <param name="z_trans_value" value = "0.0" />      # 设置整体点云绕z轴平移的距离值

  <node pkg="read_pcd" name="read_pcd" type="read_pcd" output="screen" />
  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find read_pcd)/rviz/read_pcd.rviz" />
</launch>


