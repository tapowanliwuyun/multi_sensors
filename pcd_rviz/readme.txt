0、功能
1）读取pcd点云文件发布成pointcloud2话题，以便于在 rviz中展示
2）读取txt文件中的点云发布成pointcloud2话题，以便于在 rviz中展示
		其中，txt中存储的格式为：（x空格y空格z，三个数据为一行）
		x y z

1、编译
	在src文件下：
	catkin_make

2、运行
	启动roscore

	打开rviz，位于src下
		rviz -d rviz.rviz
	在文件主目录下
		source devel/setup.bash
		1）使用pcd文件
	打开cpp文件内路径下的pcd文件		
		rosrun pcd_to_rviz pcd_to_rviz
	打开指定绝对路径的pcd文件
		rosrun pcd_to_rviz pcd_to_rviz 2 /home/bupo/autoware-jcm.pcd
		2）使用txt文件
		rosrun pcd_to_rviz pcd_to_rviz 1 ../pcd_rviz/src/pcd_to_rviz/data/SavedPoints.txt
3、安装依赖
	sudo apt-get install ros-melodic-pcl-conversions
	sudo apt-get install ros-melodic-pcl-ros


