1、编译
	在src文件下：
	catkin_make

2、运行
	启动roscore

	打开rviz，位于src下
		rviz -d rviz.rviz

	在文件主目录下
		source devel/setup.bash
	打开cpp文件内路径下的pcd文件		
		rosrun pcd_to_rviz pcd_to_rviz
	打开指定绝对路径的pcd文件
		rosrun pcd_to_rviz pcd_to_rviz /home/bupo/autoware-jcm.pcd

3、安装依赖
	sudo apt-get install ros-melodic-pcl-conversions
	sudo apt-get install ros-melodic-pcl-ros


