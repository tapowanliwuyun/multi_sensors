1. 编译
	cd ~/LeGO_LOAM/src
	git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
	cd ..
	catkin_make

2. 运行
	source devel/setup.bash
播放launch文件
	roslaunch lego_loam run.launch
播放现有的bag包
	rosbag play *.bag --clock --topic	
	如：rosbag play ~/bag_file/multi_sensor/LeGO_LOAM/2018-05-18-14-49-12_0.bag --clock 
	    rosbag play ~/bag_file/multi_sensor/LeGO_LOAM/2018-05-18-14-49-12_0.bag --clock --topic /velodyne_points /imu/data
注意：虽然 /imu/data 是可选的，但如果提供它可以大大提高估计精度。


