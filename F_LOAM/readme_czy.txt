一、编译
	cd ~/catkin_ws/src
	git clone https://github.com/wh200720041/floam.git
	cd ..
	catkin_make

二、运行
	source ~/catkin_ws/devel/setup.bash
	如果想要建图
	roslaunch floam floam_mapping.launch
三、报错
	1、运行报错
ERROR: cannot launch node of type [hector_trajectory_server/hector_trajectory_server]: hector_trajectory_server
ROS path [0]=/opt/ros/melodic/share/ros
ROS path [1]=/home/bupo/my_study/multi_sensors/F_LOAM/src
ROS path [2]=/opt/ros/melodic/share
ERROR: cannot launch node of type [hector_trajectory_server/hector_trajectory_server]: hector_trajectory_server
ROS path [0]=/opt/ros/melodic/share/ros
ROS path [1]=/home/bupo/my_study/multi_sensors/F_LOAM/src
ROS path [2]=/opt/ros/melodic/share
[FATAL] [1663206323.243588961]: Error opening file: /home/bupo/Downloads/2011_09_30_0018.bag
[rosbag_play-2] process has died [pid 6564, exit code 1, cmd /opt/ros/melodic/lib/rosbag/play --clock /home/bupo/Downloads/2011_09_30_0018.bag __name:=rosbag_play __log:=/home/bupo/.ros/log/10176a76-3498-11ed-b5a5-488ad2f54045/rosbag_play-2.log].
log file: /home/bupo/.ros/log/10176a76-3498-11ed-b5a5-488ad2f54045/rosbag_play-2*.log
	解决方法
	首先
		修改对应launch文件里面的bag包路径
	然后
		安装所需要的库
			sudo apt-get install ros-melodic-hector-trajectory-server
