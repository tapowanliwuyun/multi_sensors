一、编译
	cd ~/catkin_ws/src/
	git clone https://github.com/laboshinl/loam_velodyne.git
	cd ~/catkin_ws
	catkin_make -DCMAKE_BUILD_TYPE=Release 



二、运行
1. launch文件
	source ~/catkin_ws/devel/setup.bash
	roslaunch loam_velodyne loam_velodyne.launch
2. bag包
	运行bag包之前，要检查launch文件中的配置和所要播放的bag包之间的关系
2.1 播放nsh_indoor_outdoor.bag
  <node pkg="loam_velodyne" type="multiScanRegistration" name="multiScanRegistration" output="screen">
    <param name="lidar" value="VLP-16" /> <!-- options: VLP-16  HDL-32  HDL-64E -->
    <param name="scanPeriod" value="$(arg scanPeriod)" />

    <remap from="/multi_scan_points" to="/velodyne_points" />
  </node>

	对于上述配置执行命令：
	rosbag play ~/bag_file/multi_sensor/nsh_indoor_outdoor.bag

	结果：里程计的发布坐标系不太对劲
2.2 播放

  <node pkg="loam_velodyne" type="multiScanRegistration" name="multiScanRegistration" output="screen">
    <param name="lidar" value="HDL-64E" /> <!-- options: VLP-16  HDL-32  HDL-64E -->
    <param name="scanPeriod" value="$(arg scanPeriod)" />

    <remap from="/multi_scan_points" to="/kitti/velo/pointcloud" />
  </node>
	对于上述配置执行命令：
	rosbag play ~/bag_file/multi_sensor/kitti_lidar_only_2011_10_03_drive_0027_synced.bag
	
	结果：飘的非常离谱
	
三、报错：
	1、报错一
错误提示：
	[multiScanRegistration-2] process has died [pid 31250, exit code -11, cmd /home/bupo/my_study/multi_sensors/loam_velodyne/devel/lib/loam_velodyne/multiScanRegistration /multi_scan_points:=/velodyne_points __name:=multiScanRegistration __log:=/home/bupo/.ros/log/57f2cf2c-33d3-11ed-b5a5-488ad2f54045/multiScanRegistration-2.log].
log file: /home/bupo/.ros/log/57f2cf2c-33d3-11ed-b5a5-488ad2f54045/multiScanRegistration-2*.log
解决：参考 https://blog.csdn.net/santututu39/article/details/121478538
	只需要找到路径src/loam_velodyne下的CMakeLists.txt，将里面第35行（add_definitions( -march=native )）注释掉即可！！
