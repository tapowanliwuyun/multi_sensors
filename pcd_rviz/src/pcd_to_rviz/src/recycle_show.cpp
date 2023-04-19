//pcd_to_rviz.cpp
#include<iostream>
#include<fstream>
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include<string>

#include<sstream>
#include<cstdlib>
using namespace std;
//which contains the required definitions to load and store point clouds to PCD and other file formats.
 
main (int argc, char **argv)
{
  // 1. 初始化 
  ros::init (argc, argv, "recycle_show");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output1", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud_raw;

  pcl::PointXYZ point;
  sensor_msgs::PointCloud2 output;

std::string::size_type sz;


  string str_zong, str;
  std::fstream inf;
  string txtpath;
  // 2. 接受文件
  // 2.1 
    if (argc != 2) {
        std::cout << "请正确输入完整指令！！" << std::endl;
        txtpath = "../pcd_rviz/src/pcd_to_rviz/data/SavedPoints.txt";
        std::cout << "未输入地址，采用默认文件地址："<< txtpath << std::endl;
        return 0;
    }
    else{
          std::cout << "读取所给定路径的txt文件----" << std::endl;
          txtpath = argv[1];
          std::cout << "文件路径为 = " << txtpath << std::endl;
    }

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int num = 0;
    // 3. 读取文件数据
    inf.open(txtpath);

    if(!inf.is_open())
    {
      std::cerr <<  "open acc file failed" << std::endl;
      return 0 ;
    }
    while(!inf.eof())
    {
      //cout << num << endl;
      double x = 0,y = 0,z = 0;
      inf >> point.x >> point.y >> point.z ;
      //std::cout << "\npoint.x = " << point.x <<"\npoint.y = "<< point.y <<"\npoint.z  = "<< point.z << std::endl;
      num++;
      cloud.push_back(point);
    }
    std::cout << "本次遍历" << num <<"次，有效点云为"<<cloud.size() << "个" << std::endl;

    if (inf.is_open())
      inf.close();
    if(cloud.size() <1000)
    {
      continue;
    }
    // 4. 转换点云格式，发布点云

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom1";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer，这里时fix frame的名字

    std::cout << "开始加载"<< std::endl;
    pcl_pub.publish(output);
    std::cout << "加载结束"<< std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


