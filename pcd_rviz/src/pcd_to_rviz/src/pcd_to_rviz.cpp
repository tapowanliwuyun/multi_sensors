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
  ros::init (argc, argv, "pcd_to_rviz");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output1", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud_raw;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point;
  sensor_msgs::PointCloud2 output;

std::string::size_type sz;

  int num = 0;
  string str_zong, str;
  std::fstream inf;

  // 2. 接受文件
  // 2.1 
    if (argc != 3) {
        std::cout << "读取cpp文件内的pcd文件" << std::endl;
        pcl::io::loadPCDFile ("/home/bupo/bag_file/jcm/autoware-jcm.pcd", cloud_raw);//更换为自己的pcd文件路径
          for(int i = 0; i < cloud_raw.size();i++)
          {
              point.x = cloud_raw.at(i).x;
              point.y = cloud_raw.at(i).y;
              point.z = cloud_raw.at(i).z - 1.5;
              cloud.push_back(point);
          }
    }
    else{
      if (atol(argv[1] )== 1)
      {
          std::cout << "读取所给定路径的txt文件" << std::endl;
          
          inf.open(argv[2]);

          if(!inf.is_open())
          {
            std::cerr <<  "open acc file failed" << std::endl;
            return 0 ;
          }
          while(!inf.eof())
          {
            cout << num << endl;
          double x = 0,y = 0,z = 0;
              inf >> point.x >> point.y >> point.z ;
              num++;
              cout << point.x << endl;
              cloud.push_back(point);
          }

          if (inf.is_open())
            inf.close();
      }
      else if(atol(argv[1]) == 2)
      {
          std::cout << "读取所给定路径的pcd文件" << std::endl;
          pcl::io::loadPCDFile (argv[2], cloud_raw);
                    //Convert the cloud to ROS message
          for(int i = 0; i < cloud_raw.size();i++)
          {
              point.x = cloud_raw.at(i).x;
              point.y = cloud_raw.at(i).y;
              point.z = cloud_raw.at(i).z - 1.5;
              cloud.push_back(point);
          }
      }
      else
      {
          std::cout << "请正确的文件格式 pcd 或者 txt'" << std::endl;
          return 0 ;
      }
          std::cout << cloud.size() << std::endl;
    }

  // 3. 读取文件数据
  


  // 4. 转换点云格式，发布点云

  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "odom1";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer，这里时fix frame的名字
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    std::cout << "开始加载"<< std::endl;
    pcl_pub.publish(output);
    std::cout << "加载结束"<< std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


