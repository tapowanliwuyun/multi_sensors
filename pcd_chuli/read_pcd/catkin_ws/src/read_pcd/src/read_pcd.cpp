#include<ros/ros.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/filters/passthrough.h>  
#include<pcl/filters/voxel_grid.h> 
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/statistical_outlier_removal.h>

#include<sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointType;   //可自己修改点云格式

ros::Publisher read_pcd_pub;
ros::Publisher pointCloud_pub;

std::string pcd_doc_path;
std::string output_frame_id;
std::string pointCloud_pubTopic;
std::string output_pcd_path;

double pass_x_min, pass_x_max, pass_y_min, pass_y_max, pass_z_min, pass_z_max;  
double voxel_size;
int sor_nearby_number;
double sor_thresh_value;
double x_rotate_value, y_rotate_value, z_rotate_value;
double x_trans_value, y_trans_value, z_trans_value;


//角度制转弧度制
double rad(double d)
{
	return d * 3.1415926 / 180.0;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"read_pcd");
    ros::NodeHandle nh;

/********************************参数初始化**********************************************/
    nh.param<double>("pass_x_min",pass_x_min,1.0);
    nh.param<double>("pass_x_max",pass_x_max,2.0);
    nh.param<double>("pass_y_min",pass_y_min,1.0);
    nh.param<double>("pass_y_max",pass_y_max,2.0);
    nh.param<double>("pass_z_min",pass_z_min,1.0);
    nh.param<double>("pass_z_max",pass_z_max,2.0);
    nh.param<double>("voxel_size",voxel_size,0.01);
    nh.param<int>("sor_nearby_number",sor_nearby_number,30);
    nh.param<double>("sor_thresh_value",sor_thresh_value,1.0);
    nh.param<double>("x_rotate_value",x_rotate_value,0.0);
    nh.param<double>("y_rotate_value",y_rotate_value,0.0);
    nh.param<double>("z_rotate_value",z_rotate_value,0.0);
    nh.param<double>("x_trans_value",x_trans_value,0.0);
    nh.param<double>("y_trans_value",y_trans_value,0.0);
    nh.param<double>("z_trans_value",z_trans_value,0.0);

    nh.param<std::string>("output_frame_id", output_frame_id, "map");
    nh.param<std::string>("pcd_doc_path", pcd_doc_path, "./read_pcd/pcd/test.pcd");
    nh.param<std::string>("pointCloud_pubTopic", pointCloud_pubTopic, "/handle_point");
    nh.param<std::string>("output_pcd_path", output_pcd_path, "./read_pcd/pcd/output.pcd");
/***************************************************************************************/

    read_pcd_pub = nh.advertise<sensor_msgs::PointCloud2> ("/input_cloud",10);          //PCD原始点云发布
    pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2> (pointCloud_pubTopic,10); //PCD处理之后点云发布

/********************************PCD点云获取**********************************************/
    pcl::PointCloud<PointType>::Ptr pcd_cloud_in (new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType> (pcd_doc_path, *pcd_cloud_in) == -1)
    {
    PCL_ERROR ("Couldn't read file: %s \n", pcd_doc_path.c_str());
    return (-1);
    }

    sensor_msgs::PointCloud2 input_cloud;
    pcl::toROSMsg(*pcd_cloud_in,input_cloud);
    input_cloud.header.frame_id = output_frame_id;
    // read_pcd_pub.publish(input_cloud);
/****************************************************************************************/

/********************************点云处理*************************************************/
    pcl::PointCloud<PointType>::Ptr handle_cloud (new pcl::PointCloud<PointType>);
    /*
    //  直通滤波器 X 轴滤波 
    pcl::PointCloud<PointType>::Ptr filter_x (new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> ptx;
    ptx.setInputCloud(pcd_cloud_in);                 //输入点云
    ptx.setFilterFieldName("x");                     //对x轴进行操作
    ptx.setFilterLimits(pass_x_min, pass_x_max);     //设置直通滤波器操作范围
    // ptx.setFilterLimitsNegative(true);              //设置保留范围内，还是过滤掉范围内
    ptx.filter(*filter_x);                           //执行滤波，过滤结果保存在filter_x

    //  直通滤波器 Y 轴滤波 
    pcl::PointCloud<PointType>::Ptr filter_y (new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> pty;
    pty.setInputCloud(filter_x);                     //输入点云
    pty.setFilterFieldName("y");                     //对y轴进行操作
    pty.setFilterLimits(pass_y_min, pass_y_max);     //设置直通滤波器操作范围
    // pty.setFilterLimitsNegative(true);              //设置保留范围内，还是过滤掉范围内
    pty.filter(*filter_y);                           //执行滤波，过滤结果保存在filter_y

    //  直通滤波器 Z 轴滤波 
    pcl::PointCloud<PointType>::Ptr filter_z (new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> ptz;
    ptz.setInputCloud(filter_y);                     //输入点云
    ptz.setFilterFieldName("z");                     //对z轴进行操作
    ptz.setFilterLimits(pass_z_min, pass_z_max);     //设置直通滤波器操作范围
    // ptz.setFilterLimitsNegative(true);              //设置保留范围内，还是过滤掉范围内
    ptz.filter(*filter_z);                           //执行滤波，过滤结果保存在filter_z
    */
    //  体素降采样
    pcl::PointCloud<PointType>::Ptr vox_cloud(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> vox_grid;
    vox_grid.setInputCloud(pcd_cloud_in);
    vox_grid.setLeafSize(voxel_size, voxel_size, voxel_size); //设置滤波时创建的体素立方体(m)
    vox_grid.filter(*vox_cloud);
/*
    //  statisticalOutlierRemoval滤波器移除离群点
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	pcl::StatisticalOutlierRemoval<PointType> sor;   
	sor.setInputCloud(vox_cloud);                                                 
	sor.setMeanK(sor_nearby_number);                            //设置在进行统计时考虑查询点临近点数                                              
	sor.setStddevMulThresh(sor_thresh_value);                   //设置判断是否为离群点的阀值                                           
	sor.filter(*filtered_cloud);
    */
    /*
    //  点云旋转和平移操作
    for(int i = 0; i < filtered_cloud->points.size(); i++) 
    {
        PointType new_point;
        double rx_x,rx_y,rx_z,ry_x,ry_y,ry_z,rz_x,rz_y,rz_z;

        double px = filtered_cloud->points[i].x;
        double py = filtered_cloud->points[i].y;
        double pz = filtered_cloud->points[i].z;
        double pi = filtered_cloud->points[i].intensity;
        
        //  点云绕 X 旋转
        rx_x = px;
        rx_y = cos(rad(x_rotate_value))*py + (-sin(rad(x_rotate_value)))*pz;
        rx_z = sin(rad(x_rotate_value))*py + cos(rad(x_rotate_value))*pz;

        //  点云绕 Y 旋转
        ry_x = cos(rad(y_rotate_value))*rx_x + (-sin(rad(y_rotate_value)))*rx_z;
        ry_y = rx_y;
        ry_z = sin(rad(y_rotate_value))*rx_x + cos(rad(y_rotate_value))*rx_z;

        //  点云绕 Z 旋转
        rz_x = cos(rad(z_rotate_value))*ry_x + (-sin(rad(z_rotate_value)))*ry_y;
        rz_y = sin(rad(z_rotate_value))*ry_x + cos(rad(z_rotate_value))*ry_y;
        rz_z = ry_z;

        //  点云整体平移
        new_point.x = rz_x + x_trans_value;
        new_point.y = rz_y + y_trans_value;
        new_point.z = rz_z + z_trans_value;
        
        new_point.intensity = pi;
        handle_cloud->points.push_back(new_point);
    }
*/
/****************************************************************************************/

/********************************点云处理后发布并保存为新点云*********************************/  

    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*vox_cloud,output_cloud);
    output_cloud.header.frame_id = output_frame_id;
    // pointCloud_pub.publish(output_cloud);

    pcl::io::savePCDFileBinary(output_pcd_path, *vox_cloud);     // 保存为新的PCD点云
/****************************************************************************************/
    
    ros::Rate loop_rate(6);
    while (ros::ok())
    {
        read_pcd_pub.publish(input_cloud);
        pointCloud_pub.publish(output_cloud);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    return 0;
}

