/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:39:00
 */
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "glog/logging.h"

namespace lidar_localization {
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) {
    CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));//获取输入初始点云
    output_cloud_ptr->points.clear();//定义空的输出点云

    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);//获取起始点的角度位置

    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());//定义旋转轴的旋转向量
    Eigen::Matrix3f rotate_matrix = t_V.matrix();//转变为旋转矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();//定义变换矩阵
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();//定义逆变换
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);//将点云初始点与旋转原点对其

    velocity_ = rotate_matrix.inverse() * velocity_;
    angular_rate_ = rotate_matrix.inverse() * angular_rate_;//从原来的lidar坐标系转换到以start orientation为准的坐标系

    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);//获取点云中当前点角度
        if (orientation < 0.0)//保证角度范围在0～2pi
            orientation += 2.0 * M_PI;
        
        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)//保证角度不过小，距原点小于5度
            continue;

        //在kitti原始数据里，提供了每帧点云的起始采集时刻和终止采集时刻，kitti2bag这个功能包在把数据转成bag文件的过程中，
        // 利用起始时刻和终止时刻取了个平均值，即中间时刻，作为这一帧点云的采集时刻，上面的方法其实是把点全都转到起始时刻上去，
        // 所以我们在计算的每个激光点采集时刻上再减去50ms，这样就相当于把一帧点云的坐标系转到中间时刻对应的坐标系上去了。
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);//将当前点重新构造一个，避免在原始数据上操作

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);//获取该短时间内的旋转矩阵 Tsp_cp
        Eigen::Vector3f rotated_point = current_matrix * origin_point;//Psp_cp =Tsp_cp * Pcp_cp
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;//其实就是坐标系×向量，坐标系是转换矩阵，向量是转换之前的激光点坐标，这个转换可以先旋转再平移，也可以用4X4矩阵一次性计算，都行，不是核心问题
        CloudData::POINT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);//将去畸变后的点放进输出点云中
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());//将去畸变后的点云转换会原先的初始位置
    return true;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {
    Eigen::Vector3f angle = angular_rate_ * real_time;//该段时间内的旋转量
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;//得到当前点在第一个点坐标系下的变换 Tsp_cp
    return t_V.matrix();
}
} // namespace lidar_localization