//用于把点云由pcd文件转为ply文件
#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>  

using namespace std;
using namespace pcl;

string pcd_path = "/home/bupo/r3live_output/rgb_pt.pcd";
string ply_path = "/home/bupo/r3live_output/rgb_pt.ply";

int PCDtoPLYconvertor(string & input_filename ,string& output_filename)
{
    //此处的PointXYZI可以换成PointXYZ等其他格式的，看自己点云的格式来，只要整个程序统一即可
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(input_filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file \n");
        return (-1);
    }
    pcl::PLYWriter writer;
    writer.write<pcl::PointXYZRGBA> (output_filename, *cloud, false);
    // writer.write(ply_path, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),true,true);

    return 0;

}


int main(int argc, char **argv){
    PCDtoPLYconvertor(pcd_path,ply_path);
    return 0;
}


