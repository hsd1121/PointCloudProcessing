#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>

std::ostringstream oss;
std::ostringstream oss2;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
      
    std::time_t t = std::time(0);
    std::tm* now = std::localtime(&t);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
   	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    std::cerr << "Point cloud data: " << temp_cloud->points.size () << std::endl;

    std::ostringstream oss3;
    oss3 << oss2.str() << "/point_cloud_" << now->tm_hour << "-"
                                          << now->tm_min << "-"
                                          << now->tm_sec << ".pcd";
    pcl::io::savePCDFileASCII(oss3.str(), *temp_cloud);
    ros::shutdown();
}

int main(int argc, char** argv)
{

    std::time_t t = std::time(0);
    std::tm* now = std::localtime(&t);
    oss << "/home/user/Data/PointCloud/" << (now->tm_mon + 1) << "-" 
        << (now->tm_mday) << "-" << (now->tm_year + 1900);
    mode_t nMode = 0733;
    int nError = 0;
    nError = mkdir(oss.str().c_str(), nMode);

    oss2 << "/home/user/Data/PointCloud/" << (now->tm_mon + 1) << "-" 
        << (now->tm_mday) << "-" << (now->tm_year + 1900) << "/single";
    nError = mkdir(oss2.str().c_str(), nMode);

    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("velodyne_points", 1, cloud_cb);

    ros::spin();
}
