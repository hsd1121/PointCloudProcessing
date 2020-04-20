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
#include <pcl/visualization/cloud_viewer.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>

std::ostringstream oss;
std::ostringstream oss2;
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
      
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
   	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    //std::cerr << "Point cloud data: " << temp_cloud->points.size () << std::endl;

    viewer.showCloud(temp_cloud);

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/zed/point_cloud/cloud_registered", 1, cloud_cb);

    ros::spin();
}
