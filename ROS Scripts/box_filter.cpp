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
#include <pcl/filters/passthrough.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>

int count = 0;
std::ostringstream oss;
std::ostringstream oss2;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
      
    count++;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
   	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    std::cerr << "Point cloud data: " << temp_cloud->points.size () << std::endl;
    std::cerr << "Count: " << count << std::endl;

/*   	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(temp_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-5.0, 5.0);
    pass_x.filter(*cloud_filtered_x);
*/
   	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(temp_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-5.0, 5.0);
    pass_y.filter(*cloud_filtered_y);

    std::ostringstream oss3;
    oss3 << oss2.str() << "/filtered_point_cloud_" << count << ".pcd";
    pcl::io::savePCDFileASCII(oss3.str(), *cloud_filtered_y);

}

int main(int argc, char** argv)
{

    std::time_t t = std::time(0);
    std::tm* now = std::localtime(&t);

    oss << "/home/user01/Data/PointCloud/" << (now->tm_mon + 1) << "-"
        << (now->tm_mday) << "-" << (now->tm_year + 1900);
    mode_t nMode = 0733;
    int nError = 0;
    nError = mkdir(oss.str().c_str(), nMode);

    oss2 << "/home/user01/Data/PointCloud/" << (now->tm_mon + 1) << "-"
        << (now->tm_mday) << "-" << (now->tm_year + 1900) << "/filtered";
    nError = mkdir(oss2.str().c_str(), nMode);

    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("velodyne_points", 1, cloud_cb);

    ros::spin();
}
