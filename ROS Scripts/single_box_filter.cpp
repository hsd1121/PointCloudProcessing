#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <fstream>

int main(int argc, char** argv)
{

    if(argc != 3)
    {
        std::cerr << "param_file point_cloud_file" << std::endl;
        return -1;
    }

    double negative_x_filter = 0;
    double positive_x_filter = 0;
    double negative_y_filter = 0;
    double positive_y_filter = 0;
    double negative_z_filter = 0;
    double positive_z_filter = 0;

    std::ifstream infile(argv[1]);

    int count = 0;
    double param = 0;
    while(infile >> param) {
        switch(count) {
            case 0:
                negative_x_filter = param;
                break;
            case 1:
                positive_x_filter = param;
                break;
            case 2:
                negative_y_filter = param;
                break;
            case 3:
                positive_y_filter = param;
                break;
            case 4:
                negative_z_filter = param;
                break;
            case 5:
                positive_z_filter = param;
                break;  
            default:
                std::cout << "Shouldn't get to this. Param file setup incorrectly." << std::endl;
                break;           
        }
        count++;
    }  

    std::ostringstream oss;
    oss << argv[2];

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (oss.str(), *temp_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(oss.str().c_str());
        PCL_ERROR("\n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(temp_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(negative_x_filter, positive_x_filter);
    pass_x.filter(*cloud_filtered_x);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_filtered_x);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(negative_y_filter, positive_y_filter);
    pass_y.filter(*cloud_filtered_y);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_filtered_y);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(negative_z_filter, positive_z_filter);
    pass_z.filter(*cloud_filtered_z);

    std::string new_name = oss.str().substr(0, oss.str().size()-4);
    std::ostringstream oss2;
    oss2 << new_name << "_filtered.pcd";
    pcl::io::savePCDFileASCII(oss2.str(), *cloud_filtered_z);
    return 0;
}
