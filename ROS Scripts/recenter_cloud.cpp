#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

struct Vector3
{
    double x;
    double y;
    double z;
};

int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cerr << "Expecting input argument: file" << std::endl;
        return -1;
    }

    std::ostringstream oss;
    oss << argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (oss.str(), *temp_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(oss.str().c_str());
        PCL_ERROR("\n");
        return (-1);
    }

    Vector3 sum;
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    for(size_t i = 0; i < temp_cloud->points.size(); i++)
    {
        sum.x += temp_cloud->points[i].x;
        sum.y += temp_cloud->points[i].y;
        sum.z += temp_cloud->points[i].z;
    }

    Vector3 centroid;
    centroid.x = sum.x / temp_cloud->points.size();
    centroid.y = sum.y / temp_cloud->points.size();
    centroid.z = sum.z / temp_cloud->points.size();

    for(size_t i = 0; i < temp_cloud->points.size(); i++)
    {

        temp_cloud->points[i].x -= centroid.x;
        temp_cloud->points[i].y -= centroid.y;
        temp_cloud->points[i].z -= centroid.z;
    }

    std::string new_name = oss.str().substr(0, oss.str().size()-4);
    std::ostringstream oss2;
    oss2 << new_name << "_recentered.pcd";
    pcl::io::savePCDFileASCII(oss2.str(), *temp_cloud);
    return 0;
}
