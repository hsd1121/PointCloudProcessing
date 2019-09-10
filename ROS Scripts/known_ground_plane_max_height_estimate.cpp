#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <math.h>
#include <string>
#include <fstream>

struct Vector3
{
    double x;
    double y;
    double z;
};

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cerr << "ground_plane_file point_cloud_file" << std::endl;
        return -1;
    }

    double distance_threshold = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *ground_plane) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(argv[1]);
        PCL_ERROR("\n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *temp_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(argv[2]);
        PCL_ERROR("\n");
        return (-1);
    }

    Vector3 sum;
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    for(size_t i = 0; i < ground_plane->points.size(); i++)
    {
        sum.x += ground_plane->points[i].x;
        sum.y += ground_plane->points[i].y;
        sum.z += ground_plane->points[i].z;
    }

    Vector3 centroid;
    centroid.x = sum.x / ground_plane->points.size();
    centroid.y = sum.y / ground_plane->points.size();
    centroid.z = sum.z / ground_plane->points.size();

    double xx = 0.0;
    double xy = 0.0;
    double xz = 0.0;
    double yy = 0.0;
    double yz = 0.0;
    double zz = 0.0;

    for(size_t i = 0; i < ground_plane->points.size(); i++)
    {

       Vector3 r;
       r.x = ground_plane->points[i].x - centroid.x;
       r.y = ground_plane->points[i].y - centroid.y;
       r.z = ground_plane->points[i].z - centroid.z;
       xx += r.x * r.x;
       xy += r.x * r.y;
       xz += r.x * r.z;
       yy += r.y * r.y;
       yz += r.y * r.z;
       zz += r.z * r.z;
    }

    double det_x = yy*zz - yz*yz;
    double det_y = xx*zz - xz*xz;
    double det_z = xx*yy - xy*xy;

    double max = det_x;
    int max_letter = 1;
    if(det_y > max)
    {
        max = det_y;
        max_letter = 2;
    }
    if(det_z > max)
    {
        max = det_z;
        max_letter = 3;
    }

    Vector3 dir;
    if(max_letter == 1)
    {
        dir.x = det_x;
        dir.y = xz*yz - xy*zz;
        dir.z = xy*yz - xz*yy;
    }
    else if(max_letter == 2)
    {
        dir.x = xz*yz - xy*zz;
        dir.y = det_y;
        dir.z = xy*xz - yz*xx;
    }
    else if(max_letter == 3)
    {
        dir.x = xy*yz - xz*yy;
        dir.y = xy*xz - yz*xx;
        dir.z = det_z;
    }

    double length = (dir.x * dir.x) + (dir.y * dir.y) + (dir.z * dir.z);
    length = sqrt(length);

    dir.x = dir.x / length;
    dir.y = dir.y / length;
    dir.z = dir.z / length;

    double distance = 0;
    Vector3 subtract;
    Vector3 point_max;
    int index = -1;
    for(size_t i = 0; i < temp_cloud->points.size(); i++)
    {
        double temp_distance = 0;
        subtract.x = temp_cloud->points[i].x - centroid.x;
        subtract.y = temp_cloud->points[i].y - centroid.y;
        subtract.z = temp_cloud->points[i].z - centroid.z;
        temp_distance = (dir.x * subtract.x) + (dir.y * subtract.y) + (dir.z * subtract.z);
        
        std::cout << temp_distance << std::endl;
        //std::cout << subtract.y << std::endl;
        //std::cout << subtract.z << std::endl;
        //std::cout << yy << std::endl;
        //std::cout << yz << std::endl;
        double mag_dist = std::abs(temp_distance);
        if(mag_dist > distance) {
            distance = mag_dist;
            point_max.x = temp_cloud->points[i].x;
            point_max.y = temp_cloud->points[i].y;
            point_max.z = temp_cloud->points[i].z;
            index = i;
        }
    }
    
    std::cerr << "Max distance in frame(meters): " << distance << " meters." << std::endl;
    std::cerr << "Max distance in frame(inches): " << (distance * 39.3701) << " inches." << std::endl;

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(ground_plane);
    while(!viewer.wasStopped ())
    {
    }

    return 0;
}
