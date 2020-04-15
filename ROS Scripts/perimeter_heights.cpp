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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
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
        std::cerr << "plot_only_file perimeter_file" << std::endl;
        return -1;
    }
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr plot_only_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *plot_only_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(argv[1]);
        PCL_ERROR("\n");
        return (-1);
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr perimeter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *perimeter_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(argv[2]);
        PCL_ERROR("\n");
        return (-1);
    }
    

    std::ofstream height_file;
    height_file.open("/home/naik/Data/turfgrass_heights.csv");

    Vector3 sum;
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    for(size_t i = 0; i < perimeter_cloud->points.size(); i++)
    {
        sum.x += perimeter_cloud->points[i].x;
        sum.y += perimeter_cloud->points[i].y;
        sum.z += perimeter_cloud->points[i].z;
    }

    Vector3 centroid;
    centroid.x = sum.x / perimeter_cloud->size();
    centroid.y = sum.y / perimeter_cloud->size();
    centroid.z = sum.z / perimeter_cloud->size();

    double xx = 0.0;
    double xy = 0.0;
    double xz = 0.0;
    double yy = 0.0;
    double yz = 0.0;
    double zz = 0.0;

    for(size_t i = 0; i < perimeter_cloud->points.size(); i++)
    {
       Vector3 r;
       r.x = perimeter_cloud->points[i].x - centroid.x;
       r.y = perimeter_cloud->points[i].y - centroid.y;
       r.z = perimeter_cloud->points[i].z - centroid.z;
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
    for(size_t i = 0; i < plot_only_cloud->points.size(); i++)
    {
        double temp_distance = 0;
        subtract.x = plot_only_cloud->points[i].x - centroid.x;
        subtract.y = plot_only_cloud->points[i].y - centroid.y;
        subtract.z = plot_only_cloud->points[i].z - centroid.z;
        temp_distance = (dir.x * subtract.x) + (dir.y * subtract.y) + (dir.z * subtract.z);

        double mag_dist = std::abs(temp_distance);

        height_file << mag_dist << std::endl;

        if(mag_dist > distance) {
            distance = mag_dist;
            point_max.x = plot_only_cloud->points[i].x;
            point_max.y = plot_only_cloud->points[i].y;
            point_max.z = plot_only_cloud->points[i].z;
            index = i;
        }
    }
    
    std::cerr << "Max distance in frame(meters): " << distance << " meters." << std::endl;
    //std::cerr << "Max distance in frame(inches): " << (distance * 39.3701) << " inches." << std::endl;


    return 0;
}
