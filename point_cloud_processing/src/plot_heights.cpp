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
#include <sstream>
#include <cstdlib>


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
        std::cerr << "plot_only_param_file plot_all_pcd_file" << std::endl;
        return -1;
    }
    
    float negative_x_filter = 0;
    float positive_x_filter = 0;
    float negative_y_filter = 0;
    float positive_y_filter = 0;
    float negative_z_filter = 0;
    float positive_z_filter = 0;
    float rotation_x = 0;
    float rotation_y = 0;
    float rotation_z = 0;

    std::ifstream infile(argv[1]);

    int count = 0;
    float param = 0;
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
            case 6:
                rotation_x = param;
                break;
            case 7:
                rotation_y = param;
                break;
            case 8:
                rotation_z = param;
                break;    
            default:
                std::cout << "Shouldn't get to this. Param file setup incorrectly." << std::endl;
                break;           
        }
        count++;
    }  

    std::cout << "Importing plot_all pcd file" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr plot_all_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *plot_all_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(argv[2]);
        PCL_ERROR("\n");
        return (-1);
    }

    /*
    int perimeter_count = 0;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for(size_t i = 0; i < plot_only_cloud->size(); i++)
    {
        
        for(size_t j = 0; j < plot_all_cloud->size(); j++)
        {
            //std::cout << plot_all_cloud->points[j].x << " " << plot_only_cloud->points[i].x << std::endl;
            //if(true)
            if(plot_all_cloud->points[j].x == plot_only_cloud->points[i].x &&
               plot_all_cloud->points[j].y == plot_only_cloud->points[i].y &&
               plot_all_cloud->points[j].z == plot_only_cloud->points[i].z)
            {
                  //pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                  inliers->indices.push_back(j);
                  perimeter_count++;
                  std::cout << "Perimeter points count: " << perimeter_count << std::endl;
                  
                  extract.setInputCloud(plot_all_cloud);
                  extract.setIndices(inliers);
                  extract.setNegative(true);
                  extract.filter(*plot_all_cloud);
                  
                  break;
            }
            
        }
        
    }

    extract.setInputCloud(plot_all_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*plot_all_cloud);
    */

    std::cout << "Filtering plot out using parameters file" << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr plot_only_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    //boxFilter.setTranslation(Eigen::Vector3f(4, 1.5, 0));
    boxFilter.setMin(Eigen::Vector4f(negative_x_filter, negative_y_filter, negative_z_filter, 1.0));
    boxFilter.setMax(Eigen::Vector4f(positive_x_filter, positive_y_filter, positive_z_filter, 1.0));
    //boxFilter.setMin(Eigen::Vector4f(-1, -3, -100, 1.0));
    //boxFilter.setMax(Eigen::Vector4f(1, 3, 100, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(rotation_x, rotation_y, rotation_z));
    boxFilter.setInputCloud(plot_all_cloud);
    //boxFilter.setNegative(true);
    boxFilter.filter(*plot_only_cloud);

    std::cout << "Filtering perimeter out using parameters file" << std::endl;
    //boxFilter.setTranslation(Eigen::Vector3f(4, 1.5, 0));
    boxFilter.setMin(Eigen::Vector4f(negative_x_filter, negative_y_filter, negative_z_filter, 1.0));
    boxFilter.setMax(Eigen::Vector4f(positive_x_filter, positive_y_filter, positive_z_filter, 1.0));
    //boxFilter.setMin(Eigen::Vector4f(-1, -3, -100, 1.0));
    //boxFilter.setMax(Eigen::Vector4f(1, 3, 100, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(rotation_x, rotation_y, rotation_z));
    boxFilter.setInputCloud(plot_all_cloud);
    boxFilter.setNegative(true);
    boxFilter.filter(*plot_all_cloud);

    std::cout << "Saving perimeter and plot files in current directory" << std::endl;

    pcl::io::savePCDFileASCII("perimeter.pcd", *plot_all_cloud);
    pcl::io::savePCDFileASCII("plot_only.pcd", *plot_only_cloud);


    std::ofstream height_file;
    height_file.open("plot_heights.csv");
    height_file << "X position" << "," << "Y position" << "," << "Z position" << "," << "Height" << std::endl;

    std::cout << "Computing heights of plot" << std::endl;
    Vector3 sum;
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    for(size_t i = 0; i < plot_all_cloud->points.size(); i++)
    {
        sum.x += plot_all_cloud->points[i].x;
        sum.y += plot_all_cloud->points[i].y;
        sum.z += plot_all_cloud->points[i].z;
    }

    Vector3 centroid;
    centroid.x = sum.x / plot_all_cloud->size();
    centroid.y = sum.y / plot_all_cloud->size();
    centroid.z = sum.z / plot_all_cloud->size();

    double xx = 0.0;
    double xy = 0.0;
    double xz = 0.0;
    double yy = 0.0;
    double yz = 0.0;
    double zz = 0.0;

    for(size_t i = 0; i < plot_all_cloud->points.size(); i++)
    {
       Vector3 r;
       r.x = plot_all_cloud->points[i].x - centroid.x;
       r.y = plot_all_cloud->points[i].y - centroid.y;
       r.z = plot_all_cloud->points[i].z - centroid.z;
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
    int index = -1;
    for(size_t i = 0; i < plot_only_cloud->points.size(); i++)
    {
        double temp_distance = 0;
        subtract.x = plot_only_cloud->points[i].x - centroid.x;
        subtract.y = plot_only_cloud->points[i].y - centroid.y;
        subtract.z = plot_only_cloud->points[i].z - centroid.z;
        temp_distance = (dir.x * subtract.x) + (dir.y * subtract.y) + (dir.z * subtract.z);

        double mag_dist = std::abs(temp_distance);

        height_file << plot_only_cloud->points[i].x << "," << plot_only_cloud->points[i].y << "," << plot_only_cloud->points[i].z << "," << mag_dist << std::endl;

    }
    
    std::cout << "Saving plot_heights.csv file in current directory" << std::endl;
    height_file.close();
 
    return 0;
}
