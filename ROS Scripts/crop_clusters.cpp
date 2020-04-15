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
        std::cerr << "point_cloud_file clusters_file" << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *temp_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(argv[2]);
        PCL_ERROR("\n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream infile2(argv[2]);
    std::string line;
    int linecount = 0;
    while(std::getline(infile2, line))
    {
        if(linecount == 0)
        {
            linecount++;
            continue;
        }
        std::istringstream s(line);
        std::string field;
        int fieldcount = 0;
        int clustercount = -1;
        double rotation = -1;
        double height = -1;
        double width = -1;
        double box_x = -1;
        double box_y = -1;
        while(getline(s, field, ','))
        {
            if(fieldcount == 0)
                clustercount = std::atoi(field.c_str());
            else if(fieldcount == 1)
                rotation = std::atof(field.c_str());
            else if(fieldcount == 2)
                width = std::atof(field.c_str());
            else if(fieldcount == 3)
                height = std::atof(field.c_str());
            else if(fieldcount == 4)
                box_x = std::atof(field.c_str());
            else if(fieldcount == 5)
                box_y = std::atof(field.c_str());
           /* std::cout << clustercount << std::endl;
            std::cout << rotation << std::endl;
            std::cout << width << std::endl;
            std::cout << height << std::endl;
            std::cout << box_x << std::endl;
            std::cout << box_y << std::endl;
*/
            fieldcount++;
        }

        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setTranslation(Eigen::Vector3f(box_x, box_y, 0));
        boxFilter.setRotation(Eigen::Vector3f(0, 0, rotation));
        boxFilter.setMin(Eigen::Vector4f(0 - (height/2), 0 - (width/2), -100, 1.0));
        boxFilter.setMax(Eigen::Vector4f(0 + (height/2), 0 + (width/2), 100, 1.0));
        boxFilter.setInputCloud(temp_cloud);
        boxFilter.filter(*bodyFiltered);
        
        std::ostringstream oss;
        oss << "/home/naik/Data/clusters/point_cloud_" << clustercount + 1 << ".pcd";
        pcl::io::savePCDFileASCII(oss.str(), *bodyFiltered);

   
    }

    return 0;
}
