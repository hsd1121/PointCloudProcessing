#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    //boxFilter.setTranslation(Eigen::Vector3f(4, 1.5, 0));
    boxFilter.setMin(Eigen::Vector4f(negative_x_filter, negative_y_filter, negative_z_filter, 1.0));
    boxFilter.setMax(Eigen::Vector4f(positive_x_filter, positive_y_filter, positive_z_filter, 1.0));
    //boxFilter.setMin(Eigen::Vector4f(-1, -3, -100, 1.0));
    //boxFilter.setMax(Eigen::Vector4f(1, 3, 100, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(rotation_x, rotation_y, rotation_z));
    boxFilter.setInputCloud(temp_cloud);
    boxFilter.filter(*bodyFiltered);

    std::string new_name = oss.str().substr(0, oss.str().size()-4);
    std::ostringstream oss2;
    oss2 << new_name << "_crop_box_filtered.pcd";
    pcl::io::savePCDFileASCII(oss2.str(), *bodyFiltered);
    return 0;
}
