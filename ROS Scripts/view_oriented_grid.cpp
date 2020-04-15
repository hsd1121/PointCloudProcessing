#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <fstream>

Eigen::Quaternionf euler2Quaternion(const double roll, const double pitch, const double yaw);

Eigen::Quaternionf euler2Quaternion(const double roll, const double pitch, const double yaw)
{
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());

        Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;
        return q;
}

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cerr << "Expecting input argument: point_cloud_file clusters_data_file" << std::endl;
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_rgb->points.resize(temp_cloud->size());
    double min_z = 100;
    double max_z = -100;
    for(size_t i = 0; i < temp_cloud->points.size(); i++)
    {
        if(temp_cloud->points[i].z > max_z) {
            max_z = temp_cloud->points[i].z;
        }

        if(temp_cloud->points[i].z < min_z) {
            min_z = temp_cloud->points[i].z;
        }
    }

    for(size_t i = 0; i < temp_cloud->points.size(); i++)
    {

        cloud_rgb->points[i].x = temp_cloud->points[i].x;
        cloud_rgb->points[i].y = temp_cloud->points[i].y;
        cloud_rgb->points[i].z = temp_cloud->points[i].z;
        cloud_rgb->points[i].r = 255;
        cloud_rgb->points[i].g = 255;
        cloud_rgb->points[i].b = 255;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    
    std::ifstream infile(argv[2]);
    std::string line;
    int linecount = 0;
    int obb_count = 0;
    while(std::getline(infile, line))
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
                height = std::atof(field.c_str());
            else if(fieldcount == 3)
                width = std::atof(field.c_str());
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
        std::ostringstream obb_number;
        obb_number << "obb  " << obb_count;
        obb_count++;
        
        Eigen::Quaternionf q = euler2Quaternion(rotation, 0, 0);

        viewer->addCube(Eigen::Vector3f(box_x, box_y, 0), q, width, height, 0, obb_number.str());
        //viewer->addCube(box_x - (width/2), box_x + (width/2), box_y - (height/2), box_y + (height/2), 0, 0, 0, 0, 0, obb_number.str());

    }
    //viewer->addCube(9, 11, -1.5, 4.5, 0, 0, 0, 0, 0, "grid");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud(temp_cloud);
    //while(!viewer.wasStopped ())
    //{
    //}

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
    return 0;
}
