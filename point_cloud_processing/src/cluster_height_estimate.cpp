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
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

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
    if(argc != 4)
    {
        std::cerr << "param_file point_cloud_file clusters_file" << std::endl;
        return -1;
    }

    double distance_threshold = 0;

    std::ifstream infile(argv[1]);

    int count = 0;
    float param = 0;
    while(infile >> param) {
        switch(count) {
            case 0:
                distance_threshold = param;
                break;
            default:
                std::cout << "Shouldn't get to this. Param file setup incorrectly." << std::endl;
                break;           
        }
        count++;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *temp_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(argv[2]);
        PCL_ERROR("\n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);

    std::ofstream height_file;
    height_file.open("/home/naik/Data/PointCloud/heights.csv");
    height_file << "Cluster #,Height (meters)\n";

    std::ifstream infile2(argv[3]);
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

        height_file << clustercount << ",";

        //height += 4;
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setTranslation(Eigen::Vector3f(box_x, box_y, 0));
        boxFilter.setRotation(Eigen::Vector3f(0, 0, rotation));
        boxFilter.setMin(Eigen::Vector4f(0 - (height/2), 0 - (width/2), -100, 1.0));
        boxFilter.setMax(Eigen::Vector4f(0 + (height/2), 0 + (width/2), 100, 1.0));
        boxFilter.setInputCloud(temp_cloud);
        boxFilter.filter(*bodyFiltered);
        
    


        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        //Optional
        seg.setOptimizeCoefficients(true);
        seg.setAxis(Eigen::Vector3f(0,0,1));
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold);

        seg.setInputCloud(bodyFiltered);
        seg.segment(*inliers, *coefficients);
    	
        if(inliers->indices.size() == 0)
        {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
            return -1;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_rgb->points.resize(bodyFiltered->size() + 2);
        for(size_t i = 0; i < bodyFiltered->points.size(); i++)
        {
            cloud_rgb->points[i].x = bodyFiltered->points[i].x;
            cloud_rgb->points[i].y = bodyFiltered->points[i].y;
            cloud_rgb->points[i].z = bodyFiltered->points[i].z;
            cloud_rgb->points[i].r = 0;
            cloud_rgb->points[i].g = 0;
            cloud_rgb->points[i].b = 255;
        }

        for(size_t i = 0; i < inliers->indices.size(); ++i)
        {
           cloud_rgb->points[inliers->indices[i]].r = 255;
           cloud_rgb->points[inliers->indices[i]].g = 0;
           cloud_rgb->points[inliers->indices[i]].b = 0;
        }

        Vector3 sum;
        sum.x = 0;
        sum.y = 0;
        sum.z = 0;
        for(size_t i = 0; i < cloud_rgb->points.size(); i++)
        {
            if(cloud_rgb->points[i].r == 255)
            {
                sum.x += cloud_rgb->points[i].x;
                sum.y += cloud_rgb->points[i].y;
                sum.z += cloud_rgb->points[i].z;
            }
        }

        Vector3 centroid;
        centroid.x = sum.x / inliers->indices.size();
        centroid.y = sum.y / inliers->indices.size();
        centroid.z = sum.z / inliers->indices.size();

        double xx = 0.0;
        double xy = 0.0;
        double xz = 0.0;
        double yy = 0.0;
        double yz = 0.0;
        double zz = 0.0;

        for(size_t i = 0; i < cloud_rgb->points.size(); i++)
        {
            if(cloud_rgb->points[i].r == 255)
            {
               Vector3 r;
               r.x = cloud_rgb->points[i].x - centroid.x;
               r.y = cloud_rgb->points[i].y - centroid.y;
               r.z = cloud_rgb->points[i].z - centroid.z;
               xx += r.x * r.x;
               xy += r.x * r.y;
               xz += r.x * r.z;
               yy += r.y * r.y;
               yz += r.y * r.z;
               zz += r.z * r.z;
            }
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
        for(size_t i = 0; i < cloud_rgb->points.size(); i++)
        {
            if(cloud_rgb->points[i].b == 255)
            {
                double temp_distance = 0;
                subtract.x = cloud_rgb->points[i].x - centroid.x;
                subtract.y = cloud_rgb->points[i].y - centroid.y;
                subtract.z = cloud_rgb->points[i].z - centroid.z;
                temp_distance = (dir.x * subtract.x) + (dir.y * subtract.y) + (dir.z * subtract.z);

                double mag_dist = std::abs(temp_distance);

                //height_file << mag_dist << ",";

                if(mag_dist > distance) {
                    distance = mag_dist;
                    point_max.x = cloud_rgb->points[i].x;
                    point_max.y = cloud_rgb->points[i].y;
                    point_max.z = cloud_rgb->points[i].z;
                    index = i;
                }
            }
        }
        
        std::cerr << "Max distance in frame(meters): " << distance << " meters." << std::endl;
        //std::cerr << "Max distance in frame(inches): " << (distance * 39.3701) << " inches." << std::endl;
        height_file << distance << ",";
        height_file << std::endl;

    }

    /*
    int size = cloud_rgb->size();
    cloud_rgb->points[size - 1].x = centroid.x;
    cloud_rgb->points[size - 1].y = centroid.y;
    cloud_rgb->points[size - 1].z = centroid.z;
    cloud_rgb->points[size - 1].r = 255;
    cloud_rgb->points[size - 1].g = 255;
    cloud_rgb->points[size - 1].b = 255;

    cloud_rgb->points[size - 2].x = dir.x;
    cloud_rgb->points[size - 2].y = dir.y;
    cloud_rgb->points[size - 2].z = dir.z;
    cloud_rgb->points[size - 2].r = 255;
    cloud_rgb->points[size - 2].g = 165;
    cloud_rgb->points[size - 2].b = 0;

    cloud_rgb->points[index].r = 0;
    cloud_rgb->points[index].g = 255;
    cloud_rgb->points[index].b = 0;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0, 0, 43, 0, 0, 0, 0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (temp_cloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ> (temp_cloud, rgb, "full");
    //viewer->addPointCloud<pcl::PointXYZ> (cloud_rgb, rgb, "Cloud");
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_rgb, "test");

    unsigned int microseconds = 100000;

    while(!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        usleep(microseconds);
    }*/
    return 0;
}
