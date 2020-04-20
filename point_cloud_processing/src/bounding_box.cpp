#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <unistd.h>
#include <list>
#include <iterator>
#include <pthread.h>
#include <pcl/features/moment_of_inertia_estimation.h>

unsigned int microseconds = 10000;

int main(int argc, char** argv)
{
    std::list <pcl::PointCloud<pcl::PointXYZ> > cluster_list;
    if(argc != 3)
    {
        std::cerr << "Expecting input argument: folder #_of_.pcd_files " << std::endl;
        return -1;
    }

    std::ostringstream oss;
    oss << argv[1] << "cluster_";

    std::istringstream ss(argv[2]);
    int x;
    if(!(ss >> x)) {
        std::cerr << "Invalid number: " << argv[1] << '\n';
    }
    else if (!ss.eof()) {
        std::cerr << "Trailing characters after number: " << argv[1] << '\n';
    }

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0, 0, 43, 0, 0, 0, 0, 0, 0);
    for(int i = 0; i < x; i++) {
        std::cout << "Count: " << i << std::endl;
        std::ostringstream oss2;
        oss2 << oss.str() << i << ".pcd";
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if(pcl::io::loadPCDFile<pcl::PointXYZ> (oss2.str(), cloud) == -1)
        {
            PCL_ERROR("Couldn't read file ");
            PCL_ERROR(oss.str().c_str());
            PCL_ERROR("\n");
            return (-1);
        }
        cluster_list.push_back(cloud);
    }

    int count = 0;
    std::list <pcl::PointCloud<pcl::PointXYZ> >::iterator it;
    for(it = cluster_list.begin(); it != cluster_list.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*it, *cloud);

        std::ostringstream cluster_number;
        cluster_number << "cluster " << count;
        std::ostringstream obb_number;
        obb_number << "obb  " << count;

        double average = 0;
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            average += cloud->points[i].z;
        }
        average /= cloud->points.size();

        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            cloud->points[i].z = average;
        }

        pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud (cloud);
        feature_extractor.compute ();

        std::vector <float> moment_of_inertia;
        std::vector <float> eccentricity;
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getMomentOfInertia (moment_of_inertia);
        feature_extractor.getEccentricity (eccentricity);
        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues (major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter (mass_center);


        viewer->addPointCloud<pcl::PointXYZ> (cloud, cluster_number.str());

        Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat (rotational_matrix_OBB);
        viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, obb_number.str());

        count++;
    }

    unsigned int microseconds = 100000;
    //viewer->spinOnce (100);
    //usleep(microseconds);

    count = 0;
    for(it = cluster_list.begin(); it != cluster_list.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*it, *cloud);
        std::ostringstream cluster_number;
        cluster_number << "cluster " << count;
        std::ostringstream obb_number;
        obb_number << "obb  " << count;
        viewer->removePointCloud(cluster_number.str());

        for(int i = 0; i < cloud->points.size(); i++) {
            cloud->points[i].r = 255;
            cloud->points[i].r = 0;
            cloud->points[i].r = 0;
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(cloud, 255, 0, 0); 
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cluster_number.str());
        viewer->spinOnce (100);
            

        bool valid = false;
        while(!valid) {
            viewer->spinOnce (100);
            std::cout << "Is this cluster right? (y/n): ";
            char input;
            std::cin >> input;
            if(input == 'y') {
                valid = true;
                viewer->removePointCloud(cluster_number.str());
                viewer->removeShape(obb_number.str());
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(cloud, 0, 0, 255);
                viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cluster_number.str());
                std::ostringstream oss;
                oss << "/home/user01/Data/CorrectCluster/cluster_" << count << ".pcd";
                pcl::io::savePCDFileASCII(oss.str(), *cloud);
            }
            else if(input == 'n') {
                valid = true;
                viewer->removePointCloud(cluster_number.str());
                viewer->removeShape(obb_number.str());
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(cloud, 255, 255, 255);
                viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cluster_number.str());
            }
            else {
                valid = false;
            }
        }

        count++;
        //viewer->spinOnce(100);
        //usleep(microseconds * 3);
    }
/*
    while(!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        usleep(microseconds);
    }
*/
    return (0);
}
