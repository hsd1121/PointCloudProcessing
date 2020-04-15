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
#include <fstream>
#include <string>

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
        //viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, obb_number.str());

        count++;
    }

    std::ifstream average_file ("/home/user01/Data/averages.csv");
    std::string value;
    int count2 = 0;
    double orientation_average = -1.0;
    double width_average = -1.0;
    double height_average = -1.0;

    while(average_file.good()) {
        getline(average_file, value, ',');
        std::cout << value << std::endl;
        if(count2 == 0) {
            orientation_average = std::atof(value.c_str());
        }
        else if(count2 == 1) {
            width_average = std::atof(value.c_str());
        }
        else if(count2 == 2) {
            height_average = std::atof(value.c_str());
        }
        count2++;
    }

    orientation_average += 0.03;
    height_average -= 0.4;
    float roll = 0, pitch = 0, yaw = orientation_average;
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    std::ofstream cluster_file;
    cluster_file.open("/home/user01/Data/correct_clusters_params.csv");
    cluster_file << "Cluster #,Rotation,Height,Width,X,Y\n";
    int cluster_count = 0;
    for(count2 = 0; count2 < 28; count2++)
    {
        std::ostringstream cube_number;
        cube_number << "cube_" << count2;
        double width = -10.54 + (count2 * 1.122222);
        double height = -18.07222222222 + (count2 * 0.955555);
        Eigen::Vector3f position (width, height, 0);
        viewer->addCube (position, q, width_average, height_average, 0, cube_number.str());
        cluster_file << cluster_count << "," << orientation_average << "," << height_average << "," << width_average
                         << "," << width << "," << height << "\n";
        cluster_count++;
        for(int i = 1; i < 4; i++) {
            cube_number << "_" << i;
            double width2 = width + (i * -3.25);
            double height2 = height + (i * 3.85);
            Eigen::Vector3f position2 (width2, height2, 0);
            viewer->addCube (position2, q, width_average, height_average, 0, cube_number.str());
            cluster_file << cluster_count << "," << orientation_average << "," << height_average << "," << width_average
                         << "," << width2 << "," << height2 << "\n";
            cluster_count++;
        }
    }

    cluster_file.close();

    // Eigen::Vector3f position (10.9, 6.8, 0);
    // //Eigen::Quaternionf quat (rotational_matrix_OBB);
    // viewer->addCube (position, q, width_average, height_average, 0, "test");
    // Eigen::Vector3f position2 (0.8, -1.8, 0);
    // //Eigen::Quaternionf quat (rotational_matrix_OBB);
    // viewer->addCube (position2, q, width_average, height_average, 0, "test2");
    // Eigen::Vector3f position3 (8.656, 4.89, 0);
    // //Eigen::Quaternionf quat (rotational_matrix_OBB);
    // viewer->addCube (position3, q, width_average, height_average, 0, "test3");

    unsigned int microseconds = 100000;

    while(!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        usleep(microseconds);
    }

    return (0);
}
