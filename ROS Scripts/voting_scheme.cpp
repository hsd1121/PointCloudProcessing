#include <iostream>
#include <fstream>
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

    std::ofstream myfile;
    myfile.open ("/home/naik/Data/PointCloud/2-22-2020_Simulated/clusters_data.csv");
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

        Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat (rotational_matrix_OBB);

        Eigen::Vector3f euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);
        double width = max_point_OBB.x - min_point_OBB.x;
        double height = max_point_OBB.y - min_point_OBB.y;

        myfile << count << "," << euler[0] << "," << width << "," << height << ","
               << position_OBB.x << "," << position_OBB.y << "," << position_OBB.z 
               << std::endl;
        std::cout << "Yaw: " << euler[0] << std::endl;
        std::cout << "Width: " << width << std::endl;
        std::cout << "Height: " << height << std::endl;

        count++;
    }

    myfile.close();
    return (0);
}
