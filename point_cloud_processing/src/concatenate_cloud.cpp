#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sstream>


int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cerr << "Expecting input argument: folder #_of_.pcd_files " << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> concatenated_cloud;

    std::ostringstream oss;
    oss << argv[1] << "point_cloud_";

    std::istringstream ss(argv[2]);
    int x;
    if(!(ss >> x)) {
        std::cerr << "Invalid number: " << argv[1] << '\n';
    }
    else if (!ss.eof()) {
        std::cerr << "Trailing characters after number: " << argv[1] << '\n';
    }

    for(int i = 1; i <= x; i++) {
        std::cout << "Count: " << i << std::endl;
        std::ostringstream oss2;
        oss2 << oss.str() << i << ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ> (oss2.str(), *temp_cloud) == -1)
        {
            PCL_ERROR("Couldn't read file ");
            PCL_ERROR(oss.str().c_str());
            PCL_ERROR("\n");
            return (-1);
        }

        concatenated_cloud += *temp_cloud;
    }

    std::cout << "Done concatenating." << std::endl;
    std::cout << "SAVING FILE! DO NOT EXIT YET!" << std::endl;

    pcl::io::savePCDFileASCII("/home/naik/Data/PointCloud/concatenated_cloud.pcd", concatenated_cloud);

    std::cout << "Done saving file." << std::endl;
    return 0;
}
