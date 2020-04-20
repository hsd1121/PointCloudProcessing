#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>


int main (int argc, char** argv)
{
	if(argc != 3)
    {
        std::cerr << "Expecting input argument: folder #_of_.pcd_files " << std::endl;
        return -1;
    }

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    std::ostringstream oss2;
	oss2 << oss.str() << 1 << ".pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (oss2.str(), *Final) == -1)
    {
    	PCL_ERROR("Couldn't read file ");
        PCL_ERROR(oss.str().c_str());
        PCL_ERROR("\n");
        return (-1);
    }
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

    for(int i = 2; i <= x; i++) {
        std::cout << "Count: " << i << std::endl;
        std::ostringstream oss2;
        oss2 << oss.str() << i << ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ> (oss2.str(), *cloud_out) == -1)
        {
            PCL_ERROR("Couldn't read file ");
            PCL_ERROR(oss.str().c_str());
            PCL_ERROR("\n");
            return (-1);
        }
		icp.setInputSource(Final);
		icp.setInputTarget(cloud_out);
		icp.align(*Final);
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		viewer.showCloud(Final);
    }

	return (0);
}