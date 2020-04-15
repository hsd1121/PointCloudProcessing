#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
  srand (time (NULL));

  if(argc != 2)
  {
      std::cerr << "Expecting input argument: file" << std::endl;
      return -1;
  }

  std::ostringstream oss;
  oss << argv[1];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (oss.str(), *cloud) == -1)
  {
      PCL_ERROR("Couldn't read file ");
      PCL_ERROR(oss.str().c_str());
      PCL_ERROR("\n");
      return (-1);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint;

  searchPoint.x = 0;
  searchPoint.y = 0;
  searchPoint.z = 0;

  // K nearest neighbor search

  int K = 1;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[0] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[0] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[0] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[0] << ")" << std::endl;
  }


  return 0;
}