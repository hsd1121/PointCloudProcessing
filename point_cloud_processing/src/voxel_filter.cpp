#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <fstream>

int main (int argc, char** argv)
{

  if(argc != 3)
  {
      std::cerr << "param_file point_cloud_file" << std::endl;
      return -1;
  }

  double x_leaf = 0;
  double y_leaf = 0;
  double z_leaf = 0;

  std::ifstream infile(argv[1]);

  int count = 0;
  float param = 0;
  while(infile >> param) {
      switch(count) {
          case 0:
              x_leaf = param;
              break;
          case 1:
              y_leaf = param;
              break;
          case 2:
              z_leaf = param;
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

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

  pcl::toPCLPointCloud2(*temp_cloud, *cloud);
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (x_leaf, y_leaf, z_leaf);
  sor.filter (*cloud_filtered);


  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2( *cloud_filtered, *pc_filtered);

  std::string new_name = oss.str().substr(0, oss.str().size()-4);
  std::ostringstream oss2;
  oss2 << new_name << "_voxel_filtered.pcd";
  pcl::io::savePCDFileASCII(oss2.str(), *pc_filtered);
  return (0);
}