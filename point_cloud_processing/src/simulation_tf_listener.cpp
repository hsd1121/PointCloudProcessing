#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

int count = 0;
//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
tf::TransformListener *listener = NULL;  

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    std::cerr << "Point cloud data: " << temp_cloud->points.size () << std::endl;
    std::cerr << "Count: " << count << std::endl;
    
    tf::StampedTransform transform;
    Eigen::Affine3d transform_eigen;
    tf::Vector3 origin;
    tf::Quaternion rotation;
    try{
      listener->lookupTransform("/world", "/laser0_frame",  
                                  ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }
    count++;

    origin = transform.getOrigin();
    rotation = transform.getRotation();

    std::cout << "Origin: " << origin.x() << " " << origin.y() << " " << origin.z() << " " << origin.w() << std::endl;
    std::cout << "Quaternion: " << rotation.x() << " " << origin.y() << " " << rotation.z() << " " << rotation.w() << std::endl;
    tf::transformTFToEigen(transform, transform_eigen);

}

int main(int argc, char** argv){

   ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  // tf::TransformListener listener;  
  listener = new(tf::TransformListener);

  ros::Subscriber sub = node.subscribe("velodyne_points", 1, cloud_cb);

  // ros::Rate rate(10.0);
  // while (node.ok()){
  //   tf::StampedTransform transform;
  //   tf::Vector3 origin;
  //   tf::Quaternion rotation;
  //   Eigen::Affine3d transform_eigen;
  //   try{
  //     listener->lookupTransform("/velodyne", "/world",  
  //                                 ros::Time(0), transform);
  //   }
  //   catch (tf::TransformException ex){
  //     ROS_ERROR("%s",ex.what());
  //     ros::Duration(1.0).sleep();
  //   }

  //   origin = transform.getOrigin();
  //   rotation = transform.getRotation();

  //   std::cout << "Origin: " << origin.x() << " " << origin.y() << " " << origin.z() << " " << origin.w() << std::endl;
  //   std::cout << "Quaternion: " << rotation.x() << " " << origin.y() << " " << rotation.z() << " " << rotation.w() << std::endl;
  //   tf::transformTFToEigen(transform, transform_eigen);

  //   rate.sleep();
  // }

  ros::spin();
  return 0;
}
