#include <iostream>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

int count = 0;

void octomap_cb(const octomap_msgs::Octomap& input)
{
      
    count++;
    std::cout << count << std::endl;
    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(input);
    //tree->write("test.bt");
    octomap::OcTree* ocTree = new octomap::OcTree(0.1);
    ocTree = dynamic_cast<octomap::OcTree*>(tree);
    ocTree->writeBinary("test.bt");
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("octomap_binary", 1, octomap_cb);

    ros::spin();
}
