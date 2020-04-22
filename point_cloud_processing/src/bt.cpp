#include <iostream>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>

int count = 0;

void octomap_cb(const octomap_msgs::Octomap& input)
{
      
    count++;
    std::cout << count << std::endl;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("octomap_binary", 1, octomap_cb);

    ros::spin();
}
