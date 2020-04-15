#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int count = 0;
//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

struct Vector3
{
    double x;
    double y;
    double z;
};

void callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_data, const sensor_msgs::NavSatFixConstPtr& gps_data)
{
    ros::Time time_data = ros::Time::now();

    std::cout << time_data.sec << std::endl;
    count++;
    ofstream myfile;
    myfile.open("map_height_data.csv", ios::app);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pointcloud_data, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    std::cerr << "Point cloud data: " << temp_cloud->points.size () << std::endl;
    std::cerr << "Count: " << count << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(temp_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-1.5, 1.5);
    pass_y.filter(*cloud_filtered_y);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_filtered_y);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-1.5, 1.5);
    pass_z.filter(*cloud_filtered_z);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_rgb->points.resize(cloud_filtered_z->size() + 1);
    for(size_t i = 0; i < cloud_filtered_z->points.size(); i++)
    {
        cloud_rgb->points[i].x = cloud_filtered_z->points[i].x;
        cloud_rgb->points[i].y = cloud_filtered_z->points[i].y;
        cloud_rgb->points[i].z = cloud_filtered_z->points[i].z;
        cloud_rgb->points[i].r = 0;
        cloud_rgb->points[i].g = 0;
        cloud_rgb->points[i].b = 255;
    }

    Vector3 sum;
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    for(size_t i = 0; i < cloud_filtered_z->points.size(); i++)
    {
        sum.x += cloud_filtered_z->points[i].x;
        sum.y += cloud_filtered_z->points[i].y;
        sum.z += cloud_filtered_z->points[i].z;
    }

    Vector3 centroid;
    centroid.x = sum.x / cloud_filtered_z->points.size();
    centroid.y = sum.y / cloud_filtered_z->points.size();
    centroid.z = sum.z / cloud_filtered_z->points.size();

    double distance = (centroid.x * centroid.x) + (centroid.y * centroid.y) + (centroid.z * centroid.z);
    distance = sqrt(distance);
    int size = cloud_rgb->size();
    cloud_rgb->points[size - 1].x = centroid.x;
    cloud_rgb->points[size - 1].y = centroid.y;
    cloud_rgb->points[size - 1].z = centroid.z;
    cloud_rgb->points[size - 1].r = 255;
    cloud_rgb->points[size - 1].g = 255;
    cloud_rgb->points[size - 1].b = 255;

    //std::cout << sum.x << " " << sum.y << " " << sum.z << std::endl;
    //std::cout << centroid.x << " " << centroid.y << " " << centroid.z << std::endl;
    
    //std::cout << std::fixed << std::setprecision(8) << gps_data->latitude << " " << gps_data->longitude << std::endl;
    myfile << std::fixed << std::setprecision(8) << count << "," << gps_data->latitude << "," << gps_data->longitude << "," 
           << gps_data->altitude << "," << distance << ","
           << time_data.sec << "," << time_data.nsec << std::endl;
    myfile.close();

    //viewer.showCloud(cloud_rgb);

}

int main(int argc, char** argv)
{
    ofstream myfile;
    myfile.open("map_height_data.csv");
    myfile.close();

    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "velodyne_points", 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "/dji_sdk/gps_position", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::NavSatFix> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointcloud_sub, gps_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}