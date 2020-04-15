#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <fstream>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <math.h>

struct RGB_VAL{
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
};

RGB_VAL hsv2rgb(double h, double sl, double l);

RGB_VAL hsv2rgb(double h, double sl, double l) {

    double v;
    double r,g,b;

    r = l;   // default to gray
    g = l;
    b = l;

    v = (l <= 0.5) ? (l * (1.0 + sl)) : (l + sl - l * sl);

    if (v > 0)  {

        double m;
        double sv;
        int sextant;
        double fract, vsf, mid1, mid2;

        m = l + l - v;
        sv = (v - m ) / v;
        h *= 6.0;

        sextant = (int)h;
        fract = h - sextant;
        vsf = v * sv * fract;
        mid1 = m + vsf;
        mid2 = v - vsf;

        switch (sextant) {
                        
            case 0:
                r = v;
                g = mid1;
                b = m;
                break;

            case 1:
                r = mid2;
                g = v;
                b = m;
                break;

            case 2:
                r = m;
                g = v;
                b = mid1;
                break;

            case 3:
                r = m;
                g = mid2;
                b = v;
                break;

            case 4:
                r = mid1;
                g = m;
                b = v;
                break;

            case 5:
                r = v;
                g = m;
                b = mid2;
                break;

        }

    }

    RGB_VAL rgb;

    rgb.r = r * 255;
    rgb.g = g * 255;
    rgb.b = b * 255;
    return rgb;

}

int main(int argc, char** argv)
{

    if(argc != 4)
    {
        std::cerr << "crop_param_file plane_param_file point_cloud_file" << std::endl;
        return -1;
    }

    double distance_threshold = 0;

    std::ifstream infile(argv[2]);

    int count = 0;
    float param = 0;
    while(infile >> param) {
        switch(count) {
            case 0:
                distance_threshold = param;
                break;
            default:
                std::cout << "Shouldn't get to this. Param file setup incorrectly." << std::endl;
                break;           
        }
        count++;
    }

    std::ostringstream oss;
    oss << argv[3];

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (oss.str(), *temp_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ");
        PCL_ERROR(oss.str().c_str());
        PCL_ERROR("\n");
        return (-1);
    }

    double min_z = 100;
    double max_z = -100;
    for(size_t i = 0; i < temp_cloud->points.size(); i++)
    {
        if(temp_cloud->points[i].z > max_z) {
            max_z = temp_cloud->points[i].z;
        }

        if(temp_cloud->points[i].z < min_z) {
            min_z = temp_cloud->points[i].z;
        }
    }

    float negative_x_filter = 0;
    float positive_x_filter = 0;
    float negative_y_filter = 0;
    float positive_y_filter = 0;
    float negative_z_filter = 0;
    float positive_z_filter = 0;
    float rotation_x = 0;
    float rotation_y = 0;
    float rotation_z = 0;

    std::ifstream infile2(argv[1]);

    count = 0;
    param = 0;
    while(infile2 >> param) {
        switch(count) {
            case 0:
                negative_x_filter = param;
                break;
            case 1:
                positive_x_filter = param;
                break;
            case 2:
                negative_y_filter = param;
                break;
            case 3:
                positive_y_filter = param;
                break;
            case 4:
                negative_z_filter = param;
                break;
            case 5:
                positive_z_filter = param;
                break;
            case 6:
                rotation_x = param;
                break;
            case 7:
                rotation_y = param;
                break;
            case 8:
                rotation_z = param;
                break;    
            default:
                std::cout << "Shouldn't get to this. Param file setup incorrectly." << std::endl;
                break;           
        }
        count++;
    }  

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    //boxFilter.setTranslation(Eigen::Vector3f(4, 1.5, 0));
    boxFilter.setMin(Eigen::Vector4f(negative_x_filter, negative_y_filter, negative_z_filter, 1.0));
    boxFilter.setMax(Eigen::Vector4f(positive_x_filter, positive_y_filter, positive_z_filter, 1.0));
    //boxFilter.setMin(Eigen::Vector4f(-1, -3, -100, 1.0));
    //boxFilter.setMax(Eigen::Vector4f(1, 3, 100, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(rotation_x, rotation_y, rotation_z));
    boxFilter.setInputCloud(temp_cloud);
    boxFilter.filter(*bodyFiltered);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);

    seg.setInputCloud(bodyFiltered);
    seg.segment(*inliers, *coefficients);
    
    if(inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_rgb->points.resize(bodyFiltered->size());
    double s = 0.5;
    double v = 0.5;
    double h = 0;
    struct RGB_VAL  rgb_out;
    for(size_t i = 0; i < bodyFiltered->points.size(); i++)
    {

        cloud_rgb->points[i].x = bodyFiltered->points[i].x;
        cloud_rgb->points[i].y = bodyFiltered->points[i].y;
        cloud_rgb->points[i].z = bodyFiltered->points[i].z;
        cloud_rgb->points[i].r = 0;
        cloud_rgb->points[i].g = 0;
        cloud_rgb->points[i].b = 0;
    }

    for(size_t i = 0; i < inliers->indices.size(); ++i)
    {
        h = (cloud_rgb->points[inliers->indices[i]].z - min_z) / (max_z - min_z);
        rgb_out = hsv2rgb(h, s, v);
        cloud_rgb->points[inliers->indices[i]].r = rgb_out.r;
        cloud_rgb->points[inliers->indices[i]].g = rgb_out.g;
        cloud_rgb->points[inliers->indices[i]].b = rgb_out.b;
    }
    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud(temp_cloud);
    //while(!viewer.wasStopped ())
    //{
    //}

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
    return 0;
}
