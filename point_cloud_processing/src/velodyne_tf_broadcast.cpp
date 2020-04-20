#define _USE_MATH_DEFINES

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "velodyne_tf_broadcaster");

    ros::NodeHandle nh;

	tf::TransformBroadcaster br;
	tf::Transform transform;

	geometry_msgs::Quaternion orientation;
	orientation.x = 0;
	orientation.y = 0;
	orientation.z = 0;
	orientation.w = 1;

	tf2::Quaternion q_orig, q_rot, q_new;

	tf2::convert(orientation , q_orig);
	double r = 0, p = M_PI_2, y = M_PI_2;

	q_rot.setRPY(r, p, y);

	q_new = q_rot * q_orig;
	q_new.normalize();


	ros::Rate rate(10.0);
	while (nh.ok()){
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		transform.setRotation( tf::Quaternion(q_new.x(), q_new.y(), q_new.z(), q_new.w()) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dji", "velodyne"));
		rate.sleep();
	}

    ros::spin();
	return 0;
};