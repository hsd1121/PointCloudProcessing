#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
void callback(const geometry_msgs::PointStampedConstPtr& position_data, const geometry_msgs::QuaternionStampedConstPtr& attitude_data) {

	std::cout << "Position Data: " << position_data->point.x << " " << position_data->point.y << " " << position_data->point.z << std::endl;
	std::cout << "Attitude Data: " << attitude_data->quaternion.x << " " << attitude_data->quaternion.y << " " << attitude_data->quaternion.z << " " << attitude_data->quaternion.w << std::endl;
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(position_data->point.x, position_data->point.y, position_data->point.z));
	tf::Quaternion q(attitude_data->quaternion.x, attitude_data->quaternion.y, attitude_data->quaternion.z, attitude_data->quaternion.w);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dji"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "dji_tf_broadcaster");

    ros::NodeHandle nh;

	message_filters::Subscriber<geometry_msgs::PointStamped> position_sub(nh, "/dji_sdk/local_position", 1);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_sub(nh, "/dji_sdk/attitude", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::QuaternionStamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), position_sub, attitude_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
	return 0;
};