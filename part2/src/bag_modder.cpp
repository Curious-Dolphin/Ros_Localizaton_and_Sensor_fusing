#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <string>

class sub_Pub_bag{
    private:
    	ros::NodeHandle n;
    	ros::Subscriber sub;
    	ros::Publisher imu_pub;
    	nav_msgs::Odometry odom;
    	geometry_msgs::Quaternion odom_quat;
    	sensor_msgs::Imu copy;
    public:
    	void callback(const sensor_msgs::Imu::ConstPtr& msg){
    		copy.header.frame_id = "frame_imu";
			copy.header.seq = msg->header.seq;
			copy.header.stamp = msg->header.stamp;
	
			copy.orientation.x = msg->orientation.x;
			copy.orientation.y = msg->orientation.y;
			copy.orientation.z = msg->orientation.z;
			copy.orientation.w = msg->orientation.w;
			copy.orientation_covariance = msg->orientation_covariance;
	
			copy.angular_velocity.x = msg->angular_velocity.x;
			copy.angular_velocity.y = msg->angular_velocity.y;
			copy.angular_velocity.z = msg->angular_velocity.z;
			copy.angular_velocity_covariance = msg->angular_velocity_covariance;
	
			copy.linear_acceleration.x = msg->linear_acceleration.x;
			copy.linear_acceleration.y = msg->linear_acceleration.y;
			copy.linear_acceleration.z = msg->linear_acceleration.z;
			copy.linear_acceleration_covariance = msg->linear_acceleration_covariance;
	
			imu_pub.publish(copy);
    	}

    	sub_Pub_bag() {
	  		sub=n.subscribe("/swiftnav/rear/imu",1,&sub_Pub_bag::callback,this);
	  		imu_pub=n.advertise<sensor_msgs::Imu>("imu/data_copied",5000);
	  	}
};


int main(int argc, char *argv[])
{

	ros::init(argc, argv, "modder");
	sub_Pub_bag obj;
    ros::spin();
}