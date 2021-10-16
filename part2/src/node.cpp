#include "ros/ros.h"
#include <math.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#define l 1.765
#define PI 3.14159265

class sub_Pub{
    private:
    	ros::NodeHandle n;
    	ros::Subscriber sub;
    	ros::Publisher odom_pub;
    	nav_msgs::Odometry odom;
    	geometry_msgs::Quaternion odom_quat;
    	ros::Time last_time,current_time;
    	float steer;
    	float v;
    	int i;
    	float theta,Xatt,Yatt,Ts,omega;



	public:
		void ackerman(){
      		Ts=(current_time - last_time).toSec();
          last_time = current_time;           //Made in order to avoid the error in the first cycle caused by the difference in time stamps
      		if(i ==0 || Ts<0)
            Ts=0.04;
  	  		omega=((v/l)*tan(steer * PI/180));
  	  		theta= theta+(omega*Ts);
      		Xatt=Xatt+(v*Ts*cos(theta+((omega*Ts)/2)));
      		Yatt=Yatt+(v*Ts*sin(theta+((omega*Ts)/2)));
          

    }
		void callback(const geometry_msgs::PointStamped::ConstPtr& msg){
			v=msg->point.y/3.6;        //Pubblishing odometry on the topic
			steer=(msg->point.x)/18;
			current_time = msg->header.stamp;
			ackerman();
      odom.header.stamp = ros::Time::now();
			odom.pose.pose.position.x = Xatt;
      odom.pose.pose.position.y = Yatt;
      odom.pose.pose.position.z = 0.0;
      odom_quat = tf::createQuaternionMsgFromYaw(theta);
      odom.pose.pose.orientation = odom_quat;
   		odom.header.frame_id = "odom_frame";
      odom.child_frame_id="base_link";
      odom.twist.twist.linear.x =  v*cos(theta)/2;
      odom.twist.twist.linear.y = v*sin(theta)/2;
      odom.twist.twist.angular.z =omega; 
	    odom_pub.publish(odom);
      i++;

    }
		sub_Pub() {
	  		sub=n.subscribe("/speedsteer",1,&sub_Pub::callback,this);
	  		odom_pub=n.advertise<nav_msgs::Odometry>("/computed_odometry",5000);
        last_time= ros::Time::now();
        i=0;
        theta=0;
        Xatt=0;
        Yatt=0;
        Ts=0;
        omega=0;
    	}
};



int main(int argc, char *argv[])
{

	ros::init(argc, argv, "subs");
	sub_Pub obj;
    ros::spin();
  return 0;
}
