 #include "ros/ros.h"
 #include "project/floatStamped.h"
 #include "project/odomCustomStamped.h"
 #include <vector>
 #include <math.h>
 #include <stdio.h> 
 #include <message_filters/subscriber.h>
 #include <message_filters/synchronizer.h>
 #include <message_filters/sync_policies/approximate_time.h>
 #include <tf/transform_broadcaster.h>
 #include <nav_msgs/Odometry.h>
 #define PI 3.14159265
 #include <dynamic_reconfigure/server.h>
 #include <project/parametersConfig.h>


class sub_Pub
{
private:
	ros::NodeHandle n;
	message_filters::Subscriber<project::floatStamped> sub1;
    message_filters::Subscriber<project::floatStamped> sub2;
    message_filters::Subscriber<project::floatStamped> sub3;
    tf::Transform transform;
    tf::TransformBroadcaster br;
    tf::Quaternion q;
	ros::Publisher odom_pub;
	project::odomCustomStamped odom;
	ros::Time last_time,current_time;
	geometry_msgs::Quaternion odom_quat;
	dynamic_reconfigure::Server<project::parametersConfig> server;
    dynamic_reconfigure::Server<project::parametersConfig>::CallbackType f;
    typedef message_filters::sync_policies::ApproximateTime<project::floatStamped, project::floatStamped,project::floatStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    float Xatt,Yatt,theta,vL,vR,steer;
    int mode;
	


public:
	void callback(const project::floatStampedConstPtr& msg1, const project::floatStampedConstPtr& msg2,const project::floatStampedConstPtr& msg3){
		vL=msg1->data;
		vR=msg2->data;
		steer=(msg3->data)/18;
		current_time = msg3->header.stamp;

	
        
        if(mode==0){
        	differential_drive(Xatt,Yatt,theta,vL,vR);
        }else if(mode==1) {
        	ackerman(Xatt,Yatt,theta,vL,vR,steer);
        }

        
        transform.setOrigin( tf::Vector3(Xatt, Yatt, 0.0) );
        q.setRPY(0, 0, theta);
		odom_quat = tf::createQuaternionMsgFromYaw(theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "car"));
		odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        if(mode==0){
        	odom.type="Diff_Drive";
        }else if(mode==1){
        	odom.type="Ackerman_model";
        }
        
        odom.pose.pose.position.x = Xatt;
        odom.pose.pose.position.y = Yatt;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
   
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = (vL+vR)*cos(theta)/2;
        odom.twist.twist.linear.y = (vL+vR)*cos(theta)/2;
        odom.twist.twist.angular.z = (vR-vL)/2;
	    odom_pub.publish(odom);
	    last_time = current_time;
    }

    void differential_drive(float& Xatt,float& Yatt,float& theta,float &vL,float &vR){
      float Ts,omega,vk;
      Ts=(current_time - last_time).toSec();
  	  vk=(vL+vR)/2;
  	  omega=(vR-vL)/2;
  	  theta= theta+(omega*Ts);
      Xatt=Xatt+(vk*Ts*cos(theta+((omega*Ts)/2)));
      Yatt=Yatt+(vk*Ts*sin(theta+((omega*Ts)/2)));
    }
    void ackerman(float &Xatt,float &Yatt,float& theta,float &vL,float &vR,float &steer){
      float Ts,omega,vk;
      float l;
      l=1.765;
      Ts=(current_time - last_time).toSec();;
  	  vk=(vL+vR)/2;
  	  omega=((vk/l)*tan(steer * PI/180));
  	  theta= theta+(omega*Ts);
      Xatt=Xatt+(vk*Ts*cos(theta+((omega*Ts)/2)));
      Yatt=Yatt+(vk*Ts*sin(theta+((omega*Ts)/2)));
    }
	void dynamic_callback(project::parametersConfig &config, uint32_t level) {
		mode = config.select_P;
		if(config.reset_odom){
		Xatt = config.x_P;
		Yatt = config.y_P;
		theta = (config.theta_P)*PI/180;
	}
}


	sub_Pub() {
	  sub1.subscribe(n,"/speedL_stamped",1);
	  sub2.subscribe(n,"/speedR_stamped",1);
	  sub3.subscribe(n,"/steer_stamped",1);
	  odom_pub=n.advertise<project::odomCustomStamped>("new_Odom",5000);
      sync.reset(new Sync(MySyncPolicy(10),sub1,sub2,sub3));
      sync->registerCallback(boost::bind(&sub_Pub::callback, this,_1, _2,_3));
	  f = boost::bind(&sub_Pub::dynamic_callback,this, _1, _2);
      server.setCallback(f);
      last_time= ros::Time::now();
    }
    
};



int main(int argc, char *argv[])
{

	ros::init(argc, argv, "subs");
	sub_Pub obj;
    ros::spin();
}



























   
