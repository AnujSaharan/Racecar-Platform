#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

ros::Publisher ackermann_pub;

double delta=0;
double throttle=0;
double throttle_d=0;
double delta_max=0;

float sat(float val,float val_max){
	if(val>val_max)
		return val_max;
	if(val<-val_max)
		return -val_max;
	return val;
}

void throttleCallback(const std_msgs::Float64::ConstPtr& msg)
{
	throttle_d=sat(msg->data,1);
}

void steeringCallback(const std_msgs::Float64::ConstPtr& msg)
{
	delta=delta_max*sat(msg->data,1);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_command_to_simu");
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");

  ackermann_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/ackermann_cmd_mux/input/teleop", 10);

  ros::Subscriber throttle_sub = n.subscribe("/throttle/command", 10, throttleCallback);
  ros::Subscriber steering_sub = n.subscribe("/steering/command", 10, steeringCallback);

  //getting params
  double frequency,alpha,linear_decay,v_max;
  nh_rel.param("frequency",frequency,100.0);
  nh_rel.param("alpha",alpha,0.2);
  nh_rel.param("linear_decay",linear_decay,0.5);
  nh_rel.param("delta_max",delta_max,0.7);
  nh_rel.param("v_max",v_max,3.0);

  ros::Rate rate(frequency);
  ackermann_msgs::AckermannDriveStamped msg;

  bool positive_velocity=true;
  while (n.ok())
  {
  	if(positive_velocity && throttle_d>0.5)
  	{
  		throttle+=alpha/frequency*(throttle_d-0.5-throttle);
    }
    else
    {
    	if(!positive_velocity && throttle_d<-0.5)
    	{
    		throttle+=alpha/frequency*(throttle_d+0.5-throttle);
    	}
    	else
    	{
    		if(fabs(throttle)>0.05)
    		{
    			int sign=(throttle>0?1:-1);
    			throttle-=sign*linear_decay/frequency/v_max;
    			if(sign*throttle<0)
    			{
    				throttle=0;
    			}
    		}
    		else
    		{
    			if(throttle_d>0.5)
    			{
    				positive_velocity=true;
    				throttle+=alpha/frequency*(throttle_d-0.5-throttle);
    			}
    			else
    			{
    				if(throttle_d<-0.5)
    				{
    					positive_velocity=false;
    					throttle+=alpha/frequency*(throttle_d+0.5-throttle);
    				}
    				else
    				{
    					throttle=0;
					}
				}
			}
		}
    }
    /*if(fabs(throttle_d)>0.5 || fabs(throttle)<0.05 ){
	throttle+=alpha/frequency*(throttle_d-(throttle_d==0?0:(throttle_d>0?1:-1))*0.5-throttle);
      }else{
	int sign=(throttle>0?1:-1);
	throttle-=sign*linear_decay/frequency/v_max;
	if(sign*throttle<0){
		throttle=0;
	}
    }*/
	msg.header.frame_id="base_link";
    msg.drive.steering_angle=delta;
    msg.drive.speed=2*v_max*throttle;
    msg.header.stamp=ros::Time::now();
    ackermann_pub.publish(msg);
    rate.sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}
