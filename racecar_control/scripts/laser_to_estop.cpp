#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PointStamped.h>
#include <algorithm>
#include <vector>

ros::Publisher estop_pub;
ros::Publisher dist_pub;
double prev_range=1;
ros::Time prev_time;
double filtered_dist=1.0;
double filtered_speed=0.0;

//defined in parameters
double filter_alpha;
double max_min_dist;
double max_impact_time;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan::_ranges_type::const_iterator min_it=std::min_element(msg->ranges.begin(),msg->ranges.end());
	std::vector<float> data_dist;
	std::vector<float> data_ang;
	std::vector<sensor_msgs::LaserScan::_ranges_type::const_iterator> data_it;
	double min_dist=*min_it;
	double min_angle=msg->angle_min+std::distance(msg->ranges.begin(),min_it)*msg->angle_increment;
	double ang;
	for(int i=0;i<sizeof(msg->ranges)/sizeof(msg->ranges[0]);i++)
	{
		if((msg->ranges[i]>msg->range_min)&&(msg->ranges[i]<msg->range_max))
		{
			ang= msg->angle_min+i*msg->angle_increment;
			if (abs(min_angle-ang)<(20/180.)*3.14)
			{
				data_dist.push_back(msg->ranges[i]);
				data_ang.push_back(ang);
			}
		}
	}

	if(data_dist.size()>0)
	{
		min_dist=0;
		min_angle=0;
		min_dist=accumulate(data_dist.begin(), data_dist.end(), min_dist)/data_dist.size();
		min_angle= accumulate(data_ang.begin(), data_ang.end(), min_angle)/data_ang.size();
	}

	//ROS_INFO("closest obstacle at %f meters, in direction %f degrees",min_dist,min_angle/3.14159*180);
	if(min_dist<max_min_dist)
	{
		ROS_WARN("Emergency stop from laserdata : closest obstacle at %f meters, in direction %f degrees",min_dist,(min_angle/3.14159)*180);
	}
	double closing_speed=(min_dist-prev_range)/((msg->header.stamp - prev_time).toSec());
	//ROS_INFO("closing speed %f m/s" , closing_speed);
	filtered_dist=filter_alpha*filtered_dist+(1-filter_alpha)*min_dist;
	filtered_speed=filter_alpha*filtered_speed+(1-filter_alpha)*closing_speed;
	double impact_time=-filtered_dist/filtered_speed;
  //double impact_time=-min_dist/closing_speed;
	if(impact_time>0 & impact_time<max_impact_time)
	{
		ROS_WARN("Emergency stop from laserdata : impact time in %f seconds",impact_time);
	}
	prev_range=min_dist;
	prev_time=msg->header.stamp;
	geometry_msgs::PointStamped dist_msg;
	dist_msg.header.stamp=ros::Time(0);
	dist_msg.point.x=filtered_dist;
	dist_msg.point.y=min_angle;
	dist_pub.publish(dist_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_to_estop");
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");
  nh_rel.param("filter_alpha",filter_alpha,0.8);
  nh_rel.param("max_min_dist",max_min_dist,0.4);
  nh_rel.param("max_impact_time",max_impact_time,1.5);
  dist_pub = n.advertise<geometry_msgs::PointStamped>("/closest_obstacle", 100);
  prev_time=ros::Time::now();
  ros::Subscriber sub = n.subscribe("/scan", 100, scanCallback);
  ros::spin();
  return 0;
}
