/*
ONLY FOR DEMO USE
*/
#include "ros/ros.h"
#include "std_msgs/Float64.h"

float target;

void positionCallback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "demo_position");

	ros::NodeHandle pnh("~");
	ros::NodeHandle n;
	ros::Publisher offset_publisher = n.advertise<std_msgs::Float64>("position_offset", 100);
	ros::Subscriber target_subscriber = n.subscribe("target_offset", 1, positionCallback);
	std_msgs::Float64 offset_msg;
	float lane_width = 1.f;
	pnh.getParam("lane_width", lane_width);
	offset_msg.data = -lane_width / 2 - 0.01;
	while (ros::ok())
	{
		offset_msg.data += target / 10;
		if (offset_msg.data > lane_width / 2)
		{
			offset_msg.data = lane_width / 2 + 0.01;
		}
		if (offset_msg.data < -lane_width / 2)
		{
			offset_msg.data = -lane_width / 2 - 0.01;
		}

		ros::Duration(0.1).sleep();
		ROS_INFO("Jade %f", offset_msg.data);
		//publishing msg
		offset_publisher.publish(offset_msg);
		ros::spinOnce();
	}
}

void positionCallback(const std_msgs::Float64::ConstPtr& msg)
{
	target = msg->data;
}
