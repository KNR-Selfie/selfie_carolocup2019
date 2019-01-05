
#include <ros/ros.h>
#include "../include/selfie_parking/park.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "park");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	Park park(nh, pnh);
	park.init();
	ros::Rate rate(50);
	while(ros::ok())
	{
		park.broadcast_parking_frame();
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

