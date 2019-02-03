#include <std_msgs/Float64.h>


class Intersections
{
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Subscriber distance_sub;
	ros::Publisher stop_pub;
	Intersections(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
	
}