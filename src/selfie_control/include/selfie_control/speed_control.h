#include <std_msgs/Float64.h>
#include <ros/ros.h>

class Speed_controller
{
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Publisher speed_pub;
	ros::Subscriber curv_sub;
	double min_speed, max_speed;
	double max_curvature;

	void curvature_callback(const std_msgs::Float64 &msg);

public:
	Speed_controller(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
};