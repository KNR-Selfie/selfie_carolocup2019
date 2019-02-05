#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>



class Intersections
{
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Subscriber distance_sub;
	ros::Subscriber line_sub;
	ros::Publisher stop_pub;
	
	

	void line_callback(const std_msgs::Float64 &msg);
	void distance_callback(const std_msgs::Float64 &msg);
	
	void send_stop(bool a);


	float actual_distance_to_line;
	float last_line_distance;
	float last_distance;
	float actual_distance;
	float last_line_see_distance;
	ros::Time standing_start_time;
	float go_no_line_end;


	

	
	float stop_distance;
	float min_timeout;
	bool use_scan;
	float no_line_distance;

	enum State
	{
		no_intersection,
		line_ahead,
		standing,
		waiting,
		go_no_line
	} state;

public:

	Intersections(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
	void iter();
};