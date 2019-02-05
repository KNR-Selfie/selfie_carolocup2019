#include <selfie_control/intersections.h>
Intersections::Intersections(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh),
pnh_(pnh)
{
	stop_pub = nh_.advertise<std_msgs::Bool>("stop", 10);
	pnh_.param<float>("stop_distance",stop_distance, 0.4);
	pnh_.param<float>("min_timeout", min_timeout, 3.0);
	pnh_.param<bool>("use_scan", use_scan, false);
	pnh_.param<float>("no_line_distance", no_line_distance, 2.0);
	line_sub = nh_.subscribe("intersections", 10, &Intersections::line_callback, this);
	distance_sub = nh_.subscribe("distance", 10, &Intersections::distance_callback, this);


	state = no_intersection;
}

void Intersections::line_callback(const std_msgs::Float64 &msg)
{
	std::cout<<"line_callback"<<std::endl;
	if(state==no_intersection || state == line_ahead)
	{
		last_line_distance = msg.data;
		actual_distance_to_line=msg.data;
		last_line_see_distance=actual_distance;
		state=line_ahead;
	
	}
	
}
void Intersections::distance_callback(const std_msgs::Float64 &msg)
{
	std::cout<<"distance_callback"<<std::endl;
	actual_distance = msg.data;
}
void Intersections::iter()
{

	switch(state)
	{
		case no_intersection:
		//std::cout<<"no_intersection"<<std::endl;
		send_stop(false);
		break;

		case line_ahead:
		
		send_stop(false);
		actual_distance_to_line = last_line_distance - (actual_distance - last_line_see_distance);
		std::cout<<"line_ahead "<<actual_distance_to_line<<std::endl;
		if(actual_distance_to_line < stop_distance)
		{
			
			state = standing;
			
			standing_start_time = ros::Time::now();
		}
		break;

		case standing:
		if(ros::Time::now()-standing_start_time < ros::Duration(min_timeout))
		{
			send_stop(true);
			std::cout<<"standing"<<std::endl;
		}
		else
		{
			send_stop(false);
			std::cout<<"end standing"<<std::endl;
			state = go_no_line;
			go_no_line_end = no_line_distance + actual_distance;


		}
		break;

		case go_no_line:
		std::cout<<"go_no_line"<<std::endl;
		send_stop(false);
		if(actual_distance > go_no_line_end)
		{
			state = no_intersection;
		}
		break;

	}
}
void Intersections::send_stop(bool a)
{
	std_msgs::Bool msg;
	msg.data = a;
	stop_pub.publish(msg);
}
int main(int argc, char** argv)
{
	ros::init(argc,argv, "Intersections");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	Intersections intersections(nh, pnh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		intersections.iter();
		ros::spinOnce();
		rate.sleep();

	}
	return 0;
}