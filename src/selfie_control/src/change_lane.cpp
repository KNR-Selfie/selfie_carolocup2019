#include "ChangeLane.h"

//flag that sets true when we should do changelane maneuver
bool maneuver_done = true;
//variable that remember position offset from subscriber
float position_offset = 0;
//container for result for goal
selfie_control::ChangeLaneResult result;

//executable function used in Action Server for change lane
void execute(Server* as, ChangeLane *changelane);

//callback from subscriber with information about position goal
void position_offset_callback(const std_msgs::Float64::ConstPtr& msg);

//method that sends all required msg to the corresponding publischer
void send_msgs_to_publischers(ros::Publisher& target_pub, ros::Publisher& left_indicator_pub, ros::Publisher& right_indicator_pub, float target, bool left_turn_indicator, bool right_turn_indicator);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "change_lane");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	//geting paramets from node handle
	float lane_width = 1, error_margin = 0.1;
	pnh.getParam("lane_width", lane_width);
	pnh.getParam("error_margin", error_margin);
	//evaluation of parameter
	ROS_INFO("Width %f error %f", lane_width, error_margin);

	//Constructor for changelane class
	ChangeLane changelane(lane_width, error_margin);

	//subscriber creation
	ros::Subscriber position_offset_subscriber = nh.subscribe("position_offset", 10, &position_offset_callback);
	//publishers creation
	ros::Publisher target_publisher = nh.advertise<std_msgs::Float64>("target_offset", 100);
	ros::Publisher left_turn_indicator_publisher = nh.advertise<std_msgs::Bool>("left_turn_indicator", 100);
	ros::Publisher right_turn_indicator_publisher = nh.advertise<std_msgs::Bool>("right_turn_indicator", 100);

	//Action server creation
	Server server(nh, "change_lane", false);
	server.registerGoalCallback(boost::bind(&execute, &server, &changelane));
	server.start();

  ros::Rate loop_rate(50);
	bool maneuver_status = false;
	while (ros::ok())
	{
		if (maneuver_done == false)
		{
			//write new offset position to changelane class
			changelane.position_offset = position_offset;
			//process output values
			maneuver_status = changelane.process_target_position();
			//checking if maneuver is done already
			if (maneuver_status == true)
			{
				maneuver_done = true;
				result.changed_lane = true;
				server.setSucceeded(result);
			}
		}
		//sending msgs to topics
		send_msgs_to_publischers(target_publisher, left_turn_indicator_publisher, right_turn_indicator_publisher, changelane.target_position, changelane.left_turn_indicator, changelane.right_turn_indicator);
    loop_rate.sleep();
    ros::spinOnce();
	}

	return 0;

}

void execute(Server* as, ChangeLane* changelane)
{
	//get goal from simple action client
	changelane->be_on_left_lane = as->acceptNewGoal()->left_lane;
	//write new position offset to changelan class
	changelane->position_offset = position_offset;
	//chack if we should start maneuver
	bool maneuver_status = changelane->process_target_position();
	//when we are already on goal lane
	if (maneuver_status)
	{
		result.changed_lane = false;
		as->setSucceeded(result);
	}
	//when we accept change and want to start changing lane
	else
	{
		maneuver_done = false;
	}
}

//callback from subscriber with information about position goal
void position_offset_callback(const std_msgs::Float64::ConstPtr& msg)
{
	position_offset = msg->data;
}

void send_msgs_to_publischers(ros::Publisher& target_pub, ros::Publisher& left_indicator_pub, ros::Publisher& right_indicator_pub, float target_position, bool left_turn_indicator, bool right_turn_indicator)
{
	//msgs for publishers
	static std_msgs::Float64 target_msg;
	static std_msgs::Bool left_turn_indicator_msg;
	static std_msgs::Bool right_turn_indicator_msg;

	//target posititon publisching
	target_msg.data = target_position;
	target_pub.publish(target_msg);

	//indicators status publisching
	left_turn_indicator_msg.data = left_turn_indicator;
	right_turn_indicator_msg.data = right_turn_indicator;
	left_indicator_pub.publish(left_turn_indicator_msg);
	right_indicator_pub.publish(right_turn_indicator_msg);
}
