#include "../../selfie_parking/include/selfie_parking/simple_parking.h"

std::string goal_to_string(selfie_park::parkGoal &goal)
{
	using namespace std;
	auto point_iter = goal.parking_spot.points.begin();
	string result;
	result.append("bottom_left: ");
	result = "(x,y) = (" + to_string((*point_iter).x) + ",  " + to_string((*point_iter++).y) + " )";
	result.append("bottom_right: ");
	result = "(x,y) = (" + to_string((*point_iter).x) + ",  " + to_string((*point_iter++).y) + " )";
	result.append("top_right: ");
	result = "(x,y) = (" + to_string((*point_iter).x) + ",  " + to_string((*point_iter++).y) + " )";
	result.append("top_left: ");
	result = "(x,y) = (" + to_string((*point_iter).x) + ",  " + to_string((*point_iter++).y) + " )";

	return result;
}


void simple_parking::goalCB()
{
  selfie_park::parkGoal goal = *as_.acceptNewGoal();
	ROS_INFO("goal accepted");
	auto txt = goal_to_string(goal);
	std::cout << goal_to_string(goal);
}

void simple_parking::drive(float speed, float steering_angle, float distance)
{
	ackermann_msgs::AckermannDriveStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.drive.speed = speed;
	msg.drive.steering_angle = steering_angle;
	ackermann_pub.publish(msg);
}
void simple_parking::preemptCB()
{}

simple_parking::simple_parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
	nh_(nh),
	pnh_(pnh),
	as_(nh_, "simple_parking",  false),
	ackerman_topic_name("/drive")
{
	as_.registerGoalCallback(boost::bind(&simple_parking::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&simple_parking::preemptCB, this));
  as_.start();
  ackermann_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(ackerman_topic_name, 10);
}

simple_parking::~simple_parking()
{}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_parking");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  simple_parking park_server(nh, pnh);
  ros::spin();

	return 0;
}