#include "../include/selfie_parking/park.h"


Park::Park(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh),
pnh_(pnh),
parking_state(not_parking),
parking_speed(0.1)
{
	ackermann_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/sim_drive", 10);
}

bool Park::init()
{
	odom_sub = nh_.subscribe("/vesc/odom", 1, &Park::odom_callback, this);
	scan_sub = nh_.subscribe("/obstacles", 1, &Park::scan_callback, this);
	initialize_parking_spot();
	
	//parking_spot_sub = nh_.subscribe("/parking_spot", 10, &Park::parking_spot_callback, this);
}



std::vector<geometry_msgs::PointStamped> Park::convert_polygon_to_points(const std::string &output_frame, const geometry_msgs::PolygonStamped &msg)
{
	std::vector<geometry_msgs::PointStamped> points_vector;
	for(std::vector<geometry_msgs::Point32>::const_iterator it = msg.polygon.points.begin(); it<msg.polygon.points.end();it++)
	{
		
		geometry_msgs::PointStamped to_convert_point;
		geometry_msgs::PointStamped converted_point;
		to_convert_point.point.x = (*it).x;
		to_convert_point.point.y = (*it).y;
		to_convert_point.point.z = (*it).z;
		to_convert_point.header = msg.header;
		transform_listener.transformPoint(output_frame, to_convert_point, converted_point);
		points_vector.push_back(converted_point);

	}
	return points_vector;
}

geometry_msgs::PolygonStamped Park::convert_polygon_to_polygon(const std::string &output_frame, const geometry_msgs::PolygonStamped &msg)
{
	std::vector<geometry_msgs::PointStamped> points = convert_polygon_to_points(output_frame, msg);
	geometry_msgs::PolygonStamped polygon;

	for(std::vector<geometry_msgs::PointStamped>::const_iterator it  = points.begin();it<points.end();it++)
	{
		geometry_msgs::Point32 point;
		point.x = (*it).point.x;
		point.y = (*it).point.y;
		point.z = (*it).point.z;
		polygon.polygon.points.push_back(point);
	}
	if(points.size()>0)
	{
		polygon.header = points[0].header;
	}
	return polygon;

}
void Park::initialize_parking_spot()
{
	odom_to_parking.setOrigin(tf::Vector3(3.05, -0.5, 0.0));
	tf::Quaternion quat;
	quat.setRPY(0,0,0);
	odom_to_parking.setRotation(quat);
	front_wall = 0.9;
	back_wall = 0;
	parking_spot_mid_y = 0.15;
	traffic_lane_mid_y = 0.5;
	transform_broadcaster.sendTransform(tf::StampedTransform(odom_to_parking, ros::Time::now(),"map","parking"));
}

void Park::odom_callback(const nav_msgs::Odometry &msg)
{
	get_position(msg);
	if(parking_state == not_parking)
	{
		
		
		if(actual_front_pose.pose.position.x < 0.9)
		{
			drive(0.3, 0);
		}
		else
		{
			parking_state = init_parking_spot;
		}
	}
	else if(parking_state == init_parking_spot)
	{

		parking_state = going_in;
		ROS_INFO("init_parking_spot");
		drive(0,0);
	}
	else if(parking_state == going_in)
	{
		if(get_in())
		{
			ROS_INFO("IN PARKING_SPOT");
		}
	}

}

void Park::scan_callback(const selfie_msgs::PolygonArray &msg)
{}
void Park::get_position(const nav_msgs::Odometry &msg)
{
	geometry_msgs::PoseStamped p;
	p.pose = msg.pose.pose;
	p.header = msg.header;
	p.header.stamp = ros::Time(0);
	transform_listener.transformPose("parking", p, actual_pose);
	
	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(actual_pose.pose.orientation,q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	actual_yaw = static_cast<float>(yaw);
	actual_back_pose = actual_pose;
	actual_front_pose = actual_pose;
	actual_back_pose.pose.position.x-=cos(actual_yaw)*std::abs(ODOM_TO_BACK);
	actual_back_pose.pose.position.y-=sin(actual_yaw)*std::abs(ODOM_TO_BACK);
	actual_front_pose.pose.position.x+=cos(actual_yaw)*std::abs(ODOM_TO_FRONT);
	actual_front_pose.pose.position.y+=sin(actual_yaw)*std::abs(ODOM_TO_FRONT);
	ROS_INFO("actual_x  %f actual_y %f actual_back_x %f actual_back_y %f actual_front_x %f actual_front_y %f actual_yaw %f", actual_pose.pose.position.x, actual_pose.pose.position.y,actual_back_pose.pose.position.x, actual_back_pose.pose.position.y,actual_front_pose.pose.position.x, actual_front_pose.pose.position.y,actual_yaw);
}
void Park::drive(float speed, float steering_angle)
{
	ackermann_msgs::AckermannDriveStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.drive.speed = speed;
	msg.drive.steering_angle = steering_angle;
	ackermann_pub.publish(msg);
}

float Park::front_distance()
{
	return front_wall - actual_front_pose.pose.position.x;
}
float Park::back_distance()
{
	return actual_back_pose.pose.position.x - back_wall;
}

void Park::get_in_front()
{
	switch(move_state)
	{
		case init_move:
		mid_x = actual_front_pose.pose.position.x + (front_wall - actual_front_pose.pose.position.x) / 2;
		mid_y = parking_spot_mid_y + (actual_front_pose.pose.position.y - parking_spot_mid_y) / 2;
		move_state = first_phase;
		case first_phase:
		drive(parking_speed, -MAX_TURN);
		if(actual_front_pose.pose.position.x > mid_x || actual_pose.pose.position.y < mid_y)
		{
			move_state = second_phase;
		}
		break;
		case second_phase:
		drive(parking_speed, MAX_TURN);
		if(actual_yaw > 0 || front_distance() < 0.02)
		{
			move_state = end;
		}
		break;
		case end:
		drive(0,0);
		ROS_INFO("END");
		break;
	}
}

bool Park::get_in()
{
	switch(move_state)
	{
		case init_move:
		front = front_distance() > back_distance();
		mid_x = (front?actual_front_pose.pose.position.x + (front_wall - actual_front_pose.pose.position.x) / 2:back_wall + (actual_back_pose.pose.position.x - back_wall)/2);
		mid_y = parking_spot_mid_y + (actual_back_pose.pose.position.y - parking_spot_mid_y)/2;
		move_state = first_phase;
		case first_phase:
		drive(front?parking_speed:-parking_speed, -MAX_TURN);
		if(front)
		{
			if(actual_front_pose.pose.position.x > mid_x || actual_pose.pose.position.y < mid_y)
			{
				move_state = second_phase;
			}
		}
		else
		{
			if(actual_back_pose.pose.position.x < mid_x || actual_pose.pose.position.y < mid_y)
			{
				move_state = second_phase;
			}
		}
		break;
		case second_phase:
		drive(front?parking_speed:-parking_speed, MAX_TURN);
		if(front)
		{
			if(actual_yaw > 0 || front_distance() < 0.02)
			{
				move_state = end;
			}
		}
		else
		{
			if(actual_yaw < 0 || back_distance() < 0.02)
			{
				move_state = end;
			}
		}
		break;
		case end:
		drive(0,0);
			if(in_parking_spot())
			{
				return true;
			}
			else
			{
				move_state = init_move;
			}
		break;
	
	}
	return false;
}
bool Park::in_parking_spot()
{
	return parking_spot_mid_y + MARIGIN>actual_pose.pose.position.y>parking_spot_mid_y - MARIGIN;
}