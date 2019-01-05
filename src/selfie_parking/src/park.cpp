#include "../include/selfie_parking/park.h"

Park::Park(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh),
pnh_(pnh),
parking_state(not_parking),
parking_speed(0.1),
move_state(init_move),
transform_listener(ros::Duration(1))
{
	ackermann_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/sim_drive", 10);
}

bool Park::init()
{
	//odom_sub = nh_.subscribe("/vesc/odom", 1, &Park::odom_callback, this);
	pnh_.param("use_scan",use_scan, true);
	pnh_.param<float>("max_distance_to_wall",max_distance_to_wall, 0.02);
	pnh_.param<bool>("always_get_wall",always_get_wall,true);
	pnh_.param<float>("scan_point_max_distance", scan_point_max_distance, 0.04);
	pnh_.param<bool>("info_msgs",state_msgs,false);
	pnh_.param<float>("earlier_turn",earlier_turn, 0.01);
	pnh_.param<float>("first_to_second_phase_x_frontwards",  first_to_second_phase_x_frontwards, 1.0/3.0);
	pnh_.param<float>("first_to_second_phase_x_backwards", first_to_second_phase_x_backwards, 1.0/2.0);
	pnh_.param<float>("minimal_start_parking_x", minimal_start_parking_x, 0);
	pnh_.param<float>("maximal_start_parking_x", maximal_start_parking_x, 0);

	scan_sub.subscribe(nh_, "obstacles", 10);
	tf_filter_scan = new tf::MessageFilter<selfie_msgs::PolygonArray>(scan_sub, transform_listener, "parking", 10);
	tf_filter_scan->registerCallback(boost::bind(&Park::scan_callback, this, _1));
	odomsub.subscribe(nh_, "/vesc/odom", 10);
	tf_filter_odom = new tf::MessageFilter<nav_msgs::Odometry>(odomsub, transform_listener, "parking", 10);
	tf_filter_odom->registerCallback(boost::bind(&Park::odom_callback, this, _1));

	parking_spot_sub = nh_.subscribe("parking_spot", 10, &Park::parking_spot_callback, this);
/*
	geometry_msgs::PolygonStamped msg;
	geometry_msgs::Point32 p;
	p.x = 3.1;
	p.y = -0.2;
	msg.polygon.points.push_back(p);
	p.x = 3.09;
	p.y = -0.5;
	msg.polygon.points.push_back(p);
	p.x = 3.6;
	p.y = -0.49;
	msg.polygon.points.push_back(p);
	p.x = 3.7;
	p.y = -0.21;
	msg.polygon.points.push_back(p);
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "map";
	initialize_parking_spot(msg);
	print_params();*/
}

void Park::parking_spot_callback(const geometry_msgs::PolygonStamped &msg)
{
	if(parking_state != not_parking)
	{
		initialize_parking_spot(msg);
		parking_state = go_to_parking_spot;
	}
}

void Park::print_params()
{
	ROS_INFO("use_scan: %s", use_scan?"true":"false");
	ROS_INFO("max_distance_to_wall: %f", max_distance_to_wall);
	ROS_INFO("always_get_wall: %s", always_get_wall?"true":"false");
	ROS_INFO("scan_point_max_distance: %f", scan_point_max_distance);
	ROS_INFO("info_msgs: %s", state_msgs?"true":"false");
	ROS_INFO("earlier_turn: %f", earlier_turn);
	ROS_INFO("first_to_second_phase_x_frontwards: %f", first_to_second_phase_x_frontwards);
	ROS_INFO("first_to_second_phase_x_backwards: %f", first_to_second_phase_x_backwards);
	ROS_INFO("minimal_start_parking_x: %f", minimal_start_parking_x);
	ROS_INFO("maximal_start_parking_x: %f", maximal_start_parking_x);

}

std::vector<geometry_msgs::PointStamped> Park::convert_polygon_to_points(const geometry_msgs::PolygonStamped &msg)
{
	std::vector<geometry_msgs::PointStamped> points_vector;
	for(std::vector<geometry_msgs::Point32>::const_iterator it = msg.polygon.points.begin(); it<msg.polygon.points.end();it++)
	{
		
		geometry_msgs::PointStamped point;
		
		point.point.x = (*it).x;
		point.point.y = (*it).y;
		point.point.z = (*it).z;
		point.header = msg.header;
		points_vector.push_back(point);
	}
	return points_vector;
}

void Park::initialize_parking_spot(const geometry_msgs::PolygonStamped &msg)
{
	
	geometry_msgs::PointStamped tl, bl, br, tr;
	std::vector<geometry_msgs::PointStamped> points = convert_polygon_to_points(msg);
	
	std::vector<geometry_msgs::PointStamped> transformed_points;
	
	tl = points[0];
	bl = points[1];
	br = points[2];
	tr = points[3];
	odom_to_parking.setOrigin(tf::Vector3(bl.point.x, bl.point.y, 0.0));
	tf::Quaternion quat;
	quat.setRPY(0,0, atan((br.point.y - bl.point.y )/( br.point.x - bl.point.x)) );
	odom_to_parking.setRotation(quat);

	ros::Time stamp = tl.header.stamp;
	transform_broadcaster.sendTransform(tf::StampedTransform(odom_to_parking,ros::Time::now(),"map","parking"));
	transform_listener.waitForTransform(msg.header.frame_id, "parking",stamp, ros::Duration(5));
	for(int i = 0;i<points.size();i++)
	{
		geometry_msgs::PointStamped p;
		transform_listener.transformPoint("parking", points[i], p);
		transformed_points.push_back(p);
	}
	tl = transformed_points[0];
	bl = transformed_points[1];
	br = transformed_points[2];
	tr = transformed_points[3];

	front_wall = tr.point.x<br.point.x?tr.point.x:br.point.x;
	back_wall = tl.point.x<bl.point.x?bl.point.x:tl.point.x;
	parking_spot_width = tl.point.y<tr.point.y?tl.point.y:tr.point.y;
	parking_spot_mid_y = parking_spot_width/2;
	
	parking_spot_mid_x = back_wall + (front_wall - back_wall)/2;
	traffic_lane_mid_y = parking_spot_width + 0.2;

	parking_spot_length = front_wall - back_wall;
	initial_front_wall = front_wall;
}

void Park::odom_callback(const boost::shared_ptr<const nav_msgs::Odometry> & msg_ptr)
{
	nav_msgs::Odometry msg = *msg_ptr;
	get_position(msg);
	switch(parking_state)
	{
		case not_parking:
		if(state_msgs) ROS_INFO("not_parking");
		break;
		
		case init_parking_spot:
		parking_state = go_to_parking_spot;
		if(state_msgs) ROS_INFO("init_parking_spot");
		drive(0,0);
		break;

		case go_to_parking_spot:
		if(to_parking_spot()){parking_state = going_in;}
		if(state_msgs) ROS_INFO("go_to_parking_spot");
		break;

		case going_in:
		if(state_msgs) ROS_INFO("get_in");
		if(get_in()) parking_state = parked;
		break;

		case parked:
		if(state_msgs) ROS_INFO("PARKED");
		//blink
		parking_state = going_out;
		break;

		case going_out:
		if(state_msgs) ROS_INFO("get_out");
		if(get_out()) parking_state = out;
		break;

		case out:
		if(state_msgs) ROS_INFO("OUT");
		break;
	}
}

void Park::scan_callback(const boost::shared_ptr<const selfie_msgs::PolygonArray> & msg_ptr)
{
	selfie_msgs::PolygonArray msg = *msg_ptr;
	if(use_scan && parking_state > init_parking_spot)
	{
		std::vector<geometry_msgs::PointStamped> points;
		for(std::vector<geometry_msgs::Polygon>::const_iterator itpoly = msg.polygons.begin();itpoly < msg.polygons.end();itpoly++)
		{
			for(std::vector<geometry_msgs::Point32>::const_iterator itpoint = (*itpoly).points.begin();itpoint < (*itpoly).points.end();itpoint++)
			{
				geometry_msgs::PointStamped transformed_point, tmp_point;
				tmp_point.header = msg.header;
				tmp_point.point.x = (*itpoint).x;
				tmp_point.point.y = (*itpoint).y;
				tmp_point.point.z = (*itpoint).z;
				transform_listener.transformPoint("parking", tmp_point, transformed_point);
				points.push_back(transformed_point);
			}
		}
		std::vector<geometry_msgs::PointStamped> front_wall_points;
		float minimal_x;
		for(std::vector<geometry_msgs::PointStamped>::const_iterator itwall = points.begin();itwall < points.end();itwall++)
		{
			if(is_point_good(*itwall))
			{
				front_wall_points.push_back(*itwall);
			}
		}
		if(front_wall_points.size() < 1) return;
		else
		{
			for(std::vector<geometry_msgs::PointStamped>::const_iterator itwp = front_wall_points.begin(); itwp< front_wall_points.end();itwp++)
			{
				if(itwp == front_wall_points.begin())
				{
					minimal_x = (*itwp).point.x;
				}
				else
				{
					if((*itwp).point.x < minimal_x)
					{
						minimal_x = (*itwp).point.x;
					}
				}
			}
			front_wall = minimal_x;
			back_wall = minimal_x - parking_spot_length;
		}
	}
}

bool Park::is_point_good(const geometry_msgs::PointStamped &pt)
{
	if(always_get_wall)
	{
		return (pt.point.x > parking_spot_mid_x) && (pt.point.y > 0) &&  (pt.point.y < parking_spot_width);
	}
	else
	{
		return (pt.point.x > (initial_front_wall - scan_point_max_distance)) && (pt.point.x < (initial_front_wall + scan_point_max_distance) )&& (pt.point.y > 0) && (pt.point.y < parking_spot_width);
	}
}

void Park::get_position(const nav_msgs::Odometry &msg)
{
	geometry_msgs::PoseStamped p;
	p.pose = msg.pose.pose;
	p.header = msg.header;
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
	return front_wall - actual_front_pose.pose.position.x - sin(actual_yaw)*CAR_WIDTH/2;
}

float Park::back_distance()
{
	return actual_back_pose.pose.position.x - back_wall - sin(actual_yaw)*CAR_WIDTH/2;
}

bool Park::get_in()
{
	switch(move_state)
	{
		case init_move:
		if(state_msgs)ROS_INFO("init_move");
		front = front_distance() > back_distance();
		right = actual_pose.pose.position.y > parking_spot_mid_y;
		move_state = get_straigth;

		case get_straigth:
		if(state_msgs) ROS_INFO("get_straigth");
		if(front && right)
		{
			if(actual_yaw > 0) drive(parking_speed, -MAX_TURN);
			else move_state = get_middles;
			
		}
		else if(!front && right)
		{
			if(actual_yaw < 0) drive(-parking_speed, -MAX_TURN);
			else move_state = get_middles;
			
		}
		if(front && !right)
		{
			if(actual_yaw < 0) drive(parking_speed, MAX_TURN);
			else move_state = get_middles;			
		}
		else if(!front && !right)
		{
			if(actual_yaw > 0) drive(-parking_speed, MAX_TURN);
			else move_state = get_middles;
			
		}
		break;

		case get_middles:
		if(state_msgs) ROS_INFO("get_middles");
		mid_x = (front?actual_front_pose.pose.position.x  + (front_wall - max_distance_to_wall  - actual_front_pose.pose.position.x)*first_to_second_phase_x_frontwards:  back_wall + (actual_back_pose.pose.position.x + max_distance_to_wall - back_wall)*first_to_second_phase_x_backwards);
		mid_y = parking_spot_mid_y + (actual_back_pose.pose.position.y - parking_spot_mid_y)/2;
		move_state = first_phase;
		

		
		case first_phase:
		if(state_msgs) ROS_INFO("1st phase");
		
		drive(front?parking_speed:-parking_speed, right?-MAX_TURN:MAX_TURN);
		if(front && right)
		{
			if(actual_front_pose.pose.position.x > mid_x - earlier_turn || actual_pose.pose.position.y < mid_y) move_state = second_phase;
			
		}
		else if(!front && right)
		{
			if(actual_back_pose.pose.position.x < mid_x + earlier_turn || actual_pose.pose.position.y < mid_y) move_state = second_phase;
			
		}
		if(front && !right)
		{
			if(actual_front_pose.pose.position.x > mid_x  - earlier_turn|| actual_pose.pose.position.y > mid_y) move_state = second_phase;
			
		}
		else if(!front && !right)
		{
			if(actual_back_pose.pose.position.x < mid_x + earlier_turn || actual_pose.pose.position.y > mid_y) move_state = second_phase;
			
		}
		break;
		case second_phase:
		if(state_msgs) ROS_INFO("2nd phase");
		drive(front?parking_speed:-parking_speed, right?MAX_TURN:-MAX_TURN);
		if(front && right)
		{
			if(actual_yaw > 0 || front_distance() < max_distance_to_wall) move_state = end;
		}
		else if(!front && right)
		{
			if(actual_yaw < 0 || back_distance() < max_distance_to_wall) move_state = end;
		}
		if(front && !right)
		{
			if(actual_yaw < 0 || front_distance() < max_distance_to_wall) move_state = end;
		}
		else if(!front && !right)
		{
			if(actual_yaw > 0 || back_distance() < max_distance_to_wall) move_state = end;
		}
		break;
		case end:
		if(state_msgs) ROS_INFO("end");
		drive(0,0);
			if(in_parking_spot())
			{
				move_state = init_move;
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

bool Park::get_out()
{

	switch(move_state)
	{
		case init_move:
		front = front_distance() > back_distance();
		right = actual_pose.pose.position.y > traffic_lane_mid_y;
		move_state = get_straigth;

		case get_straigth:
		if(state_msgs) ROS_INFO("get_straigth");
		if(front && right)
		{
			if(actual_yaw > 0) drive(parking_speed, -MAX_TURN);
			else move_state = get_middles;
			
		}
		else if(!front && right)
		{
			if(actual_yaw < 0) drive(-parking_speed, -MAX_TURN);
			else move_state = get_middles;
			
		}
		if(front && !right)
		{
			if(actual_yaw < 0) drive(parking_speed, MAX_TURN);
			else move_state = get_middles;			
		}
		else if(!front && !right)
		{
			if(actual_yaw > 0) drive(-parking_speed, MAX_TURN);
			else move_state = get_middles;
			
		}
		break;

		case get_middles:
		right = actual_pose.pose.position.y > traffic_lane_mid_y;
		mid_x = (front?actual_front_pose.pose.position.x  + (front_wall - max_distance_to_wall  - actual_front_pose.pose.position.x)*first_to_second_phase_x_frontwards: back_wall + (actual_back_pose.pose.position.x + max_distance_to_wall - back_wall)*first_to_second_phase_x_backwards);		
		mid_y = traffic_lane_mid_y + (actual_back_pose.pose.position.y - traffic_lane_mid_y)/2;
		move_state = first_phase;

		case first_phase:

		drive(front?parking_speed:-parking_speed, right?-MAX_TURN:MAX_TURN);
		if(front && right)
		{
			if(actual_front_pose.pose.position.x > mid_x - earlier_turn || actual_pose.pose.position.y < mid_y) move_state = second_phase;
			
		}
		else if(!front && right)
		{
			if(actual_back_pose.pose.position.x < mid_x + earlier_turn || actual_pose.pose.position.y < mid_y) move_state = second_phase;
			
		}
		if(front && !right)
		{
			if(actual_front_pose.pose.position.x > mid_x - earlier_turn || actual_pose.pose.position.y > mid_y) move_state = second_phase;
			
		}
		else if(!front && !right)
		{
			if(actual_back_pose.pose.position.x < mid_x + earlier_turn || actual_pose.pose.position.y > mid_y) move_state = second_phase;
			
		}
		break;

		case second_phase:;
		drive(front?parking_speed:-parking_speed, right?MAX_TURN:-MAX_TURN);
		if(front && right)
		{
			if(actual_yaw > 0 || front_distance() < max_distance_to_wall) move_state = end;
		}
		else if(!front && right)
		{
			if(actual_yaw < 0 || back_distance() < max_distance_to_wall) move_state = end;
		}
		if(front && !right)
		{
			if(actual_yaw < 0 || front_distance() < max_distance_to_wall) move_state = end;
		}
		else if(!front && !right)
		{
			if(actual_yaw > 0 || back_distance() < max_distance_to_wall) move_state = end;
		}
		break;
		case end:
		drive(0,0);
			if(in_traffic_lane())
			{
				move_state = init_move;
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

bool Park::to_parking_spot()
{
	if(actual_pose.pose.position.x < back_wall + minimal_start_parking_x)
	{
		drive(parking_speed, 0);
		return false;
	}
	else if(actual_pose.pose.position.x > front_wall + maximal_start_parking_x)
	{
		drive(-parking_speed, 0);
		return false;
	}
	else return true;
}

bool Park::in_parking_spot()
{
	
	float l = cos(actual_yaw)*CAR_WIDTH/2;
	bool is_in = parking_spot_width > actual_back_pose.pose.position.y + l && actual_back_pose.pose.position.y - l > 0 && parking_spot_width > actual_front_pose.pose.position.y + l && actual_front_pose.pose.position.y - l > 0;
	return is_in;
}

bool Park::in_traffic_lane()
{
	
	float l = cos(actual_yaw)*CAR_WIDTH/2;
	return traffic_lane_mid_y + MARGIN > actual_pose.pose.position.y && actual_pose.pose.position.y > traffic_lane_mid_y - MARGIN;
	
}

void Park::broadcast_parking_frame()
{
	transform_broadcaster.sendTransform(tf::StampedTransform(odom_to_parking, ros::Time::now(),"map","parking"));

}