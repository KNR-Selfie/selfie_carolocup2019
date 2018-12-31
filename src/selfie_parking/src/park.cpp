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
	odom_sub = nh_.subscribe("/vesc/odom", 1, &Park::odom_callback, this);
	scan_sub = nh_.subscribe("/obstacles", 1, &Park::scan_callback, this);
	pnh_.param("use_scan",use_scan, false);
	pnh_.param<float>("max_distance_to_wall",max_distance_to_wall, 0.04);
	pnh_.param<bool>("always_get_wall",always_get_wall,true);
	pnh_.param<float>("scan_point_max_distance", scan_point_max_distance, 0.04);
	pnh_.param<bool>("info_msgs",state_msgs, false);
	pnh_.param<float>("earlier_turn",earlier_turn, 0.1);






	geometry_msgs::PolygonStamped msg;
	geometry_msgs::Point32 p;
	p.x = 3.1;
	p.y = -0.2;
	msg.polygon.points.push_back(p);
	p.x = 3.09;
	p.y = -0.5;
	msg.polygon.points.push_back(p);
	p.x = 3.9;
	p.y = -0.49;
	msg.polygon.points.push_back(p);
	p.x = 3.89;
	p.y = -0.21;
	msg.polygon.points.push_back(p);
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "map";

	
	initialize_parking_spot(msg);
	//parking_spot_sub = nh_.subscribe("/parking_spot", 10, &Park::parking_spot_callback, this);
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

geometry_msgs::PolygonStamped Park::convert_polygon_to_polygon(const geometry_msgs::PolygonStamped &msg)
{
	std::vector<geometry_msgs::PointStamped> points = convert_polygon_to_points(msg);
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
		ROS_INFO("%f %f ",transformed_points[i].point.x, transformed_points[i].point.y);
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

void Park::odom_callback(const nav_msgs::Odometry &msg)
{
	transform_broadcaster.sendTransform(tf::StampedTransform(odom_to_parking, ros::Time::now(),"map","parking"));
	get_position(msg);
	switch(parking_state)
	{
		case not_parking:
		if(state_msgs) ROS_INFO("not_parking");
		if(actual_front_pose.pose.position.x < 0.1) drive(0.2, 0);
		else parking_state = init_parking_spot;
		break;
		
		case init_parking_spot:
		parking_state = going_in;
		if(state_msgs) ROS_INFO("init_parking_spot");
		drive(0,0);
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

void Park::scan_callback(const selfie_msgs::PolygonArray &msg)
{

	if(use_scan && parking_state > init_parking_spot)
	{
		std::vector<geometry_msgs::PointStamped> points;
		for(std::vector<geometry_msgs::Polygon>::const_iterator itpoly = msg.polygons.begin();itpoly < msg.polygons.end();itpoly++)
		{
			for(std::vector<geometry_msgs::Point32>::const_iterator itpoint = (*itpoly).points.begin();itpoint < (*itpoly).points.end();itpoint++)
			{
				geometry_msgs::PointStamped transformed_point, tmp_point;
				tmp_point.header = msg.header;
				tmp_point.header.stamp = ros::Time(0);
				tmp_point.point.x = (*itpoint).x;
				tmp_point.point.y = (*itpoint).y;
				tmp_point.point.z = (*itpoint).z;
				
				transform_listener.transformPoint("map", tmp_point, transformed_point);
				transformed_point.header.stamp = ros::Time(0);
				transform_listener.transformPoint("parking", transformed_point, transformed_point);
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
			
			ROS_INFO("%f %f", back_wall, front_wall);
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

bool Park::get_in()
{

	switch(move_state)
	{
		case init_move:
		if(state_msgs)ROS_INFO("init_move");
		front = front_distance() > back_distance();
		right = actual_pose.pose.position.y > parking_spot_mid_y;
		mid_x = (front?actual_front_pose.pose.position.x + (front_wall - actual_front_pose.pose.position.x) / 2:back_wall + (actual_back_pose.pose.position.x - back_wall)/2);
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
		mid_x = (front?actual_front_pose.pose.position.x + (front_wall - actual_front_pose.pose.position.x) / 2:back_wall + (actual_back_pose.pose.position.x - back_wall)/2);
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
		case second_phase:
		//ROS_INFO("%f %f",front_distance(),back_distance());
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

bool Park::in_parking_spot()
{
	ROS_INFO(" pos %f", actual_pose.pose.position.y);
	return parking_spot_mid_y + MARIGIN>actual_pose.pose.position.y && actual_pose.pose.position.y>parking_spot_mid_y - MARIGIN;
}

bool Park::in_traffic_lane()
{
	ROS_INFO(" pos %f", actual_pose.pose.position.y);
	return traffic_lane_mid_y + MARIGIN>actual_pose.pose.position.y && actual_pose.pose.position.y>traffic_lane_mid_y - MARIGIN;
	
}