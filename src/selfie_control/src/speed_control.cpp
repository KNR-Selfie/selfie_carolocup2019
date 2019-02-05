
#include <selfie_control/speed_control.h>

Speed_controller::Speed_controller(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh),
pnh_(pnh)
{
	min_speed = 0.1;
	max_speed = 1.0;
	max_curvature = 1.0;
	speed_pub = nh_.advertise<std_msgs::Float64>("speed_control", 10);
	curv_sub = nh_.subscribe("curvature", 1, &Speed_controller::curvature_callback, this);

}
void Speed_controller::curvature_callback(const std_msgs::Float64 &msg)
{
	std_msgs::Float64 speed_msg;
	speed_msg.data = max_speed - msg.data * ((max_speed - min_speed)/max_curvature);
	if(speed_msg.data > max_speed) speed_msg.data = max_speed;
	else if(speed_msg.data < min_speed) speed_msg.data = min_speed;
	speed_pub.publish(speed_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "speed_control");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	Speed_controller speed_controller(nh, pnh);
	ros::spin();
}



