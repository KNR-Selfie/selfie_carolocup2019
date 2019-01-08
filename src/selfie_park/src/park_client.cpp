#include <../include/selfie_park/park_client.h>

ParkClient::ParkClient(const ros::NodeHandle &nh):
nh_(nh),
ac_("park", true)
{}

void ParkClient::send_goal()
{
    selfie_park::parkGoal msg;
    geometry_msgs::Point32 p;
    p.x = 3.1;
    p.y = -0.2;
    msg.parking_spot.points.push_back(p);
    p.x = 3.1;
    p.y = -0.5;
    msg.parking_spot.points.push_back(p);
    p.x = 3.6;
    p.y = -0.5;
    msg.parking_spot.points.push_back(p);
    p.x = 3.6;
    p.y = -0.2;
    msg.parking_spot.points.push_back(p);
    msg.park = true;
    ac_.sendGoal(msg);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "park_client");
    ros::NodeHandle nh;
    ParkClient client(nh);
    ros::Duration(1).sleep();
    client.send_goal();
    std::cout<<"sent goal"<<std::endl;


    return 0;

}