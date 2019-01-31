
#include <selfie_msgs/searchAction.h>
#include <selfie_msgs/parkingAction.h>

#include <selfie_msgs/PolygonArray.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <actionlib/client/simple_action_client.h>

#include <selfie_park/parkAction.h>
#include <geometry_msgs/Polygon.h>

enum parking_state{searching=0,  planning_failed=1, planning=2, parking=3};

class mockManager{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_park::parkAction> park_client_;
    actionlib::SimpleActionClient<selfie_msgs::searchAction> search_client_;
    ros::Publisher parking_state_pub;

    parking_state state;

    public:
    mockManager(const ros::NodeHandle &nh):
      nh_(nh), park_client_("park", true), search_client_("search", true)
    {
        parking_state_pub = nh_.advertise<std_msgs::Int16>("/parking_state", 10);
    }

    void send_goal(float dist)
    {
      selfie_msgs::searchGoal msg;
      msg.min_spot_lenght = dist;
      search_client_.sendGoal(msg);
    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "park_client");
    ros::NodeHandle nh;
    mockManager client(nh);
    ros::Duration(1).sleep();
    client.send_goal(0.6);
    std::cout<<"sent goal"<<std::endl;


    return 0;

}