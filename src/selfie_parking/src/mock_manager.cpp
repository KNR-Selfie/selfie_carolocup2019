
#include <selfie_msgs/searchAction.h>
#include <selfie_msgs/parkingAction.h>

#include <selfie_msgs/PolygonArray.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <actionlib/client/simple_action_client.h>

//#include <selfie_park/parkAction.h>
#include <geometry_msgs/Polygon.h>

enum parking_state{searching=0,  planning_failed=1, planning=2, parking=3};

class mockManager{
private:
    ros::NodeHandle nh_;
 //   actionlib::SimpleActionClient<selfie_park::parkAction> park_client_;
    actionlib::SimpleActionClient<selfie_msgs::searchAction> search_client_;
    ros::Publisher parking_state_pub;

    parking_state state;

    public:
    mockManager(const ros::NodeHandle &nh):
      nh_(nh), search_client_("search", true)       //park_client_("park", true), 
    {
      //  parking_state_pub = nh_.advertise<std_msgs::Int16>("/parking_state", 10);
    }

    bool search()
    {
        ROS_INFO("waiting for server to start");
        search_client_.waitForServer();
        ROS_INFO("put min lenght for a parking spot [float]: ");
        float x;
        std::cin >> x;
        ros::Duration(1).sleep();
        send_goal(x);
        ROS_INFO("goal sent");

        bool finished = search_client_.waitForResult(ros::Duration(20.0));

        if(finished)
        {
            auto result = search_client_.getResult();
            geometry_msgs::Polygon parking_place = result->parking_spot;
            std::cout << (parking_place.points.begin())->x << std::endl;
            ROS_INFO("goal_received!!!");
        }
        return finished;
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
    ros::init(argc, argv, "mock_manager");
    ros::NodeHandle nh;
    mockManager client(nh);

    if( client.search())
        ROS_WARN("place found!!");
    else
    {
        ROS_WARN("place not found in given time!");
    }
    return 0;

}