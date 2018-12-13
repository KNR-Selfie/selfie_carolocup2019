#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ctime>
#include <math.h>
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>

#define START_NUMBER 0
#define RANGE 100
#define NODES_NUMBER 10000
#define EPSILON 3.0

struct Point
{
  int x,y;
};

float dist(Point p1, Point p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
Point step_from_to(Point p1, Point p2)
{
    if(dist(p1, p2) < EPSILON)
        return p2;
    else
    {
        float theta = atan2(p2.y - p1.y, p2.x - p1.x);
        Point approx_point;
        approx_point.x = p1.x + EPSILON * cos(theta);
        approx_point.y = p1.y + EPSILON * sin(theta);
        return approx_point;
    }

}
Point get_rand_point()
{
    Point random;
    random.x =( std::rand() % RANGE ) + START_NUMBER;
    random.y =( std::rand() % RANGE ) + START_NUMBER;
    return random;
}
void draw_edge(Point pt1,Point pt2,visualization_msgs::Marker& marker)
{
    geometry_msgs::Point point;

    point.x = pt1.x;
    point.y = pt1.y;
    marker.points.push_back(point);
    point.x = pt2.x;
    point.y = pt2.y;
    marker.points.push_back(point);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv,"RRT");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    ros::Publisher rrt_pub = nh.advertise<visualization_msgs::Marker>("rrt_edges",1000);

    ros::Rate loop_rate(0.25);

    Point start;
    start.x = 0;
    start.y = 0;
    Point tmp;
    Point rand_point;
    Point new_node;
    std::vector<Point> nodes;
    srand(time(NULL));

    visualization_msgs::Marker edge_list;
    edge_list.header.frame_id = "my_frame";
    edge_list.action = visualization_msgs::Marker::ADD;
    edge_list.type = visualization_msgs::Marker::LINE_LIST;
    edge_list.scale.x = 0.1;
    edge_list.scale.y = 0.1;
    edge_list.color.g = 1.0f;
    edge_list.color.a = 1.0;
    edge_list.id = 2;


    while(ros::ok())
    {
        edge_list.points.clear();
        nodes.clear();

        //append start point
        nodes.push_back(start);

        for(int i = 0;i<NODES_NUMBER;i++)
        {
            rand_point = get_rand_point();
            tmp = nodes[0];

            //find the closest
            for(int j = 0;j<nodes.size();j++)
            {
                if(dist(nodes[j],rand_point) < dist(tmp,rand_point))
                    tmp = nodes[j];
            }
            new_node = step_from_to(tmp,rand_point);
            if(new_node.x>30 && new_node.x<60 && new_node.y>30 && new_node.y<60)
            {
                i--;
                continue;
            }
            nodes.push_back(new_node);

            draw_edge(tmp,new_node,edge_list);
            rrt_pub.publish(edge_list);
        }

    loop_rate.sleep();
    ros::spinOnce();
    }



    return 0;
}

