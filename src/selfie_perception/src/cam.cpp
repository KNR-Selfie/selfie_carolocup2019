#include <selfie_perception/lanedetector.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_markings");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LaneDetector laneDetector(nh, pnh);
    if (laneDetector.init())
    {
        ROS_INFO("detect_markings initialized");
    }
    else
    {
        ROS_INFO("detect_markings doesn't work");
        return 1;
    }

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}