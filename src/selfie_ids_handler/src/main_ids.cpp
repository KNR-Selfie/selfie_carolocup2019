#include "ros/ros.h"
#include "ids.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "selfie_ids_handler");
  ros::NodeHandle n;
  //ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu>("imu", 100);
  
  //ros::Subscriber ackerman_subscriber = n.subscribe("drive", 1, ackermanCallback);

  
  while (ros::ok())
  {
    ROS_INFO("SPIN");
	cv::namedWindow("Raw", cv::WINDOW_NORMAL);
	//cv::imshow("Raw", binary_frame_);
	cv::waitKey(100);
    ros::spinOnce();
  }
}


