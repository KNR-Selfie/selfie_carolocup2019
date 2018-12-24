#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "selfie_frame_receiver");
  ros::NodeHandle n;
  
  ros::Subscriber image_subscriber = n.subscribe("image_ids_raw", 1, imageCallback);
  
  while (ros::ok())
  {
    ROS_INFO("SPIN");

    ros::spinOnce();
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		current_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::namedWindow("Binarization", cv::WINDOW_NORMAL);
	cv::imshow("Binarization", binary_frame_);
	cv::waitKey(1);
	
}

