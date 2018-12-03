#pragma once

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <selfie_msgs/RoadMarkings.h>
#include <geometry_msgs/Point.h>

#define PI 3.1415926

class LaneDetector
{
  public:
	LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
	~LaneDetector();
	bool init();

  private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher lanes_pub_;

	cv::Mat kernel_v_;
	cv::Mat current_frame_;
	cv::Mat gray_frame_;
	cv::Mat binary_frame_;
	cv::Mat mask_;
	cv::Mat canny_frame_;
	cv::Mat visualization_frame_;
	cv::Mat homography_frame_;

	std::vector<std::vector<cv::Point> > lanes_vector_;
	std::vector<std::vector<cv::Point> > lanes_vector_last_frame;

	void imageCallback(const sensor_msgs::ImageConstPtr &msg);
	void openCVVisualization();
	void mergeMiddleLane();
	void quickSortLinesY(int left, int right);
	void quickSortPointsY(std::vector<cv::Point> &vector_in, int left, int right);
	float getDistance(cv::Point p1, cv::Point p2);
	void recognizeLines();
	void publishMarkings();
	void detectLines(cv::Mat &input_frame, std::vector<std::vector<cv::Point> > &output_lanes);
	void drawPoints(cv::Mat &frame);
	void homography(cv::Mat input_frame, cv::Mat &homography_frame);
	void printInfoParams();

	float binary_treshold_;
	bool mask_initialized_;
	bool visualize_;
	float max_mid_line_gap_;
	float max_mid_line_X_gap_;
};
