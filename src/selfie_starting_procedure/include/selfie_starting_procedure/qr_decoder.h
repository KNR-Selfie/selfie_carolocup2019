#include <cstddef>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <zbar.h>

class Qr_decoder
{

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    int qr_tresh;

    ros::Subscriber sub_image;

    zbar::ImageScanner scanner;
    cv_bridge::CvImagePtr cv_ptr;
    zbar::Image image;
    int preview_param;
    int start_counter;
    int search_flag;

public:
    Qr_decoder(const ros::NodeHandle& _pnh,const ros::NodeHandle& _nh);

    void decode( cv_bridge::CvImagePtr raw_image);
    void imageRowCallback(const sensor_msgs::Image::ConstPtr msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
    void distCallback(const std_msgs::Float32::ConstPtr msg);
    void begin_search();
    bool end_search();
};


