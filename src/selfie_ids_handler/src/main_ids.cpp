#include "ros/ros.h"
#include "ids.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



#define IMSHOW_RATE 1
#define CAMERA_INDEX 0
#define CAM_RES_X IDS_WIDTH
#define CAM_RES_Y IDS_HEIGHT

int main(int argc, char **argv)
{
  ros::init(argc, argv, "selfie_ids_handler");
  ros::NodeHandle n;
  
  image_transport::ImageTransport it(n);
  
  cv::Mat ids_image(CAM_RES_Y,CAM_RES_X,CV_8UC3);
  
  int denom = 0;

  image_transport::Publisher image_pub_;
  image_pub_ = it.advertise("/image_ids", 1);
  cv_bridge::CvImagePtr cv_ptr;

  cv::namedWindow("Frame",1);  
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; // >> message to be sent

  //initialization of ids
  ids.init();
  
    while(ros::ok()){
      
      //getting new frame from ids
      ids.get_frame_to(ids_image);


      

      std_msgs::Header header; // empty header
      header.seq = 1; // user defined counter
      header.stamp = ros::Time::now(); // time
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, ids_image);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      image_pub_.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);


      //showing image from camera
      if(++denom >= IMSHOW_RATE){
        denom = 0;
        cv::imshow("0 Frame", ids_image);
        cv::waitKey(1);
      }
      ros::spinOnce();
    }
  
  ids.exit();
 return 0;
}


