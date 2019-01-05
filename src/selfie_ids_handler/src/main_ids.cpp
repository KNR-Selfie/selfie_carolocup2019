#include "ids.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define IMSHOW_RATE 1
#define CAMERA_INDEX 0
#define CAM_RES_X IDS_WIDTH
#define CAM_RES_Y IDS_HEIGHT

#define DEBUG_MODE

int main(int argc, char **argv)
{
  ros::init(argc, argv, "selfie_ids_handler");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  image_transport::Publisher image_pub;
  image_pub = it.advertise("/image_ids", 1);
  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; 
  
  cv::Mat ids_image(CAM_RES_Y,CAM_RES_X,CV_8UC3);
#ifdef DEBUG_MODE
  cv::namedWindow("Frame",1);  
  int denom = 0;
#endif //DEBUG_MODE

  //initialization of ids
  ids.init();
  
  while(ros::ok()){
    //getting new frame from ids
    ids.get_frame_to(ids_image);
   
    //converting mat frame to msg format
    std_msgs::Header header;
    header.seq = 1; 
    header.stamp = ros::Time::now(); 
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, ids_image);
    //converting from cv_bridge to sensor_msgs::Image
    img_bridge.toImageMsg(img_msg); 

    //publishing message
    image_pub.publish(img_msg); 

#ifdef DEBUG_MODE
    //showing image from camera
    if(++denom >= IMSHOW_RATE){
      denom = 0;
      cv::imshow("0 Frame", ids_image);
      cv::waitKey(1);
    }
#endif
    ros::spinOnce();
  }
  
  ids.exit();
  return 0;
}


