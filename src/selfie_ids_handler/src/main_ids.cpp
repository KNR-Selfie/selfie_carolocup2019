#include "ros/ros.h"
//#include "ids.h"

#define IMSHOW_RATE 1
#define CAMERA_INDEX 0
#define CAM_RES_X IDS_WIDTH
#define CAM_RES_Y IDS_HEIGHT

//std::mutex mu;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "selfie_ids_handler");
  ros::NodeHandle n;
  
  //cv::Mat ids_image(CAM_RES_Y,CAM_RES_X,CV_8UC3);
  
  static int denom = 0;
  //struct timespec start;
  //unsigned int licznik_czas = 0;
  //float fps = 0;
  //cv::namedWindow("0 Frame",1);  
  ROS_INFO("Created window");
  //ids.init();
  
    while(ros::ok()){
      
      //ids.get_frame_to(ids_image);
      //ROS_INFO("cos");
      if(++denom >= IMSHOW_RATE){
        denom = 0;
        //cv::imshow("0 Frame", ids_image);
        //cv::waitKey(10);
      }
      ros::spinOnce();
    }
 return 0;
}


