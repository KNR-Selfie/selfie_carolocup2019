#include "change_lane_logic.h"


void ChangeLaneLogic::check_lane_status(geometry_msgs::Polygon polygon){
     
    
    uint8_t left_points_tmp = 0;
    uint8_t right_points_tmp = 0;

    for (int i=0; i<4; ++i){
        if (car_lane == c_left && check_point(polygon.points[i].x+0.2,-polygon.points[i].y, line_coef.left[2],line_coef.left[1],line_coef.left[0], line_coef.center[2],line_coef.center[1],line_coef.center[0]))
            left_points_tmp++;
        if (car_lane == c_right && check_point(polygon.points[i].x+0.2,-polygon.points[i].y, line_coef.center[2],line_coef.center[1],line_coef.center[0], line_coef.right[2],line_coef.right[1],line_coef.right[0]))
            right_points_tmp++;
            
    }
    if (left_points_tmp > 1){
        left_points++;
    }
    if (right_points_tmp > 1){
        right_points++;
    }

}

uint8_t ChangeLaneLogic::check_point(float x, float y, float a0_l, float a1_l, float a2_l, float a0_r, float a1_r, float a2_r){
    ROS_INFO("x: %f y: %f", x,y);
    float xr = a2_r*y*y+a1_r*y+a0_r;
    float xl = a2_l*y*y+a1_l*y+a0_l;
    if (xr<x && xl>x){
        return 1;
    }
    return 0;


}


