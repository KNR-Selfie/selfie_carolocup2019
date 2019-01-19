#pragma once

#ifdef BOOST_UBLAS_TYPE_CHECK
#	undef BOOST_UBLAS_TYPE_CHECK
#endif
#define BOOST_UBLAS_TYPE_CHECK 0
#ifndef _USE_MATH_DEFINES
#	define _USE_MATH_DEFINES
#endif
#define MAT_HEIGHT 480
#define MAT_WIDTH 640

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <vector>
#include <stdexcept>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"

class poly
{
public:
  std::vector<float> x_raw_pts;
  std::vector<float> y_raw_pts;
  std::vector<float> coeff;
  std::vector<float> y_output_pts;

  poly();
  void polyfit(int nDegree );
  void polyval();
  float polyval(float x);
  void adjust(poly good_poly, int avg_points);

  void get_row_pts(const std::vector<cv::Point> point_vec);
  void fit_middle(poly left,poly right,int degree);
  std_msgs::Float64 get_pos_offset(float x, float y);
};