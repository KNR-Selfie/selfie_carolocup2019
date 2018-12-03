#pragma once

#ifdef BOOST_UBLAS_TYPE_CHECK
#	undef BOOST_UBLAS_TYPE_CHECK
#endif
#define BOOST_UBLAS_TYPE_CHECK 0
#ifndef _USE_MATH_DEFINES
#	define _USE_MATH_DEFINES
#endif

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <vector>
#include <stdexcept>
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
  void get_row_pts(const std::vector<geometry_msgs::Point> point_vec);
  void fit_middle(poly left,poly right,int degree);
  std_msgs::Float64 get_pos_offset(float x, float y);
};


class tangent
{
    float coeff[2];
public:
    tangent(float a = 0, float b = 0);
    void calc_coeff(poly polynom,float x);
    float calc_angle(tangent tg);
    void set_coeff(float a, float b);
    std_msgs::Float64 get_head_offset(tangent tg);
};
