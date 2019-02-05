#!/usr/bin/env python

import rospy

import numpy as np
from scipy.optimize import minimize
from numpy.polynomial.polynomial import Polynomial

from selfie_msgs.msg import RoadMarkings
from std_msgs.msg import Float64, Float32, UInt16, Bool
from selfie_msgs.msg import PolygonArray
from ChangeLaneClass import ChangeLaneClass
from geometry_msgs.msg import Polygon, PolygonStamped

CLC= ChangeLaneClass()


def road_markings_callback(msg):
  #save parameters for each line
  CLC.c_poly = Polynomial(msg.center_line)
  CLC.r_poly = Polynomial(msg.right_line)
  CLC.l_poly = Polynomial(msg.left_line)

def distance_callback(msg):
  #save distance from stm32
  CLC.distance = msg.data
  #showing variables on screen
  #rospy.loginfo("Points: %d \t Lane: %d", CLC.points_on_lane, CLC.right_lane)
  
def obstacles_callback(msg):
  if (CLC.get_call ==0):
    CLC.polygons[:] = []
    CLC.get_call = 1
    for box_nr in range (len(msg.polygons)-1, 0, -1):    
        CLC.polygons.append(msg.polygons[box_nr])

def stop_callback(msg):
  CLC.stop_call = msg.data



    
if __name__ == '__main__':
    rospy.init_node('change_lane_logic')

    road_markings_sub = rospy.Subscriber('road_markings', RoadMarkings, road_markings_callback, queue_size=1)
    obstacles_sub = rospy.Subscriber('obstacles', PolygonArray, obstacles_callback, queue_size=1)
    distance_sub = rospy.Subscriber('distance', Float32, distance_callback, queue_size=1)
    stop_sub = rospy.Subscriber('stop', Bool, stop_callback, queue_size=1)
    
  

    change_lane_pub = rospy.Publisher('box_right', UInt16, queue_size=1)
   
    CLC.border_distance_x = rospy.get_param('~border_x', 0.8)
    CLC.border_distance_y = rospy.get_param('~border_y', 0.5)
    CLC.fraction = rospy.get_param('~fraction', 0.3)
    CLC.threshold_normal = rospy.get_param('~thresh_normal', 2)
    CLC.threshold_anormal = rospy.get_param('~thresh_anormal', 2)
    CLC.tests = rospy.get_param('~tests', True)

    rospy.loginfo("Parameters:")
    rospy.loginfo("border_x = %f, border_y = %f",CLC.border_distance_x, CLC.border_distance_y)
    rospy.loginfo("fraction = %f", CLC.fraction)
    rospy.loginfo("threshold_normal = %f threshold_anormal = %f", CLC.threshold_normal, CLC.threshold_anormal)

    if CLC.tests == False:
      CLC.create_client()
    
    while not rospy.is_shutdown():
      if CLC.get_call == 0:
        CLC.polygons[:] = []

            
      #rospy.loginfo("Lane: %d F: %d R: %d",CLC.right_lane, CLC.points_front, CLC.points_right)

      CLC.change_lane_procedure()
      CLC.get_call = 0
      #rospy.loginfo("T: %d. La: %d",CLC.trybe, CLC.right_lane)
      change_lane_pub.publish(CLC.box_right)
      rospy.sleep(0.01)
      #rospy.spin()
