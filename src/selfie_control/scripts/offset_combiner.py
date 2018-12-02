#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from selfie_control.cfg import AngularCoeffConfig

UPDATE_RATE = 50

position_offset = 0.0
angular_offset = 0.0

def config_callback(config, level):
    global L
    L = config['L']

    rospy.loginfo("Reconfigure request: L=" + str(L) + "m")

    return config

def position_offset_callback(msg):
    global position_offset
    position_offset = msg.data

def angular_offset_callback(msg):
    global angular_offset
    angular_offset = msg.data

if __name__ == '__main__':
    rospy.init_node('selfie_offset_combiner')

    srv = Server(AngularCoeffConfig, config_callback)

    position_offset_sub = rospy.Subscriber('position_offset',
                                            Float64,
                                            position_offset_callback,
                                            queue_size=1)

    angular_offset_sub = rospy.Subscriber('angular_offset',
                                           Float64,
                                           angular_offset_callback,
                                           queue_size=1)

    combined_offset_pub = rospy.Publisher('combined_offset', Float64, queue_size=1)

    rate = rospy.Rate(UPDATE_RATE)
    while not rospy.is_shutdown():
        combined_offset = position_offset + L*angular_offset
        combined_offset_pub.publish(combined_offset)
        rate.sleep()
