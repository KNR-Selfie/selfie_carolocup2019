#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

def in1_callback(msg):
    global in1_val
    in1_val = msg.data

def in2_callback(msg):
    global in2_val
    in2_val = msg.data

if __name__ == '__main__':
    rospy.init_node('min_float64')

    in1 = rospy.get_param('~in_topic1')
    in2 = rospy.get_param('~in_topic2')
    out = rospy.get_param('~out_topic')
    default_value = rospy.get_param('~default_value', 0.0)
    publish_rate = rospy.get_param('~publish_rate', 50)

    global in1_val, in2_val
    in1_val = default_value
    in2_val = default_value

    rospy.Subscriber(in1, Float64, in1_callback)
    rospy.Subscriber(in2, Float64, in2_callback)

    pub = rospy.Publisher(out, Float64, queue_size=1)

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        pub.publish(min(in1_val, in2_val))
        rate.sleep()
