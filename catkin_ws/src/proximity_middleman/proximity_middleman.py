#!/usr/bin/env python
# todo: license here

import math

import rospy
from sensor_msgs.msg import LaserScan,Range

def callback_relay(in_msg):
    global pub
    
    # find the main axis projection of each range, and save the minimum
    ang = in_msg.angle_min
    inc = in_msg.angle_increment
    min_range = math.inf
    for range in in_msg.ranges:
        val = math.cos(ang)*range
        ang = ang + inc
        if val < min_range:
            min_range = val

    # create and publish the message with noise
    msg = Range()
    msg.header = in_msg.header
    msg.radiation_type = msg.INFRARED
    msg.field_of_view = in_msg.angle_max - in_msg.angle_min
    msg.min_range = in_msg.range_min
    msg.max_range = in_msg.range_max
    msg.range = min_range
    
    pub.publish(msg)


def proximity_middleman():
    global pub

    rospy.init_node('proximity_middleman', anonymous=True)
    
    # get what to subscribe and publish to
    rospy.Subscriber('proximity_raw', LaserScan, callback_relay)
    pub = rospy.Publisher('proximity', Range, queue_size=10)

    #keep the process alive
    rospy.spin()


if __name__ == '__main__':
    try:
        proximity_middleman()
    except rospy.ROSInterruptException:
        pass
