#!/usr/bin/env python
# todo: license here

import random

import rospy
from geometry_msgs.msg import Twist

def callback_relay(in_msg):
    global pub, forward_gaussian, turn_gaussian
    
    # the only input movements are going to be forwards/backwards velocity and angular rotation
    # many controllers appear to use x for the forward direction, but the robot has y as the forward direction based on project spec
    fwd = in_msg.linear.x
    turn = in_msg.angular.z

    fwd_noise = 0.0
    turn_noise = 0.0
    # generate noise if any nonzero input
    if fwd or turn:
        fwd_noise = random.gauss(forward_gaussian[0], forward_gaussian[1])
        turn_noise = random.gauss(turn_gaussian[0], turn_gaussian[1])
    
    # create and publish the message with noise
    msg = Twist()
    msg.linear.y = fwd + fwd_noise
    msg.angular.z = turn + turn_noise
    
    pub.publish(msg)


def drive_twist_middleman():
    global pub, forward_gaussian, turn_gaussian

    rospy.init_node('drive_twist_middleman')
    
    # get the random noise
    forward_gaussian = rospy.get_param('/simulation/forward_gaussian', [0.0, 0.2])
    turn_gaussian = rospy.get_param('/simulation/turn_gaussian', [0.0, 0.2])

    # get what to publish to and what to subscribe to
    pubto = rospy.get_param('/simulation/drive_mm_out', '/robot/cmd_vel')
    pub = rospy.Publisher(pubto, Twist, queue_size=10)
    
    subto = rospy.get_param('/simulation/drive_mm_in', '/controller/drive')
    rospy.Subscriber(subto, Twist, callback_relay)

    # store the parameters back so that the parameter server can be updated with default values if not present
    rospy.set_param('/simulation/forward_gaussian', forward_gaussian)
    rospy.set_param('/simulation/turn_gaussian', turn_gaussian)
    rospy.set_param('/simulation/drive_mm_out', pubto)
    rospy.set_param('/simulation/drive_mm_in', subto)
    
    # todo: consider adding way to change random noise on the fly
    
    #keep the process alive
    rospy.spin()


if __name__ == '__main__':
    try:
        drive_twist_middleman()
    except rospy.ROSInterruptException:
        pass
