#!/usr/bin/env python
# todo: license here

import random
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from proj1robot_msgs.msg import ControlInput


state_changed = False
fwd = 0.0
turn = 0.0

def callback_relay(in_msg):
    global state_changed, fwd, turn
    
    # Block while state_changed is already true
    while state_changed:
        time.sleep(0.0005)
    # the only input movements are going to be forwards/backwards velocity and angular rotation
    # many controllers appear to use x for the forward direction, but the robot has y as the forward direction based on project spec
    fwd = in_msg.linear.x
    turn = in_msg.angular.z
    state_changed = True


def drive_twist_middleman():
    global state_changed, fwd, turn

    rospy.init_node('drive_twist_middleman')
    
    # get the rate
    rate = rospy.get_param('/simulation/drive_rate', 20)
    
    # get the random noise
    forward_gaussian = rospy.get_param('/simulation/forward_gaussian', [0.0, 0.2])
    turn_gaussian = rospy.get_param('/simulation/turn_gaussian', [0.0, 0.2])

    # get what to publish to and what to subscribe to
    pubto = rospy.get_param('/simulation/drive_mm_out', '/robot/cmd_vel')
    pub = rospy.Publisher(pubto, Twist, queue_size=10)
    pub_raw = rospy.Publisher('/robot/cmd_vel_raw', ControlInput, queue_size=10) #got a little lazy here
    
    subto = rospy.get_param('/simulation/drive_mm_in', '/controller/drive')
    rospy.Subscriber(subto, Twist, callback_relay)

    # store the parameters back so that the parameter server can be updated with default values if not present
    rospy.set_param('/simulation/drive_rate', rate)
    rospy.set_param('/simulation/forward_gaussian', forward_gaussian)
    rospy.set_param('/simulation/turn_gaussian', turn_gaussian)
    rospy.set_param('/simulation/drive_mm_out', pubto)
    rospy.set_param('/simulation/drive_mm_in', subto)
    
    # start running at rate 
    next = time.time()
    msg = Twist()
    msg_raw = ControlInput()
    while True:
        # update control state if changed
        if state_changed:
            msg_raw.forward = fwd
            msg_raw.turn = turn
            state_changed = False

        # generate noise based on input
        fwd_noise = random.gauss(forward_gaussian[0], forward_gaussian[1]) * msg_raw.forward
        turn_noise = random.gauss(turn_gaussian[0], turn_gaussian[1]) * msg_raw.turn
        
        # update and publish the message with noise
        msg.linear.y = msg_raw.forward + fwd_noise
        msg.angular.z = msg_raw.turn + turn_noise
        
        pub.publish(msg)
        pub_raw.publish(msg_raw)
        
        # delay
        next = next + 1.0/rate
        try:
            time.sleep(next-time.time())
        except:
            # if timing is off, just skip
            pass


if __name__ == '__main__':
    try:
        drive_twist_middleman()
    except rospy.ROSInterruptException:
        pass
