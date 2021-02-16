#!/usr/bin/env python
# todo: license here

import math
import random
import threading
import time

import rospy
from proj1robot_msgs.msg import GPS
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState


# Calculate the distance between two points
def euclidean_norm(p1, p2):
    x = p1.x - p2.x
    y = p1.y - p2.y
    z = p1.z - p2.z
    return math.sqrt(x*x + y*y + z*z)


# generate the local gps data based on the model state
def calc_pos_and_publish():
    global pub, gps_gaussian
    
    # get the position of the robot model
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state('proj1robot')
        # remember, the model origin is 0.2 towards the rear of the geometric center.
        
        beacon = Point()
        beacon.x = 0
        beacon.y = 0
        beacon.z = 4
        
        # create the GPS message and calculate each beacon distance
        gps = GPS()
        gps.header = model_state.header
        gps.beacon00 = euclidean_norm(beacon, model_state.pose.position)
        
        beacon.x = 10
        gps.beacon10 = euclidean_norm(beacon, model_state.pose.position)
        
        beacon.y = 10
        gps.beacon11 = euclidean_norm(beacon, model_state.pose.position)
        
        beacon.x = 0
        gps.beacon01 = euclidean_norm(beacon, model_state.pose.position)
        
        # peturb each beacon distance
        gps.beacon00 = gps.beacon00 + gps.beacon00 * random.gauss(gps_gaussian[0], gps_gaussian[1])
        gps.beacon10 = gps.beacon10 + gps.beacon10 * random.gauss(gps_gaussian[0], gps_gaussian[1])
        gps.beacon01 = gps.beacon01 + gps.beacon01 * random.gauss(gps_gaussian[0], gps_gaussian[1])
        gps.beacon11 = gps.beacon11 + gps.beacon11 * random.gauss(gps_gaussian[0], gps_gaussian[1])
        
        # publish the message
        pub.publish(gps)

    except rospy.ServiceException as e:
        print("Service call failed: %s, robot may not have been spawned yet"%e)


def local_gps():
    global pub, gps_gaussian

    rospy.init_node('local_gps')
    
    # get the rate
    rate = rospy.get_param('/simulation/gps_rate', 30)
    
    # get the random noise (it scales with distance. this is noise at 1m)
    gps_gaussian = rospy.get_param('/simulation/gps_gaussian', [0.0, 0.05])
    
    # store the parameters back so that the parameter server can be updated with default values if not present
    rospy.set_param('/simulation/gps_rate', rate)
    rospy.set_param('/simulation/gps_gaussian', gps_gaussian)

    # get what to publish to and wait for the gazebo service to start
    pub = rospy.Publisher('/robot/gps', GPS, queue_size=10)
    
    rospy.wait_for_service('/gazebo/get_model_state')
    
    # start running at rate 
    next = time.time()
    while True:
        calc_pos_and_publish()
        next = next + 1.0/rate
        time.sleep(next-time.time())
    
    #keep the process alive
    rospy.spin()


if __name__ == '__main__':
    try:
        local_gps()
    except rospy.ROSInterruptException:
        pass
