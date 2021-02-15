#!/usr/bin/env python
# todo: license here

import random
import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from tf.transformations import quaternion_from_euler

# mini helper function to generate the random starting position.

def generate_random_position():

    # get the random noise and location
    xyY_mean = rospy.get_param('/simulation/spawn_mean_gauss', [0.05, 2.0, -1.5708])
    xyY_stddev = rospy.get_param('/simulation/spawn_stddev_gauss', [0.1, 0.1, 0.05])
    rospy.set_param('/simulation/spawn_mean_gauss', xyY_mean)
    rospy.set_param('/simulation/spawn_stddev_gauss', xyY_stddev)

    # generate values
    xyY = []
    for mean, stddev in zip(xyY_mean, xyY_stddev):
        xyY.append(random.gauss(mean, stddev))

    # don't let it get below zero x otherwise robot spawns in a broken position
    if xyY[0] < 0:
        xyY[0] = 0

    print("-x " + str(xyY[0]) + " -y " + str(xyY[1]) + " -Y " + str(xyY[2]))

    q = quaternion_from_euler(0, 0, xyY[2])

    # generate the modelstate information
    state = ModelState()
    state.model_name = 'proj1robot'
    state.pose.position.x = xyY[0]
    state.pose.position.y = xyY[1]
    state.pose.position.z = 0
    state.pose.orientation.x = q[0]
    state.pose.orientation.y = q[1]
    state.pose.orientation.z = q[2]
    state.pose.orientation.w = q[3]

    # now try to add it to the robot
    print("waiting for gazeobo")
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_state(state)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    try:
        generate_random_position()
    except rospy.ROSInterruptException:
        pass
