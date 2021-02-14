#!/usr/bin/env python
# license here

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def drive_twist_middleman():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    global pub
    
    # get the random noise
    forward_gaussian = rospy.get_param('forward_gaussian', [0.0, 0.05])
    turn_gaussian = rospy.get_param('turn_gaussian', [0.0, 0.05])

    # get what to subscribe to and what to publish to
    #rospy.get_param('
    pub = rospy.Publisher('/robot/cmd_vel',
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
