#!/usr/bin/env python
# todo: license here

import random
import threading
import time
import math
import numpy as np
from numpy.linalg import inv

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from proj1robot_msgs.msg import GPS
from proj1robot_msgs.msg import Pose
from proj1robot_msgs.msg import ControlInput

lasttime = 0

# Kalman Filter parameters
inputs = np.array([0.0,0.0])
x_initial = np.array([0.05, 2.0, -math.pi/2])
x_init_dev = np.array([0.1*0.1,0.1*0.1,0.05*0.05]) # random uses standard deviation, but we want variance here, so square each term
# # model
# A_initial = np.eye(3, dtype=np.float64)
# B_initial = np.array([[math.cos(x_initial[2]), 0],
                     # [math.sin(x_initial[2]), 0],
                     # [0, 1]])
W_values = np.array([0.0,0.0]) #not loaded yet!

# kalman filters for each estimate
# ignore the use of dicts here, they get replaced with lists
# kalman_all = {}     #integrate Inputs, IMU, Odometry, and GPS
# kalman_dr = {}      #integrate Inputs
# kalman_dr_imu = {}  #integrate Inputs and IMU
# kalman_dr_all = {}  #integrate Inputs, IMU, and Odometry
# kalman_gps = {}     #integrate Inputs, IMU, and GPS

# update filters at every input command for simplicity
# Every sensor is treated as independent! (for simplicity)
sensor_data = {}
sensor_data['odom'] = {}
sensor_data['imu'] = {}
sensor_data['gps'] = {}
sensor_data.update
# noise of odometry, x, y, rot
sensor_data['odom']['y_k'] = np.array([0.0,0.0,0.0])
sensor_data['odom']['noise'] = np.array([0.0,0.0,0.0])
sensor_data['odom']['C_k'] = np.eye(3, dtype=np.float64)
sensor_data['odom']['ready'] = False
# noise of imu, rot
sensor_data['imu']['y_k'] = np.array([0.0])
sensor_data['imu']['noise'] = np.array([0.0])
sensor_data['imu']['C_k'] = np.array([0.0,0.0,1.0])
sensor_data['imu']['ready'] = False
# noise of GPS, 00, 01, 11, 10
sensor_data['gps']['y_k'] = np.array([0.0,0.0,0.0,0.0])
sensor_data['gps']['noise'] = np.array([0.0,0.0,0.0,0.0])
sensor_data['gps']['ready'] = False

# just for initializing the kalman
def kalman_init():
    global W_values, x_initial, x_init_dev
    A_k = np.array([[1, 0, -inputs[0]*math.sin(x_initial[2])],
                    [0, 1, inputs[0]*math.cos(x_initial[2])],
                    [0, 0, 1]])
    B_k = np.array([[math.cos(x_initial[2]), 0],
                    [math.sin(x_initial[2]), 0],
                    [0, 1]])
    # load initial uncertainty
    P = np.eye(3, dtype=np.float64)*x_init_dev
    # get control uncertainty
    # W_k = W_matrix*np.array([forward_gaussian[1]^2, turn_gaussian[1]^2)
    # calculate the initial sigma
    E = A_k.dot(P.dot(A_k.T)) # + B_k*W_k*B_k.T #There's no input yet, so there's no input noise

    # build the structure
    return (x_initial, E)


# convert the z rotation to incomplete quaternion (w,z)
def rot_quat(rot):
    return np.array([math.cos(rot/2), math.sin(rot/2)])


#convert incomplete quaternion (w,z) to z rotation
def quat_rot(quat):
    return math.atan2(2*(quat[0]*quat[1]),1-2*(quat[1]*quat[1]))


#given two incomplete quaternions (numpy vector of w,z), sum the rotations
def quat_sum(quat1, quat2):
    w = quat1[0]*quat2[0] - quat1[1]*quat2[1]
    z = quat1[0]*quat2[1] + quat2[0]*quat1[1]
    return np.array([w,z])


#sum euler angles
def euler_sum(rot1, rot2):
    return quat_rot(quat_sum(rot_quat(rot1), rot_quat(rot2)))


#sum states/state + change
def state_sum(x1, x2):
    return np.array([x1[0] + x2[0],x1[1] + x2[1],euler_sum(x1[2], x2[2])])


# Provides an update for the provided parameters
def kalman_update(kalman_input, C_k, V_k, y_k):
    global inputs, W_values
    
    #quickly make the W_matrix
    W_matrix = np.eye(2, dtype=np.float64)*inputs*W_values
    
    # we're expecting x_k, E_k
    x_k = kalman_input[0]
    E_k = kalman_input[1]
    
    # Calculate model parameters given the inputs
    A_k = np.array([[1, 0, -inputs[0]*math.sin(x_k[2])],
                    [0, 1, inputs[0]*math.cos(x_k[2])],
                    [0, 0, 1]])
    B_k = np.array([[math.cos(x_k[2]), 0],
                    [math.sin(x_k[2]), 0],
                    [0, 1]])
    
    # Predict the covariance
    P_k = inv(C_k.dot(E_k.dot(C_k.T)) + V_k)
    P_k = E_k - E_k.dot(C_k.T.dot(P_k.dot(C_k.dot(E_k))))
    
    # get the kalman gain
    K_k = P_k.dot(C_k.T.dot(inv(V_k)))
    
    # evolve the x_k given inputs
    x_k_guess = state_sum(x_k, B_k.dot(inputs))
    
    # final estimate
    x_k_innovation = y_k - C_k.dot(x_k_guess)
    x_k = state_sum(x_k_guess, K_k.dot(x_k_innovation))
    
    # update the covariance
    E_k = A_k.dot(P_k.dot(A_k.T)) + B_k.dot(W_matrix.dot(B_k.T))
    
    #update and return model
    return (x_k,E_k)


# Callback to store the updated input, then run each kalman filter
def callback_input(in_msg):
    global inputs, kalman_all, kalman_dr, kalman_dr_imu, kalman_dr_all, kalman_gps, lasttime
    
    delta = in_msg.header.stamp - lasttime
    lasttime = in_msg.header.stamp
    inputs = np.array([in_msg.forward, in_msg.turn]) * delta.to_sec()


# Callback to store relavent odometry information
def callback_odom(in_msg):
    global sensor_data
    data = np.array([in_msg.pose.pose.position.x,
                     in_msg.pose.pose.position.y,
                     quat_rot([in_msg.pose.pose.orientation.w,in_msg.pose.pose.orientation.z])])

    cov = np.array([in_msg.pose.covariance[0],
                    in_msg.pose.covariance[7],
                    in_msg.pose.covariance[35]])

    #"Direct" measurement, so just identity matrix
    #C_k = np.eye(3)
    #defined at start

    sensor_data['odom']['y_k'] = data
    sensor_data['odom']['noise'] = cov
    sensor_data['odom']['ready'] = True


# Callback to store relavent IMU information
def callback_imu(in_msg):
    data = np.array([quat_rot([in_msg.orientation.w,in_msg.orientation.z])])
    cov = np.array([in_msg.orientation_covariance[8]])
    # This doesn't seem to be exactly the same as the bearing sensor
    # "Direct measurement", so just extract the rotation
    #C_k = np.array([0.0,0.0,1.0])
    #defined at start

    sensor_data['imu']['y_k'] = data
    sensor_data['imu']['noise'] = cov
    sensor_data['imu']['ready'] = True

def callback_gps(in_msg):
    global noise
    #noise['gps']
    
    # project each beacon
    data = np.array([0.0,0.0,0.0,0.0])
    data[0] = math.cos(math.asin(4/in_msg.beacon00))*in_msg.beacon00
    data[1] = math.cos(math.asin(4/in_msg.beacon01))*in_msg.beacon01
    data[2] = math.cos(math.asin(4/in_msg.beacon11))*in_msg.beacon11
    data[3] = math.cos(math.asin(4/in_msg.beacon10))*in_msg.beacon10
    
    # project each stddev (estimated by neglecting small change in angle)
    cov = np.array([0.0,0.0,0.0,0.0])
    cov[0] = noise['gps'][1]*in_msg.beacon00/data[0]
    cov[1] = noise['gps'][1]*in_msg.beacon01/data[1]
    cov[2] = noise['gps'][1]*in_msg.beacon11/data[2]
    cov[3] = noise['gps'][1]*in_msg.beacon10/data[3]
    # was stored as stddev, so multiply by self for cov
    cov = cov*cov
    
    # C_k is in the input callback for each relavent filter
    # get the C_k
    # latest position estimate
    #C_k = np.array([[(x_k[0]-0.0)/data[0], (x_k[1]-0.0)/data[0], 0.0],
    #                [(x_k[0]-0.0)/data[1], (x_k[1]-10.0)/data[1], 0.0],
    #                [(x_k[0]-10.0)/data[2], (x_k[1]-10.0)/data[2], 0.0],
    #                [(x_k[0]-10.0)/data[3], (x_k[1]-0.0)/data[3], 0.0]])
    sensor_data['gps']['y_k'] = data
    sensor_data['gps']['noise'] = cov
    sensor_data['gps']['ready'] = True


# publish the estimated pose at rate
def publish_pose():
    global kalman_all, kalman_dr, kalman_dr_imu, kalman_dr_all, kalman_gps, pub, rate
    
    next = time.time()
    local_est = Pose()
    local_est.header.frame_id = 'world'
    while True:
        local_est.header.stamp = rospy.Time.now()

        #all
        x_k = kalman_all[0]
        local_est.x = x_k[0]
        local_est.y = x_k[1]
        local_est.theta = x_k[2]
        # publish the message
        pub['all'].publish(local_est)

        #dr
        x_k = kalman_dr[0]
        local_est.x = x_k[0]
        local_est.y = x_k[1]
        local_est.theta = x_k[2]
        # publish the message
        pub['dr'].publish(local_est)

        #dr_imu
        x_k = kalman_dr_imu[0]
        local_est.x = x_k[0]
        local_est.y = x_k[1]
        local_est.theta = x_k[2]
        # publish the message
        pub['dr_imu'].publish(local_est)

        #dr_all
        x_k = kalman_dr_all[0]
        local_est.x = x_k[0]
        local_est.y = x_k[1]
        local_est.theta = x_k[2]
        # publish the message
        pub['dr_all'].publish(local_est)

        #gps
        x_k = kalman_gps[0]
        local_est.x = x_k[0]
        local_est.y = x_k[1]
        local_est.theta = x_k[2]
        # publish the message
        pub['gps'].publish(local_est)

        # delay
        next = next + 1.0/rate
        try:
            time.sleep(next-time.time())
        except:
            # if timing is off, just skip
            pass


# Estimate the pose at rate
def filter_pose():
    global kalman_all, kalman_dr, kalman_dr_imu, kalman_dr_all, kalman_gps, pub, rate
    
    next = time.time()
    # are we ready?
    while not (sensor_data['odom']['ready'] and sensor_data['imu']['ready'] and sensor_data['gps']['ready']):
        time.sleep(0.0005)

    while True:
        # run dead reckoning
        kalman_dr = kalman_update(kalman_dr, np.eye(3), np.eye(3), np.zeros([3]))
        
        # run dead reckoning w/ imu (only orientation)
        C_k = np.reshape(sensor_data['imu']['C_k'],[1,3])
        V_k = np.reshape(sensor_data['imu']['noise'],[1,1])
        y_k = np.reshape(sensor_data['imu']['y_k'],[1,1])
        kalman_dr_imu = kalman_update(kalman_dr_imu, C_k, V_k, y_k)
        
        # Run dead reckoning w/ IMU and Odometry
        C_k = np.concatenate((np.reshape(sensor_data['imu']['C_k'],[1,3]),sensor_data['odom']['C_k']), axis=0)
        V_k = np.eye(4) * np.concatenate((sensor_data['imu']['noise'], sensor_data['odom']['noise']), axis=None)
        y_k = np.concatenate((sensor_data['imu']['y_k'], sensor_data['odom']['y_k']), axis=None)
        kalman_dr_all = kalman_update(kalman_dr_all, C_k, V_k, y_k)
        
        # run gps + imu
        x_k = kalman_gps[0]
        data = sensor_data['gps']['y_k']
        C_k = np.array([[(x_k[0]-0.0)/data[0], (x_k[1]-0.0)/data[0], 0.0],
                        [(x_k[0]-0.0)/data[1], (x_k[1]-10.0)/data[1], 0.0],
                        [(x_k[0]-10.0)/data[2], (x_k[1]-10.0)/data[2], 0.0],
                        [(x_k[0]-10.0)/data[3], (x_k[1]-0.0)/data[3], 0.0]])
        C_k = np.concatenate((np.reshape(sensor_data['imu']['C_k'],[1,3]),C_k), axis=0)
        V_k = np.eye(5) * np.concatenate((sensor_data['imu']['noise'], sensor_data['gps']['noise']), axis=None)
        y_k = np.concatenate((sensor_data['imu']['y_k'], sensor_data['gps']['y_k']), axis=None)
        kalman_gps = kalman_update(kalman_gps, C_k, V_k, y_k)
        
        #run all
        x_k = kalman_all[0]
        C_k = np.array([[(x_k[0]-0.0)/data[0], (x_k[1]-0.0)/data[0], 0.0],
                        [(x_k[0]-0.0)/data[1], (x_k[1]-10.0)/data[1], 0.0],
                        [(x_k[0]-10.0)/data[2], (x_k[1]-10.0)/data[2], 0.0],
                        [(x_k[0]-10.0)/data[3], (x_k[1]-0.0)/data[3], 0.0]])
        C_k = np.concatenate((np.reshape(sensor_data['imu']['C_k'],[1,3]),C_k,sensor_data['odom']['C_k']), axis=0)
        V_k = np.eye(8) * np.concatenate((sensor_data['imu']['noise'], sensor_data['gps']['noise'], sensor_data['odom']['noise']), axis=None)
        y_k = np.concatenate((sensor_data['imu']['y_k'], sensor_data['gps']['y_k'], sensor_data['odom']['y_k']), axis=None)
        kalman_all = kalman_update(kalman_all, C_k, V_k, y_k)

        # force python to evaluate all the maps (ugly, but python otherwise recurses and breaks itself)
        # probably should explore alternate for of data storage and representation, oops
        #list(kalman_dr)
        #list(kalman_dr_imu)
        #list(kalman_dr_all)
        #list(kalman_gps)
        #list(kalman_all)
        #didn't work

        # delay
        next = next + 1.0/rate
        try:
            time.sleep(next-time.time())
        except:
            # if timing is off, just skip
            pass

def pose_est():
    global kalman_all, kalman_dr, kalman_dr_imu, kalman_dr_all, kalman_gps, pub, rate, noise, lasttime

    rospy.init_node('pose_est', anonymous=True)
    lasttime=rospy.Time.now();

    # get the pose_est publishing rate
    rate = rospy.get_param('/simulation/pose_pub_rate', 20)

    # setup the kalman filters
    kalman_all = kalman_init()
    kalman_dr = kalman_init()
    kalman_dr_imu = kalman_init()
    kalman_dr_all = kalman_init()
    kalman_gps = kalman_init()

    # get the random noise parameters
    noise = {};
    noise['drive_forward'] = rospy.get_param('/simulation/forward_gaussian')
    noise['drive_turn'] = rospy.get_param('/simulation/turn_gaussian')
    noise['gps'] = rospy.get_param('/simulation/gps_gaussian')
    noise['imu'] = 0.05
    # odometry noise pulled from message
    # but ROS_planar_gazebo locks at 0.00001 for xy, and 0.001 for rot

    # get what to publish to and what to subscribe to
    pub = {}
    pub['all'] = rospy.Publisher('/robot/pose_est', Pose, queue_size=10)
    pub['dr'] = rospy.Publisher('/robot/pose_dr_est', Pose, queue_size=10)
    pub['dr_imu'] = rospy.Publisher('/robot/pose_dr_imu_est', Pose, queue_size=10)
    pub['dr_all'] = rospy.Publisher('/robot/pose_dr_all_est', Pose, queue_size=10)
    pub['gps'] = rospy.Publisher('/robot/pose_gps_est', Pose, queue_size=10)

    # subscribe to the cmd vel, odom, imu, and GPS
    rospy.Subscriber('cmd_vel', ControlInput, callback_input)
    rospy.Subscriber('odom', Odometry, callback_odom)
    rospy.Subscriber('imu', Imu, callback_imu)
    rospy.Subscriber('gps', GPS, callback_gps)

    # store the parameters back so that the parameter server can be updated with default values if not present
    rospy.set_param('/simulation/pose_pub_rate', rate)

    # start a thread to publish the pose at rate
    pubThread = threading.Thread(target=publish_pose)
    pubThread.daemon = True
    pubThread.start()

    # start a thread to filter the pose at rate
    filterThread = threading.Thread(target=filter_pose)
    filterThread.daemon = True
    filterThread.start()
    
    # spin
    rospy.spin()

if __name__ == '__main__':
    try:
        pose_est()
    except rospy.ROSInterruptException:
        pass
