#!/usr/bin/env python
# todo: license here

import threading
import time

import pickle
import matplotlib.pyplot as plt
import numpy as np

import rospy
from proj1robot_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState

state = {}
state['all'] = []
state['dr'] = []
state['dr_imu'] = []
state['dr_all'] = []
state['gps'] = []
state['ground_truth'] = []


#convert incomplete quaternion (w,z) to z rotation
def quat_rot(quat):
    return math.atan2(2*(quat[0]*quat[1]),1-2*(quat[1]*quat[1]))


#callback for the fully integrated localization
#also call service from here
def callback_all(in_msg):
    global state, get_model_state
    state['all'].append([in_msg.x, in_msg.y, in_msg.theta])

    # get the current robot state
    ground_truth = get_model_state('proj1robot','')
    rot = quat_rot(model_state.pose.orientation.w, model_state.pose.orientation.z)
    state['ground_truth'].append([model_state.pose.position.x, model_state.pose.position.y, rot])


#callbacks
def callback_dr(in_msg):
    global state
    state['dr'].append([in_msg.x, in_msg.y, in_msg.theta])


#callbacks
def callback_dr(in_msg):
    global state
    state['dr'].append([in_msg.x, in_msg.y, in_msg.theta])


#callbacks
def callback_dr_imu(in_msg):
    global state
    state['dr_imu'].append([in_msg.x, in_msg.y, in_msg.theta])


#callbacks
def callback_dr_all(in_msg):
    global state
    state['dr_all'].append([in_msg.x, in_msg.y, in_msg.theta])


#callbacks
def callback_gps(in_msg):
    global state
    state['gps'].append([in_msg.x, in_msg.y, in_msg.theta])


def publish_pose():
    global state

    # publish every second
    next = time.time()
    while True:
        # for each list, get the numpy representation
        gt = np.array(state['ground_truth'])
        true_x = gt[:,1]
        true_y = gt[:,2]
        true_theta = gt[:,3]
        
        i = 1
        for k,v in state.items:
            if k == 'ground_truth':
                continue
            
            #retrieve the values
            st = np.array(v)
            x = st[:,1]
            y = st[:,1]
            theta = st[:,1]
            
            #plot and label the ground_truth
            plt.figure(i)
            plt.clf()
            plt.plot(true_x, 'b')
            # label the final number
            plt.annotate('%0.2f' % e_k[-1], xy=(1, e_k[-1]), xytext=(8, 0),
                         xycoords=('axes fraction', 'data'), textcoords='offset points')
            plt.plot(true_y, 'r')
            # label the final number
            plt.annotate('%0.2f' % e_k[-1], xy=(1, e_k[-1]), xytext=(8, 0),
                         xycoords=('axes fraction', 'data'), textcoords='offset points')
            plt.plot(true_theta, 'g')
            # label the final number
            plt.annotate('%0.2f' % e_k[-1], xy=(1, e_k[-1]), xytext=(8, 0),
                         xycoords=('axes fraction', 'data'), textcoords='offset points')
            #plot the results from this estimate
            plt.plot(x, 'b-')
            plt.plot(y, 'r-')
            plt.plot(theta, 'g-')
            plt.legend(['True X', 'True Y', 'True Theta', 'Predicted X', 'Predicted Y', 'Predicted Theta'])
            plt.ylabel('Position (m) / Z-rotation (rad)')
            plt.xlabel('Time')
            plt.title('Pose Estimation using ' + k)
            plt.draw()
            
            #increment figure
            i = i + 1
        
        next = next + 1.0
        try:
            time.sleep(next-time.time())
        except:
            # if timing is off, just skip
            pass


def pose_stats(duration):
    global get_model_state, state

    rospy.init_node('proximity_middleman', anonymous=True)
    
    # get what to subscribe and publish to
    # did the lazy route
    subscriber = []
    subscriber.append(rospy.Subscriber('/robot/pose_est', Pose, callback_all))
    subscriber.append(rospy.Subscriber('/robot/pose_dr_est', Pose, callback_dr))
    subscriber.append(rospy.Subscriber('/robot/pose_dr_imu_est', Pose, callback_dr_imu))
    subscriber.append(rospy.Subscriber('/robot/pose_dr_all_est', Pose, callback_dr_all))
    subscriber.append(rospy.Subscriber('/robot/pose_gps_est', Pose, callback_gps))

    # we're running off the assumption that everything is already running
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # start a thread to display all the visualizations
    visThread = threading.Thread(target=publish_pose)
    visThread.daemon = True
    visThread.start()

    # sleep until kill
    exit = time.time() + duration
    time.sleep(exit-time.time())
    for sub in subscriber
        sub.unregister()
    
    #save the visualizations
    #and dump the stats locally
    # for each list, get the numpy representation
    gt = np.array(state['ground_truth'])
    true_x = gt[:,1]
    true_y = gt[:,2]
    true_theta = gt[:,3]
    np.savetxt('ground_truth.csv', gt, delimiter=",")
    
    i = 1
    for k,v in state.items:
        if k == 'ground_truth':
            continue
        
        #retrieve the values
        st = np.array(v)
        x = st[:,1]
        y = st[:,1]
        theta = st[:,1]
        np.savetxt(k+'.csv', st, delimiter=",")
        
        print("MSE for " + k, ((gt-st)**2).mean(axis=0))
        
        #plot and label the ground_truth
        plt.figure(i)
        plt.clf()
        plt.plot(true_x, 'b')
        # label the final number
        plt.annotate('%0.2f' % e_k[-1], xy=(1, e_k[-1]), xytext=(8, 0),
                     xycoords=('axes fraction', 'data'), textcoords='offset points')
        plt.plot(true_y, 'r')
        # label the final number
        plt.annotate('%0.2f' % e_k[-1], xy=(1, e_k[-1]), xytext=(8, 0),
                     xycoords=('axes fraction', 'data'), textcoords='offset points')
        plt.plot(true_theta, 'g')
        # label the final number
        plt.annotate('%0.2f' % e_k[-1], xy=(1, e_k[-1]), xytext=(8, 0),
                     xycoords=('axes fraction', 'data'), textcoords='offset points')
        #plot the results from this estimate
        plt.plot(x, 'b-')
        plt.plot(y, 'r-')
        plt.plot(theta, 'g-')
        plt.legend(['True X', 'True Y', 'True Theta', 'Predicted X', 'Predicted Y', 'Predicted Theta'])
        plt.ylabel('Position (m) / Z-rotation (rad)')
        plt.xlabel('Time')
        plt.title('Pose Estimation using ' + k)
        plt.savefig("vis\\"+k+".png")
        plt.draw()
        #increment figure
        i = i + 1
    
    #dump the raw stats locally
    filehandler = open("states.pkl", "wb")
    pickle.dump(state)
    filehandler.close()

    # leave it running so the user can see the end stats (mse)
    rospy.spin()

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 2:
            print("Please enter a duration in seconds")
        else:
            pose_stats(argv[1])
    except rospy.ROSInterruptException:
        pass
