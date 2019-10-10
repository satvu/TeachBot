#!/usr/bin/env python
## IMPORTS ##
# Basic
import rospy, numpy, math
# from pygame import mixer
# from playsound import playsound
# import pyttsx3
# import cv2

from std_msgs.msg import Bool, String, Int32, Float64, Float64MultiArray, UInt16, Empty

import sensor_msgs
from ur.msg import *

#ur specific from final_push.py in ur_rtde
import threading
from scipy.fftpack import fft 
from statistics import mode, mean
from geometry_msgs.msg import WrenchStamped
import helper as hp

# corresponding modes for the UR limb
JOINT_MOVE = 0
ADMITTANCE = 1

class Module:

    def __init__(self):
        #Initialize node
        rospy.init_node('ur_comm_node', anonymous=True)
        self.limb_finished = False #whether or not the limb is ready, whether or not it can take a command rn
        self.VERBOSE = True 
        self.request_in_progress = False 

        # Publishing Topics
        self.pub_goal = rospy.Publisher('position', Position, queue_size=1) #this is to communicate with the limb
        self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1) #this is for the module/browser
        self.limb_mode_command = rospy.Publisher('/ur_mode', Int32, queue_size=1) #this is for changing the mode

        # Subscribing Topics
        rospy.Subscriber('/GoToJointAngles', GoToJointAngles, self.cb_GoToJointAngles) #get info from browser
        rospy.Subscriber('position_result', PositionResult, self.cb_position_complete) #get info from limb
        rospy.Subscriber('/joint_move', joint_move, self.cb_joint_move) #admittance command from browser


        # Global Vars
        self.audio_duration = 0
        self.finished = False
        self.startPos = 0
        self.devMode = False
        self.seqArr = []

    def cb_GoToJointAngles(self, req):
        if self.limb_finished is True:
            self.limb_finished = False 
            if self.VERBOSE: rospy.loginfo('going to joint angles')

            startTime = rospy.get_time()
            goal = Position()
            target = eval(req.name)
            print target
            goal = Position(base=target[0], shoulder=target[1], elbow=target[2], wrist1=target[3], wrist2=target[4], wrist3=target[5])
            self.pub_goal.publish(goal)

        else: 
            rospy.loginfo('limb is busy for go to joint angles')

    def cb_joint_move(self, req):
        self.command_complete = False 
        if self.limb_finished is True:
            self.limb_finished = False 
            if self.VERBOSE: rospy.loginfo('admittance mode')
            self.limb_mode_command.publish(ADMITTANCE)
            
        else: 
            rospy.loginfo('limb is busy')


    def cb_position_complete(self, pos_res):
        if pos_res.completed == True and pos_res.initialized == True:
            self.command_complete_topic.publish()
            self.limb_finished = True
    
## DEFINE IMPORTANT CONSTANTS ##
if __name__ == '__main__':
    # Position CONSTANTS
    SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
    ZERO = [0, -1.57, 0, -1.57, 0, 0]
    WRIST_3_FWD = [0, -3.14, 0, -3.14, -1.57, -1]
    WRIST_2_FWD = [0, -3.14, 0, -3.14, -.57, 0]
    WRIST_1_FWD = [0, -3.14, 0, -2.14, -1.57, 0]
    ELBOW_FWD = [0, -3.14, 1, -3.14, -1.57, 0]
    SHOULDER_FWD = [0, -2.80, 0, -3.14, -1.57, 0]
    BASE_FWD = [0.60, -3.14, 0, -3.14, -1.57, 0]

    m = Module()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


