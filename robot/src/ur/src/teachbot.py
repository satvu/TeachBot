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

class Module:

    def __init__(self):
        #Initialize robot
        rospy.init_node('ur_comm_node', anonymous=True)

        # Publishing Topics
        self.pub_goal = rospy.Publisher('position', Position, queue_size=1) #this is to communicate with the limb
        self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1) #this is for the module/browser

        # Subscribing Topics
        rospy.Subscriber('/GoToJointAngles', GoToJointAngles, self.cb_GoToJointAngles)
        rospy.Subscriber('position_result', PositionResult, self.cb_position_complete)

        # Global Vars
        self.audio_duration = 0
        self.finished = False
        self.startPos = 0
        self.devMode = False
        self.seqArr = []

    def cb_GoToJointAngles(self, req):
        if self.VERBOSE: rospy.loginfo('Going to joint angles')

        if req.name is '':
			goal = Position(base=req.j0pos, shoulder=req.j1pos, elbow=req.j2pos, wrist1=req.j3pos, wrist2=req.j4pos, wrist3=req.j5pos)
			self.pub_goal.publish(goal)
        else:
            if req.wait == True:
                startTime = rospy.get_time()
                goal = Position()
                #this is from TeachBot, UR doesn't actually take in speed ratio and ways. you go to joint position like the above
                goto = self.limb.go_to_joint_angles(eval(req.name), speed_ratio=speed_ratio, ways = ways)

    def cb_position_complete(self, pos_res):
        if pos_res.completed and pos_res.initialized:
            self.command_complete_topic.publish()
    
## DEFINE IMPORTANT CONSTANTS ##
if __name__ == '__main__':
    # Position CONSTANTS
    SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
    ZERO = [0, -1.57, 0, -1.57, 0, 0]
    WRIST_3_FWD = [0, -3.14, 0, -3.14, -1.57, -1]
    WRIST_1_FWD = [0, -3.14, 0, -2.14, -1.57, 0]

    Module()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


