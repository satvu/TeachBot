## IMPORTS ##
# Basic
import rospy, numpy, math
from pygame import mixer
from playsound import playsound
import pyttsx3
import cv2

# Intera
import intera_interface
from std_msgs.msg import (
	Bool,
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16,
	Empty
)
import sensor_msgs
import intera_core_msgs

#ur specific from final_push.py in ur_rtde
import threading
from scipy.fftpack import fft 
from statistics import mode, mean
from geometry_msgs.msg import WrenchStamped
import helper as hp

class Module():
    #Initialize robot
    rospy.init_node('UR_comm_node', anonymous=True)
    intera_interface.HeadDisplay().display_image('logo.png')

    # Publishing Topics
	self.pub_goal = rospy.Publisher('position', Position, queue_size=1)

    # Subscribing Topics
    rospy.Subscriber('/GoToJointAngles', GoToJointAngles, self.cb_GoToJointAngles)

    # Global Vars
    self.audio_duration = 0
    self.finished = False
    self.startPos = 0
    self.devMode = False
    self.seqArr = []

	## HELPER FUNCTION ##
	# Returns true if two positions equal each other or are at least within a given tolerance
	def endpoints_equal(self, pose1, pose2, tol=0):
		equality = True
		if 'position' in pose1 and 'position' in pose2:
			equality &= abs(pose1['position'].x-pose2['position'].x)<=tol \
			   and abs(pose1['position'].y-pose2['position'].y)<=tol \
			   and abs(pose1['position'].z-pose2['position'].z)<=tol
		if 'orientation' in pose1 and 'orientation' in pose2:
			equality &= abs(pose1['orientation'].x-pose2['orientation'].x)<=tol \
			   and abs(pose1['orientation'].y-pose2['orientation'].y)<=tol \
			   and abs(pose1['orientation'].z-pose2['orientation'].z)<=tol \
			   and abs(pose1['orientation'].w-pose2['orientation'].w)<=tol
		return equality


    def cb_GoToJointAngles(self, req):
        if self.VERBOSE: rospy.loginfo('Going to joint angles')

        if req.name is '':
			goal = Position(base=req.j0pos, shoulder=req.j1pos, elbow=req.j2pos, wrist1=req.j3pos, wrist2=req.j4pos, wrist3=req.j5pos)
			self.pub_goal.publish(goal)
			return
