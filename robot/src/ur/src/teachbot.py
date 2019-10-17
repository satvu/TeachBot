#!/usr/bin/env python
## IMPORTS ##
# Basic
import rospy, numpy, math

from std_msgs.msg import Bool, String, Int32, Float64, Float64MultiArray, UInt16, Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
import sensor_msgs
import threading

from ur.msg import *

from scipy.fftpack import fft 
from statistics import mode, mean
from geometry_msgs.msg import WrenchStamped

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class Module:

    def __init__(self):
        #Initialize node
        rospy.init_node('ur_comm_node', anonymous=True)
        self.VERBOSE = True 

        self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1) #this is for the module/browser

        # Subscribing Topics
        rospy.Subscriber('/GoToJointAngles', GoToJointAngles, self.cb_GoToJointAngles) #get info from browser
        # rospy.Subscriber('/joint_move', joint_move, self.cb_joint_move) #admittance command from browser

        # Action Clients
        self.joint_traj_client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Global Vars
        self.audio_duration = 0
        self.finished = False
        self.startPos = 0
        self.devMode = False
        self.seqArr = []

    def cb_GoToJointAngles(self, req):
            self.joint_traj_client.wait_for_server()

            followJoint_msg = FollowJointTrajectoryGoal()

            traj_msg = JointTrajectory()
            traj_msg.joint_names = JOINT_NAMES

            jointPositions_msg = JointTrajectoryPoint()
            jointPositions_msg.positions = eval(req.name)
            jointPositions_msg.time_from_start = rospy.Duration(3)

            traj_msg.points = [jointPositions_msg,]
            followJoint_msg.trajectory = traj_msg

            self.joint_traj_client.send_goal(followJoint_msg)
            self.joint_traj_client.wait_for_result()

            self.command_complete_topic.publish()
    
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


