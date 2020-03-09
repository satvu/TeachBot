#!/usr/bin/env python
## IMPORTS ##
# Basic
import rospy, math
import numpy as np
import sys
import roslib
from ur_kinematics import *

from std_msgs.msg import Bool, String, Int32, Float64, Float64MultiArray, UInt16, Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
import sensor_msgs
import threading

from ur.msg import *
from ur.srv import *

from geometry_msgs.msg import WrenchStamped

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
class Module():
    JOINTS = 6

    def __init__(self):
        #Initialize node
        rospy.init_node('ur_comm_node', anonymous=True)
        self.VERBOSE = True

        # Action Servers
        self.GoToJointAnglesAct = actionlib.SimpleActionServer('/teachbot/GoToJointAngles', GoToJointAnglesAction, execute_cb=self.cb_GoToJointAngles, auto_start=True)
        
        # Service Servers
        rospy.Service('/teachbot/audio_duration', AudioDuration, self.rx_audio_duration)


        # Action Clients - Publish to robot
        self.joint_traj_client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Subscribed Topics
        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.forwardJointState)

        # Publish topics 
        self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1) #this is for the module/browser
        self.position_topic = rospy.Publisher('/teachbot/position', JointInfo, queue_size=1)
        self.velocity_topic = rospy.Publisher('/teachbot/velocity', JointInfo, queue_size=1)
        self.effort_topic = rospy.Publisher('/teachbot/effort', JointInfo, queue_size=1)

        # Global Vars
        self.audio_duration = 0
        self.finished = False
        self.startPos = 0
        self.devMode = False
        self.seqArr = []

        rospy.loginfo('TeachBot is initialized and ready to go.')


    def rx_audio_duration(self,data):
        self.audio_duration = data.audio_duration
        return True

    '''
    Read information from joint states, parse into position, velocity, and effort, then 
    forward the information by publishing it.
    '''
    def forwardJointState(self, data):
        position = JointInfo()
        velocity = JointInfo()
        effort = JointInfo()
        for j in range(Module.JOINTS):
            setattr(position, 'j'+str(j), data.position[j])
            setattr(velocity, 'j'+str(j), data.velocity[j])
            setattr(effort, 'j'+str(j), data.effort[j])
            # TODO: Ask about the control variable that is in Sawyer (is that needed here?)
        rospy.loginfo(position)
        rospy.loginfo(velocity)
        rospy.loginfo(effort)
        self.position_topic.publish(position)
        self.velocity_topic.publish(velocity)
        self.effort_topic.publish(effort)

    '''
    When the subscriber to "GoToJointAngles" receives the name of a constant from the browser,
    it is evaluated in this function and parsed into a "FollowTrajectoryGoal" object and sent to the
    action client provided by Universal Robots driver. Upon completion, it publishes to "/command-complete"
    to let the browser know that it has finished its task.
    '''
    def cb_GoToJointAngles(self, goal):
        self.joint_traj_client.wait_for_server()

        followJoint_msg = FollowJointTrajectoryGoal()

        if goal.name != '':
            print goal.name
            followJoint_msg.trajectory = self.create_traj_goal(eval(goal.name))
        else:
            followJoint_msg.trajectory = self.create_traj_goal([goal.j0pos, goal.j1pos, goal.j2pos, goal.j3pos, goal.j4pos, goal.j5pos])
        
        # Send goal to action client and wait for completion of task
        self.joint_traj_client.send_goal(followJoint_msg)
        self.joint_traj_client.wait_for_result()

        # Set success and return to browser module
        result = ur.msg.GoToJointAnglesResult()
        result.success = True
        self.GoToJointAnglesAct.set_succeeded(result)


    def create_traj_goal(self, array):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES

        jointPositions_msg = JointTrajectoryPoint()
        jointPositions_msg.positions = array
        jointPositions_msg.time_from_start = rospy.Duration(3)

        traj_msg.points = [jointPositions_msg,]

        return traj_msg


if __name__ == '__main__':
    ## DEFINE IMPORTANT CONSTANTS --- MAKE SURE THEY MATCH WITH MODULE 1 OR 2 CONSTANTS ##

    # POSITION CONSTANTS - ARRAYS THAT MATCH JOINT_NAMES
    ZERO = [0, -1.57, 0, -1.57, 0, 0]
    SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
    WRIST_3_FWD = [0, -3.14, 0, -3.14, -1.57, -.5]
    WRIST_2_FWD = [0, -3.14, 0, -3.14, -1.0, 0]
    WRIST_1_FWD = [0, -3.14, 0, -2.54, -1.57, 0]
    ELBOW_FWD = [0, -3.14, 0.5, -3.14, -1.57, 0]
    SHOULDER_FWD = [0, -2.80, 0, -3.14, -1.57, 0]
    BASE_FWD = [0.50, -3.14, 0, -3.14, -1.57, 0]
    
    default = ZERO

    # TODO: Figure out what no_hit is and what j4 max is    
    # TODO: Figure out DPS becaues the arm needs to be over the table for this part
    DSP = SCARA[1]
    j2scara = SCARA[2]
    over_table = -3.5
    j4max = SCARA[4]

    # 1
    joint_motor_animation_0 = SCARA
    joint_motor_animation_1 = [0, -3.14, -0.25, -3.14, -1.25, 0]
    # 4
    joint_test = [0]*Module.JOINTS
    joint_test = [WRIST_3_FWD, WRIST_2_FWD, WRIST_1_FWD, ELBOW_FWD, SHOULDER_FWD, BASE_FWD]
    
    # 6 - 15
    # TODO Find the hard-coded values that work for UR
    # joint_dof_start = [-1.57, DSP,j2scara,0,-j4max,0]
    # joint_dof_shoulder = [-1.57, -2.80 ,j2scara,0,-j4max,0]
    # joint_dof_elbow = [-1.57, DSP, .30, 0,-j4max,0]
    # joint_dof_wrist = [-1.57, DSP,j2scara,0, 0.45,0]
    # joint_dof_up = [-1.57, DSP, j2scara,0,-j4max,0]
    # for now use the following so it goes through this section of the module but doesn't swing around like crazy
    joint_dof_start = ZERO
    joint_dof_shoulder = ZERO
    joint_dof_elbow = ZERO
    joint_dof_wrist = ZERO
    joint_dof_up = ZERO

    m = Module()

    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


