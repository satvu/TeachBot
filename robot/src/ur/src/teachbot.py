#!/usr/bin/env python
## IMPORTS ##
# Basic
import rospy, math
import numpy as np
import sys
import roslib

from std_msgs.msg import Bool, String, Int32, Float64, Float64MultiArray, UInt16, Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import WrenchStamped
import actionlib
import sensor_msgs
import threading

# Messages for communicating with module 
from ur.msg import *
from ur.srv import *

# Imports for kinematics
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
class Module():
    JOINTS = 6

    def __init__(self):
        #Initialize node
        rospy.init_node('ur_comm_node', anonymous=True)
        self.VERBOSE = True

        # Variables for kinematics conversions
        self.baselink = "shoulder_pan_joint"
        self.endlink = "wrist_3_joint" 
        self.tree = kdl_parser.treeFromParam("/robot_description")
        chain_ee = self.tree.getChain(self.baselink, self.endlink)
        self.fk_ee = kdl.ChainFkSolverPos_recursive(chain_ee)

        # Action Servers
        self.GoToJointAnglesAct = actionlib.SimpleActionServer('/teachbot/GoToJointAngles', GoToJointAnglesAction, execute_cb=self.cb_GoToJointAngles, auto_start=True)
        
        # Service Servers
        rospy.Service('/teachbot/audio_duration', AudioDuration, self.rx_audio_duration)

        # Action Clients - Publish to robot
        self.joint_traj_client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # TODO: Remove this soon, won't need anymore
        # Publish to browser so you know something is complete
        self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1) #this is for the module/browser

        # Global Vars
        self.audio_duration = 0
        self.finished = False
        self.startPos = 0
        self.devMode = False
        self.seqArr = []


    def rx_audio_duration(self,data):
        '''
        Audio duration of presentation/files
        '''
        self.audio_duration = data.audio_duration
        return True


    def cb_GoToJointAngles(self, goal):
        '''
        When the subscriber to "GoToJointAngles" receives the name of a constant from the browser,
        it is evaluated in this function and parsed into a "FollowTrajectoryGoal" object and sent to the
        action client provided by Universal Robots driver. Upon completion, it publishes to "/command-complete"
        to let the browser know that it has finished its task.
        '''
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

    def cb_GoToCartesianPose(self, req):
        '''
        Receives "GoToCartesianPose" message and based on which paramters are given, perform kinematic different
        operations.
        '''
        # Evaluate all of the paramters
        joint_angles = eval(req.joint_angles)
        relative_pose = eval(req.relative_pose)
        endpoint_pose = eval(req.endpoint_pose)
        position = eval(req.position)
        orientation = eval(req.orientation)

        if joint_angles and len(joint_angles) != len(JOINT_NAMES):
            rospy.logerr('len(joint_angles) does not match len(JOINT_NAMES)!')
            return None

        if (position is None and orientation is None and relative_pose is None and endpoint_pose is None):
            if joint_angles:
                # Forward Kinematics <-- comment from the Sawyer package, but seems this is just a goToJointAngles
                joint_input = GoToJointAngles()
                joint_input.name = req.joint_angles
                self.cb_GoToJointAngles(joint_input)

                # call to GoToJointAngles already sends a command_complete so we can return None to escape this function 
                return None 

            else:
                rospy.loginfo("No Cartesian pose or joint angles given.")
                return None
        else:
            # Inverse Kinematics 
            converted_joint_angles = ur_kin.inverse(endpoint_pose)
            joint_input = GoToJointAngles()

            # Set the new joint angle to go to
            joint_input.j0pos = converted_joint_angles[0]
            joint_input.j1pos = converted_joint_angles[1]
            joint_input.j2pos = converted_joint_angles[2]
            joint_input.j3pos = converted_joint_angles[3]
            joint_input.j4pos = converted_joint_angles[4]
            joint_input.j5pos = converted_joint_angles[5]

            self.cb_GoToJointAngles(joint_input)

            return None 

    def create_traj_goal(self, array):
        '''
        Helper function that takes in an array corresponding with the positions of each joint (starting from
        the base in the 0 index to wrist_3 in the last index) and returning a JointTrajectory() Message to use 
        to send to an action server (the ROS UR driver) 
        '''
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
    joint_dof_start = [-1.57, DSP,j2scara,0,-j4max,0]
    joint_dof_shoulder = [-1.57, -2.80 ,j2scara,0,-j4max,0]
    joint_dof_elbow = [-1.57, DSP, .30, 0,-j4max,0]
    joint_dof_wrist = [-1.57, DSP,j2scara,0, 0.45,0]
    joint_dof_up = [-1.57, DSP, j2scara,0,-j4max,0]

    m = Module()

    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


