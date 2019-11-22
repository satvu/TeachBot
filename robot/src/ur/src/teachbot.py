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
        self.curr_pos # store current position, used in inverse kinematics

        ####### Variables for kinematics conversions ##########
        #the base and the end-effector names
        self.baselink = "shoulder_pan_joint"
        self.endlink = "wrist_3_joint" 
        #parse the robot_description
        self.tree = kdl_parser.treeFromParam("/robot_description")
        #create a chain using defined start/end joints
        chain_ee = self.tree.getChain(self.baselink, self.endlink)
        #create a forward kinematics solver
        self.fk = kdl.ChainFkSolverPos_recursive(chain_ee)
        #create an inverse kinematics solver
        vik=ChainIkSolverVel_pinv(chain)
        self.ik=ChainIkSolverPos_NR(chain_ee,self.fk,vik)

        ####### Servers, Subscribers, Clients, and Publishers ##########
        # Action Servers - get action goals from browser 
        self.GoToJointAnglesAct = actionlib.SimpleActionServer('/teachbot/GoToJointAngles', GoToJointAnglesAction, execute_cb=self.cb_GoToJointAngles, auto_start=True)
        self.GoToCartesianPoseAct = actionlib.SimpleActionServer('/teachbot/GoToCartesianPose', GoToCartesianPoseAction, execute_cb=self.cb_GoToCartesianPose, auto_start=True)

        # Service Servers
        rospy.Service('/teachbot/audio_duration', AudioDuration, self.rx_audio_duration)

        # Action Clients - Publish to robot
        self.joint_traj_client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Subscribers
        #TODO: Check that this is the write subscription and message type
        rospy.Subscriber('/robot/joint_states', sensor_msgs.msg.JointState, self.forwardJointState)

        # Publishers to browser so you know something is complete
        # TODO: Do we still need command complete and others if we have actions? This is still on main repo 
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


    def rx_audio_duration(self,data):
        '''
        Audio duration of presentation/files
        '''
        self.audio_duration = data.audio_duration
        return True
    
    def forwardJointState(self, data):
        '''
        Read the joint state information from the robot, parse, and store or publish to the appropriate topics.
        '''
        #publish info back to browser 
    	position = JointInfo()
		velocity = JointInfo()
		effort = JointInfo()
		for j in range(Module.JOINTS):
			setattr(position, 'j'+str(j), data.position[j+1])
			setattr(velocity, 'j'+str(j), data.velocity[j+1])
			setattr(effort, 'j'+str(j), data.effort[j+1])
		self.position_topic.publish(position)
		self.velocity_topic.publish(velocity)
		self.effort_topic.publish(effort)
        #for inverse kinematics
        self.curr_pos = position


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
        
        return True 

    def cb_GoToCartesianPose(self, req):
        '''
        Receives "GoToCartesianPose" message and based on which paramters are given, perform kinematic different
        operations.
        '''
        # Evaluate all of the paramters (defined in this file, passed through as string in the action)
        joint_angles = eval(req.joint_angles)
        relative_pose = eval(req.relative_pose)
        endpoint_pose = eval(req.endpoint_pose)
        position = eval(req.position)
        orientation = eval(req.orientation)

        # Other values defined in Sawyer (see limb_plus.py)
        # TODO: See if we actually need these values 
        in_tip_frame = False
        joint_angles = None
        tip_name = self.endlink
        linear_speed = 0.6
        linear_accel = 0.6
        rotational_speed = 1.57
        rotational_accel = 1.57 
        timeout = None
        endpoint_pose = None

        # hold 6 values, all joint angles 
        converted_joint_angles = []

        # set up result 
        result_GoToCartesianPose = GoToCartesianPoseResult()
        result_GoToCartesianPose.is_done = False

        if joint_angles and len(joint_angles) != len(JOINT_NAMES):
            rospy.logerr('len(joint_angles) does not match len(JOINT_NAMES)!')
            return None

        if (position is None and orientation is None and relative_pose is None and endpoint_pose is None):
            if joint_angles:
                # Forward Kinematics <-- comment from the Sawyer package, this section should be fk, but seems this is just a goToJointAngles
                joint_input = GoToJointAngles()
                joint_input.name = req.joint_angles
                self.cb_GoToJointAngles(joint_input)

                # successfully completed action, return to browser
                result_GoToCartesianPose.is_done = True
                self.GoToCartesianPoseAct.set_succeeded(result_GoToCartesianPose)

                # escape function 
                return None 

            else:
                rospy.loginfo("No Cartesian pose or joint angles given.")
                return None
        else:
            # TODO: Figure out what this chunk is about. There is not a 
			# endpoint_state = self.tip_state(tip_name)
			# if endpoint_state is None:
			# 	rospy.logerr('Endpoint state not found with tip name %s', tip_name)
			# 	return None
			# pose = endpoint_state.pose

			if relative_pose is not None:
				if len(relative_pose) != 6:
					rospy.logerr('Relative pose needs to have 6 elements (x,y,z,roll,pitch,yaw)')
					return None
				# create kdl frame from relative pose
				rot = kdl.Rotation.RPY(relative_pose[3],
										 relative_pose[4],
										 relative_pose[5])
				trans = kdl.Vector(relative_pose[0],
									 relative_pose[1],
									 relative_pose[2])
				f2 = kdl.Frame(rot, trans)

                # TODO: base frame or end effector frame, figure out ik for this 
				# and convert the result back to a pose message
				if in_tip_frame:
                    # end effector frame
                    pose = posemath.toMsg(posemath.fromMsg(pose) * f2)
				else:
				    # base frame
                    q_out=JntArray(6)
                    # TODO: do I need to do any checking w/ the value ik_result?
                    ik_result = self.ik.CartToJnt(self.curr_pos, f2, q_out)
                    converted_joint_angles = q_out
                    
			else:
				if endpoint_pose is None:
                    # create kdl frame from given
					if position is not None and len(position) == 3:
                        trans = kdl.Vector(position[0], position[1], position[2])
					if orientation is not None and len(orientation) == 4:
                        rot = kdl.Rotation.RPY(orientation[0], orientation[1], orientation[2])
                        # TODO: Figure out where to put wrench (orientation[3])
                    f2 = kdl.Frame(rot, trans)
                    q_out=JntArray(6)
                    # TODO: do I need to do any checking w/ value of ik_result?
                    ik_result = self.ik.CartToJnt(self.curr_pos, f2, q_out)
                    converted_joint_angles = q_out
                    

				else:
					trans = kdl.Vector(position[0], position[1], position[2])
                    rot = kdl.Rotation.RPY(orientation[0], orientation[1], orientation[2])
                    # TODO: Figure out where to put wrench (orientation[3])
                    q_out=JntArray(6)
                    # TODO: do I need to do any checking w/ value of ik_result?
                    ik_result = self.ik.CartToJnt(self.curr_pos, f2, q_out)
                    converted_joint_angles = q_out

			if not joint_angles:
                # Set the new joint angle to go to
                joint_input = GoToJointAngles()
                joint_input.j0pos = converted_joint_angles[0]
                joint_input.j1pos = converted_joint_angles[1]
                joint_input.j2pos = converted_joint_angles[2]
                joint_input.j3pos = converted_joint_angles[3]
                joint_input.j4pos = converted_joint_angles[4]
                joint_input.j5pos = converted_joint_angles[5]

                self.cb_GoToJointAngles(joint_input)
                
                # successfully completed action, return to browser
                result_GoToCartesianPose.is_done = True
                self.GoToCartesianPoseAct.set_succeeded(result_GoToCartesianPose)
                
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


