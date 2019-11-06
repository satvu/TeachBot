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
    JOINTS = 6

    def __init__(self):
        #Initialize node
        rospy.init_node('ur_comm_node', anonymous=True)
        self.VERBOSE = True

        # Subscribing Topics (get commands from browser)
        rospy.Subscriber('/GoToJointAngles', GoToJointAngles, self.cb_GoToJointAngles)
        rospy.Subscriber('/GoToCartesianPose', GoToCartesianPose, self.cb_GoToCartesianPose)

        # Action Clients - Publish to robot
        self.joint_traj_client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Publish to browser so you know something is complete
        self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1) #this is for the module/browser

        # Global Vars
        self.audio_duration = 0
        self.finished = False
        self.startPos = 0
        self.devMode = False
        self.seqArr = []

    '''
    When the subscriber to "GoToJointAngles" receives the name of a constant from the browser,
    it is evaluated in this function and parsed into a "FollowTrajectoryGoal" object and sent to the
    action client provided by Universal Robots driver. Upon completion, it publishes to "/command-complete"
    to let the browser know that it has finished its task.
    '''
    def cb_GoToJointAngles(self, req):
        self.joint_traj_client.wait_for_server()

        followJoint_msg = FollowJointTrajectoryGoal()

        if req.name != '':
            print req.name
            followJoint_msg.trajectory = self.create_traj_goal(eval(req.name))
        else:
            followJoint_msg.trajectory = self.create_traj_goal([req.j0pos, req.j1pos, req.j2pos, req.j3pos, req.j4pos, req.j5pos, req.j6pos])
        
        # Send goal to action client and wait for completion of task
        self.joint_traj_client.send_goal(followJoint_msg)
        self.joint_traj_client.wait_for_result()

        self.command_complete_topic.publish()

    '''
    Receives "GoToCartesianPose" message and based on which paramters are given, perform kinematic different
    operations.
    '''
    def cb_GoToCartesianPose(self, req):
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
        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES

        jointPositions_msg = JointTrajectoryPoint()
        jointPositions_msg.positions = array
        jointPositions_msg.time_from_start = rospy.Duration(3)

        traj_msg.points = [jointPositions_msg,]

        return traj_msg


## DEFINE IMPORTANT CONSTANpointsTS --- MAKE SURE THEY MATCH WITH MODULE 1 OR 2 CONSTANTS ##
if __name__ == '__main__':
    # POSITION CONSTANTS - ARRAYS THAT MATCH JOINT_NAMES
    SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
    ZERO = [0, -1.57, 0, -1.57, 0, 0]
    WRIST_3_FWD = [0, -3.14, 0, -3.14, -1.57, -1]
    WRIST_2_FWD = [0, -3.14, 0, -3.14, -.57, 0]
    WRIST_1_FWD = [0, -3.14, 0, -2.14, -1.57, 0]
    ELBOW_FWD = [0, -3.14, 1, -3.14, -1.57, 0]
    SHOULDER_FWD = [0, -2.80, 0, -3.14, -1.57, 0]
    BASE_FWD = [0.60, -3.14, 0, -3.14, -1.57, 0]

    # 1
    joint_motor_animation_0 = SCARA
    joint_motor_animation_1 = joint_motor_animation_0[:]
    joint_motor_animation_1[1] = -2.90
    joint_motor_animation_1[4] = -2.90

    # 4
    joint_test = [WRIST_3_FWD, WRIST_2_FWD, WRIST_1_FWD, ELBOW_FWD, SHOULDER_FWD, BASE_FWD]

    m = Module()

    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


