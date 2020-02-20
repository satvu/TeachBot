#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.msg as smsg
from std_msgs.msg import Bool, Int32, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ROTATE = [0, -3.14, 1.0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]
VEL_0 = [1.652628802228719e-10, 4.42951238155365e-10,2.825678568333387e-10, 4.566105365753174e-10, 4.458395600318909e-10, .29]
VEL_ROTATE_1 = [0, 0, 2e-9, 0, 0, 0]
VEL_ROTATE_2 = [0, 0, 1, 0, 0, 0]
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class VelocityClient:

    def __init__(self):
        rospy.Subscriber('/joint_states', JointState, self.send_command)

        self.client = actionlib.SimpleActionClient('/scaled_vel_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
                
    def send_command(self, joints):
        print 'press enter'
        pause = raw_input()

        self.client.wait_for_server()

        followJoint_msg = FollowJointTrajectoryGoal()

        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES

        jointPositions_msg = JointTrajectoryPoint()

        # jointPositions_msg.positions = ROTATE
        # jointPositions_msg.velocities = VEL_ROTATE_1

        # jointPositions_msg.positions = SCARA
        jointPositions_msg.velocities = VEL_ROTATE_2

        jointPositions_msg.accelerations = [0, 0, 0, 0, 0, 0]
        jointPositions_msg.time_from_start = rospy.Duration(3)

        traj_msg.points = [jointPositions_msg,]
        followJoint_msg.trajectory = traj_msg

        self.client.send_goal(followJoint_msg)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('request')
    pc = VelocityClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




