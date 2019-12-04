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
ROTATE_WRIST = [0, -3.14, 0, -3.14, -1.57, -1]
ROTATE_WRIST_BACK = [0, -3.14, 0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class PositionClient:

    def __init__(self):

        rospy.Subscriber('/ur_hardware_interface/robot_program_running', Bool, self.cb_robot_ready)

        self.cart_client = actionlib.SimpleActionClient('/teachbot/GoToCartesianPose', GoToCartesianPoseAction)
        
    def cb_robot_ready(self, res):
        if res.data:

            raw_input("Press Enter to test joint angles...")
            cart_msg = GoToCartesianPoseGoal()
            cart_msg.position = "None"
            cart_msg.orientation = "None"
            cart_msg.relative_pose = "None"
            cart_msg.joint_angles = "None"
            cart_msg.endpoint_pose = "None"
            self.joint_traj_client.send_goal(followJoint_msg)
            self.joint_traj_client.wait_for_result()

            raw_input("Press Enter to test position and orientation given...")
            cart_msg = GoToCartesianPoseGoal()
            cart_msg.position = "zero_kin_pos"
            cart_msg.orientation = "zero_kin_orientation"
            cart_msg.relative_pose = "None"
            cart_msg.joint_angles = "None"
            cart_msg.endpoint_pose = "None"
            self.joint_traj_client.send_goal(followJoint_msg)
            self.joint_traj_client.wait_for_result()

            raw_input("Press Enter to test relative pose...")
            cart_msg = GoToCartesianPoseGoal()
            cart_msg.position = "None"
            cart_msg.orientation = "None"
            cart_msg.relative_pose = "zero_kin_rel_base"
            cart_msg.joint_angles = "None"
            cart_msg.endpoint_pose = "None"
            self.joint_traj_client.send_goal(followJoint_msg)
            self.joint_traj_client.wait_for_result()

            raw_input("Press Enter to test endpoint pose...")
            cart_msg = GoToCartesianPoseGoal()
            cart_msg.position = "None"
            cart_msg.orientation = "None"
            cart_msg.relative_pose = "None"
            cart_msg.joint_angles = "None"
            cart_msg.endpoint_pose = "zero_kin_endpoint_pose"
            self.joint_traj_client.send_goal(followJoint_msg)
            self.joint_traj_client.wait_for_result()

                
                
if __name__ == '__main__':
    rospy.init_node('request')
    pc = PositionClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




