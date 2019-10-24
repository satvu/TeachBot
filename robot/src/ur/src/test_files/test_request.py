#!/usr/bin/env python
'''
Created on July 1, 2019
Author: Albert Go (albertgo@mit.edu)
'''

import rospy
import sys
import roslib
import sensor_msgs.msg as smsg
from ur.msg import Position, PositionFeedback, PositionResult, GoToCartesianPose
from std_msgs.msg import Bool, Int32
from ur_kin_py import forward, reverse

SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ROTATE_WRIST = [0, -3.14, 0, -3.14, -1.57, -1]
ROTATE_WRIST_BACK = [0, -3.14, 0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]

JOINT_MOVE = 0
ADMITTANCE = 1

class PositionClient:

    def __init__(self):

        self.progress = []
        self.completed = False
        
        self.pub_goal = rospy.Publisher('/GoToCartesianPose', GoToCartesianPose, queue_size = 1)

        print 'press enter to send to test forward kinematics'
        converted_joint_angles1 = ur_kin.forward(np.array(SCARA))
        print converted_joint_angles1

        print 'press enter to send to test inverse kinematics'
        converted_joint_angles2 = ur_kinematics.ur_kin_py.inverse(np.array(converted_joint_angles1))
        print converted_joint_angles2

        print 'prese enter to test with robot'
        user = raw_input()
        pose = GoToCartesianPose()
        pose.endpoint_pose = 'test_cartesian_pose'
        self.pub_goal(pose)


if __name__ == '__main__':
    rospy.init_node('request')
    pc = PositionClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




