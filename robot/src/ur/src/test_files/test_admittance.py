#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.msg as smsg
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

from ur.msg import *
from ur.srv import *

SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ROTATE = [0, -3.14, 1.0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

'''
{
    "type":"set_robot_mode",
    "mode":"admittance ctrl",
    "resetPos":"joint_dof_start",
    "joints":[5],
    "min_thresh":[0.5]
},
'''

class VelocityClient:

    def __init__(self):
        # sending to group vel trajectory
        rospy.Subscriber('/joint_states', JointState, self.send_velocities)
        self.publish_velocity = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
                
	def cb_AdmittanceCtrl(self, joints, resetPos, rateNom=10, tics=15):
		self.joint_safety_check(lambda self : self.limb.go_to_joint_angles(resetPos), lambda self : None)
		rospy.Publisher('/robot/joint_state_publish_rate',UInt16,queue_size=10).publish(rateNom)		# Set publish rate

		# Filter effort and convert into velocity
		velocities = self.limb.joint_velocities()
		for joint in velocities.keys():
			if joint in joints.keys():
				allForces = [self.control['effort'][i][joint] for i in range(self.control['order'])]
				filteredForce = sum(allForces)/self.control['order']
				filteredForce = filteredForce + joints[joint]['bias']
				if abs(filteredForce) < joints[joint]['min_thresh']:
					velocities[joint] = 0
				else:
					velocities[joint] = -joints[joint]['F2V']*filteredForce
			else:
				velocities[joint] = 0

		self.limb.set_joint_velocities(velocities)

if __name__ == '__main__':
    rospy.init_node('request')
    pc = VelocityClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass