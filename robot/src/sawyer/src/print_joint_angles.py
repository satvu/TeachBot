#!/usr/bin/env python

import rospy
import intera_interface

rospy.init_node('Hello_Sawyer')
limb = intera_interface.Limb('right')
angles = limb.joint_angles()
print('[' + str(angles['right_j0']) + ',' + str(angles['right_j1']) + ',' + str(angles['right_j2']) + ',' + str(angles['right_j3']) + ',' + str(angles['right_j4']) + ',' + str(angles['right_j5']) + ',' + str(angles['right_j6']) + ']')
