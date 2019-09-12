#!/usr/bin/env python

import rospy
import intera_interface

rospy.init_node('Hello_Sawyer')
limb = intera_interface.Limb('right')
pose = limb.endpoint_pose()
print('position: [' + str(pose['position'].x) + ',' + str(pose['position'].y) + ',' + str(pose['position'].z) + ']')
print('orientation: [' + str(pose['orientation'].x) + ',' + str(pose['orientation'].y) + ',' + str(pose['orientation'].z) +',' +  str(pose['orientation'].w) + ']')