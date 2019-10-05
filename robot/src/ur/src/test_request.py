#!/usr/bin/env python
'''
Created on July 1, 2019
Author: Albert Go (albertgo@mit.edu)
'''

import rospy
import sensor_msgs.msg as smsg
from ur.msg import Position, PositionFeedback, PositionResult
from std_msgs.msg import Bool

SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]

counter = 0

class PositionClient:

	def __init__(self):
		
		self.progress = []
		self.completed = False

		rospy.Subscriber('position_feedback', PositionFeedback, self.feedback_cb)
		rospy.Subscriber('position_result', PositionResult, self.send_positions)

		self.pub_goal = rospy.Publisher('position', Position, queue_size=1)

	def send_positions(self, data):
		if data.initialized == True and data.completed == True:
			goal = Position(base=0, shoulder=-2.19, elbow=0, wrist1=-2.31, wrist2=-1.57, wrist3=0)
			print(goal)
			self.pub_goal.publish(goal)
			return
		elif data.initialized == True and data.completed != True:
			print 'Awaiting completion. Progress:', self.progress
		else:
			print 'Waiting for robot to initialize.'

	def feedback_cb(self, data):
		#print data.progress
		self.progress = data.progress


if __name__ == '__main__':
	rospy.init_node('request')
	pc = PositionClient()

	try:              
		rospy.spin()

	except rospy.ROSInterruptException:
		pass




