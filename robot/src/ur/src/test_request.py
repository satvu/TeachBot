#!/usr/bin/env python
'''
Created on July 1, 2019
Author: Albert Go (albertgo@mit.edu)
'''

import rospy
import sensor_msgs.msg as smsg
from ur_rtde.msg import Position, PositionFeedback, PositionResult
from std_msgs.msg import Bool

class PositionClient:

	def __init__(self):
		
		self.progress = []
		self.completed = False

		rospy.Subscriber('position_feedback', PositionFeedback, self.feedback_cb)
		rospy.Subscriber('position_result', PositionResult, self.send_positions)

		self.pub_goal = rospy.Publisher('position', Position, queue_size=1)

	def send_positions(self, data):

		if data.initialized == True and data.completed == True:
			print 'Enter a request, if no specific request, type and enter "none"'
			user = raw_input()
			print 'Sending Goal'
			goal = Position(base=0, shoulder=-2.19, elbow=0, wrist1=-2.31, wrist2=-1.57, wrist3=0, head =user)
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




