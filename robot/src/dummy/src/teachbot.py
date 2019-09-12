#! /usr/bin/env python

## IMPORTS ##
import rospy
from std_msgs.msg import (
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16,
	Empty
)
from dummy.msg import *

class Module():
	VERBOSE = True

	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Initialize
		rospy.init_node('Dummy_comm_node', anonymous=True)

		# Publishing topics
		self.position_topic = rospy.Publisher('/position', JointInfo, queue_size=1)
		self.velocity_topic = rospy.Publisher('/velocity', JointInfo, queue_size=1)
		self.effort_topic = rospy.Publisher('/effort', JointInfo, queue_size=1)
		self.dev_topic = rospy.Publisher('/dev_topic', Int32, queue_size = 10)
		self.scroll_wheel_button_topic = rospy.Publisher('/scroll_wheel_button_topic', Empty, queue_size = 10)
		self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1)

		# Subscribing topics
		rospy.Subscriber('/audio_duration', Float64, self.rx_audio_duration)
		rospy.Subscriber('/GoToJointAngles', GoToJointAngles, self.cb_GoToJointAngles)

		# Global Vars
		self.audio_duration = 0
		self.finished = False
		self.startPos = 0
		self.devMode = False
		self.seqArr = []

		# Initialization complete. Spin.
		rospy.loginfo('Ready.')
		rospy.spin()

	def addSeq(self, seq):
		self.seqArr.append(seq.asDict())

	## ROS PUBLISHERS ##
	# Publishes message to ROSTopic to be receieved by JavaScript
	def pub_cmd(self, cmd):
		msg = Int32()
		msg.data = cmd
		self.cmd2browser.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(cmd))
	def pub_num(self, num):
		msg = Float64()
		msg.data = num
		self.numeric_topic.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(num))
	def pub_arr(self, arr):
		msg = Float64MultiArray()
		msg.data = arr
		self.array_topic.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(arr))
	def pub_dev(self, cmd):
		msg = Int32()
		msg.data = cmd
		self.dev_topic.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(cmd))

	## ROS SUBSCRIBERS ##
	def rx_audio_duration(self,data):
		self.audio_duration = data.data

	def forwardJointState(self, data):
		position = JointInfo()
		velocity = JointInfo()
		effort = JointInfo()
		for j in range(Module.JOINTS):
			setattr(position, 'j'+str(j), data.position[j])
			setattr(velocity, 'j'+str(j), data.velocity[j])
			setattr(effort, 'j'+str(j), data.effort[j])
		self.position_topic.publish(position)
		self.velocity_topic.publish(velocity)
		self.effort_topic.publish(effort)

	def cb_GoToJointAngles(self, req):
		rospy.loginfo('going to joint angles')
		self.command_complete_topic.publish()

class Sequence():
	def __init__(self, idn, timestamp=None):
		self.idn = idn
		if timestamp is None:
			timestamp = rospy.get_time()
		self.timestamp = timestamp
		self.actions = []

	def addAction(self, name, timestamp=None):
		if timestamp is None:
			timestamp = rospy.get_time()
		self.actions.append({'name': name, 'timestamp': timestamp})

	def asDict(self):
		dct = {'idn': self.idn, 'timestamp': self.timestamp, 'actions': self.actions}

## DEFINE IMPORTANT CONSTANTS ##
if __name__ == '__main__':
	Module()