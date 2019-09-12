#!/usr/bin/env python
import rospy

from std_msgs.msg import (String,Int32, Float64, Float64MultiArray, UInt16)
import time

verbose = False
joints =7
class listener():
	def __init__(self):
		# Publishing topics
		self.iframe_sequence_pub = rospy.Publisher('/cmd2browser', Int32, queue_size = 10)
		self.numeric_pub = rospy.Publisher('/numeric_topic', Float64, queue_size=10)
		self.array_pub = rospy.Publisher('/array_topic', Float64MultiArray, queue_size=10)

		# Subscribing topics
		rospy.Subscriber('/cmd2shell', Int32, self.rx_command)
		rospy.Subscriber('/audio_duration', Float64, self.rx_audio_duration)
		rospy.Subscriber('/right_navigator_button', String, self.rx_nav_button)

		# Global Vars
		self.audio_duration = 0;
		self.finished = False;
		self.startPos = 0

		rospy.init_node('fake', anonymous = True)

		rospy.spin()

		# Initialize Sawyer
		# rospy.init_node('Sawyer_Sparrow_comm_node', anonymous=True)
		# go_to.joint_angles(empty_angle_arg)
		# self.limb = intera_interface.Limb('right')
		# navigator.right

	def pub_cmd(self, counter):
		int_msg = Int32()
		int_msg.data = counter
		self.iframe_sequence_pub.publish(int_msg)
		if verbose: rospy.loginfo("I sent: " + str(counter))
	def pub_num(self, num):
		msg = Float64()
		msg.data = num
		self.numeric_pub.publish(msg)
		if verbose: rospy.loginfo("I sent: " + str(num))
	def pub_arr(self, arr):
		msg = Float64MultiArray()
		msg.data = arr
		self.array_pub.publish(msg)
		if verbose: rospy.loginfo("I sent: " + str(arr))



	def joint_impedance_move(self,b,k,terminatingCondition,pCMD=lambda self: None, rateNom=50):
		while not terminatingCondition(self):
			rospy.Publisher('robot/joint_state_publish_rate',UInt16,queue_size=10).publish(rateNom)

	def rx_command(self, data):
		seq = data.data
		print("recieved", seq)

		if seq == 11:
			for i in range(100):
				rospy.Publisher('robot/joint_state_publish_rate',UInt16,queue_size=10).publish(rateNom)
				self.pub_num(i)
				time.sleep(1)

		if seq == 14:
			for i in range(100):
				rospy.Publisher('robot/joint_state_publish_rate',UInt16,queue_size=10).publish(rateNom)
				self.pub_num(i)
				time.sleep(1)
		print("sending", seq)
		#self.pub_cmd(seq)
		self.pub_cmd(seq)

	def rx_audio_duration(self,data):
		self.audio_duration = data.data

	def rx_nav_button(self,data):
		print('Rx: ' + data.data)
		self.finished = data.data.startswith("Button 'OK'")

if __name__ == '__main__':
	"Wheel value: 20"
	"Wheel value: 20"
	listener()
	rospy.spin()
