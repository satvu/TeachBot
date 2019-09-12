#! /usr/bin/env python

## IMPORTS ##
import rospy
import navigator
import math
import intera_interface 
import go_to
import waypoint
import threading
import zeroG
import pexpect
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

class listener():
	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Publishing topics
		self.iframe_sequence_pub = rospy.Publisher('/iframe_sequence', Int32, queue_size = 10)
		self.unique_input_pub = rospy.Publisher('/unique_input', String, queue_size = 10)
		self.numeric_pub = rospy.Publisher('/numeric_input', Float64, queue_size=10)
		self.array_pub = rospy.Publisher('/array_input', Float64MultiArray, queue_size=10)
		self.zeroG_pub = rospy.Publisher('/zeroG_topic', String, queue_size = 10)
		
		# Subscribing topics
		rospy.Subscriber('/bot_sequence', Int32, self.rx_command)
		rospy.Subscriber('/audio_duration', Float64, self.rx_audio_duration)

		# Global Vars
		self.audio_duration = 0;
		
		# Initialize Sawyer
		rospy.init_node('Sawyer_Sparrow_comm_node', anonymous=True)
		go_to.joint_angles(empty_angle_arg)
		self.limb = intera_interface.Limb('right')
		navigator.right()

	## HELPER FUNCTIONS ##
	# Returns current joint angle
	def read_angle(self, joint):
		arm = 'right'
		limb = intera_interface.Limb(arm)
		return  limb.joint_angle(joint)

	# Publishes message to ROSTopic to be receieved by JavaScript
	def pub_cmd(self, counter):
		int_msg = Int32()
		int_msg.data = counter
		self.iframe_sequence_pub.publish(int_msg)
		rospy.loginfo("I sent: " + str(counter))
	def pub_num(self, num):
		msg = Float64()
		msg.data = num
		self.numeric_pub.publish(msg)
		rospy.loginfo("I sent: " + str(num))
	def pub_arr(self, arr):
		msg = Float64MultiArray()
		msg.data = arr
		self.array_pub.publish(msg)
		rospy.loginfo("I sent: " + str(arr))

	# Allows user to move arm in zero G mode
	def user_move(self):
		startPose = self.limb.endpoint_pose()
		zeroG.constrained(zeroG_no_constraints)
		while(self.endpoints_equal(startPose,self.limb.endpoint_pose(),tol=0.01)):	# Wait for user to begin moving arm
			pass
		rospy.sleep(0.5)
		while(not self.endpoints_equal(startPose,self.limb.endpoint_pose())):		# Wait for user to stop moving arm
			startPose = self.limb.endpoint_pose()
			rospy.sleep(1)
		zeroG.constrained(zeroG_all_constraints)

	# Returns true if two positions equal each other or are at least within a given tolerance
	def endpoints_equal(self, pose1, pose2, tol=0):
		return abs(pose1['position'].x-pose2['position'].x)<=tol \
			   and abs(pose1['position'].y-pose2['position'].y)<=tol \
			   and abs(pose1['position'].z-pose2['position'].z)<=tol \
			   and abs(pose1['orientation'].x-pose2['orientation'].x)<=tol \
			   and abs(pose1['orientation'].y-pose2['orientation'].y)<=tol \
			   and abs(pose1['orientation'].z-pose2['orientation'].z)<=tol \
			   and abs(pose1['orientation'].w-pose2['orientation'].w)<=tol

	# Publishes message to ROSTopic to be receieved by JavaScript
	def pub_msg(self, counter):
		int_msg = Int32()
		int_msg.data = counter
		self.iframe_sequence_pub.publish(int_msg)

	## LISTENER EVENTS ##
	# Runs whenever message is received on main ROSTopic from JavaScript
	def rx_command(self, data):
		seq = data.data
		rospy.loginfo(rospy.get_caller_id() + "I heard: " + str(seq))

		if (seq==0):
			# TODO: Move robot, control peripherals, etc.
		
		# TODO: Add other cases

		# Advance in JS
		self.pub_msg(seq)

	def rx_audio_duration(self,data):
		self.audio_duration = data.data

## DEFINE IMPORTANT CONSTANTS ##
if __name__ == '__main__':
	# Turn off zero G mode
	child = pexpect.spawn('rostopic pub -r 10 /robot/limb/right/suppress_cuff_interaction std_msgs/Empty')

	# Commonly used objects
	empty_angle_arg = go_to.joint_angle_arg()
	zero_angle_arg = go_to.joint_angle_arg(joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed_ratio = .9, accel_ratio = .5)
	joint_buttons = go_to.joint_angle_arg(joint_angles = [-0.0393427734375,-0.71621875,0.022359375,1.811921875,-0.116017578125,2.77036035156,-4.48708789063], speed_ratio = 0.9, accel_ratio = 0.5)
	zeroG_all_constraints = zeroG.constrained_arg(orientation_x=False, orientation_y=False, orientation_z=False, position_x=False, position_y=False, position_z=False)
	zeroG_no_constraints = zeroG.constrained_arg(orientation_x=False, orientation_y=False, orientation_z=False, position_x=True, position_y=True, position_z=True)

	# TODO: Put more constants here

	listener()