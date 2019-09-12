#! /usr/bin/env python
#Script handles module 3 for the slider and communication to localhost
import os
import rospy
import navigator
import math
import intera_interface 
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Lights,
    Cuff,
    RobotParams,
)
import go_to
import waypoint
import time
import threading
import zeroG
import gripper_cuff
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Int64
from std_msgs.msg import Float32
from std_msgs.msg import Bool
module_number = 3
sequence_msg = Int32()
sequence_msg.data = 0
unique_msg = String()
unique_msg.data = ''	
sequence_msg = Int32()
sequence_msg.data = -1
int_msg = Int32()
int_msg.data = 0
float_msg = Float32()
float_msg.data = 0.0
bool_msg = Bool()
bool_msg.data = True
slideDelay_msg = Int32()
slideDelay_msg.data = 0
slideCount_msg = Int64()
slideCount_msg.data = 0
slideLED_msg = Int32()
slideLED_msg.data = 0
#rosrun rosserial_python serial_node.py /dev/ttyUSB0
#rosrun rosserial_python serial_node.py /dev/ttyACM*

class listener():
	def __init__(self):
		self.photoFlag = False
		self.conveyorFlag = False
		self.slideEvent = True
		self.slideClose = False
		self.tempString = ''
		self.sawyerStatus = 0
		self.sequenceNum = 0

		self.moveCounter = 0
		self.time = 0

		self.userBoolFlag = True
		self.userMovedBoxes = 0
		self.iframe_sequence_pub = rospy.Publisher('/iframe_sequence', Int32, queue_size = 10)
		self.unique_input_pub = rospy.Publisher('/unique_input', String, queue_size = 10)
		self.slideDelay_pub = rospy.Publisher('/slider_delay', Int32, queue_size = 2)
		self.slideInput_pub = rospy.Publisher('/slider_count', Int64, queue_size = 2)
		self.slideLED_pub = rospy.Publisher('/slider_led', Int32, queue_size = 2)
		self.sawyer_pub = rospy.Publisher('/sawyer_sequence', Int32, queue_size = 2)
		self.sawyerMode_pub = rospy.Publisher('/sawyer_mode', Int32, queue_size = 2)
		self.sawyerTime_pub = rospy.Publisher('/sawyer_timer', Float32, queue_size = 2)

		self.conveyorMode_pub = rospy.Publisher('/conveyor_mode', Int32, queue_size = 2)
		self.conveyorSpeed_pub = rospy.Publisher('/conveyor_speed', Int32, queue_size = 2)
		self.conveyorTime_pub = rospy.Publisher('/conveyor_time', Int32, queue_size = 2)
		self.conveyorControl_pub = rospy.Publisher('/conveyor_control', Bool, queue_size = 1)
		self.conveyorDelay_pub = rospy.Publisher('/conveyor_delay', Int32, queue_size = 1)

		rospy.Subscriber('/photo_output', Bool, self.photoInterruptor_callback)
		rospy.Subscriber('/sawyer_slider_return', Int32, self.return_callback)
		rospy.Subscriber('/bot_sequence', Int32, self.callback)
		rospy.Subscriber('/save_info', String, self.save_info_callback)
		rospy.Subscriber('/slide_output', Int32, self.slideCallback)
		rospy.Subscriber('/conveyor_output', Int32, self.conveyorCallback)
		rospy.Subscriber('/user_input', String, self.user_inputCallback)
		rospy.init_node('peripheral_node', anonymous=True)

		self.correct_answer = ['Move conveyor belt', 'Stop conveyor belt when box passes the light sensor', 'Move arm down', \
		'Close robot arm gripper', 'Move arm up','Move arm to drop off location', 'Move arm down', 'Open robot arm gripper', \
		'Move arm up', 'Move arm to conveyor pickup location']
		# #highest speed
		# slideDelay_msg.data = 200
		# self.slideDelay_pub.publish(slideDelay_msg)
		# #roughly 1/4 travel distance. 30000 counts. 110k is roughly full length. Positive is towards end of stroke length. Negative is towards the motor
		# slideCount_msg.data = -slideDistance_count
		# self.slideInput_pub.publish(slideCount_msg)
		# while(self.slideEvent):
		# 	rospy.sleep(1)
		# self.slideEvent = True
		# slideLED_msg.data = 1
		# self.slideLED_pub.publish(slideLED_msg)
		# rospy.sleep(2)
		# slideLED_msg.data = 0
		# self.slideLED_pub.publish(slideLED_msg)
		# self.slideDelay_pub.publish(slideDelay_msg)
		# self.slideSequence() #calls 0
		int_msg.data = 0
		self.conveyorSpeed_pub.publish(int_msg)
		#stop time is measured in ms
		int_msg.data = 2000
		self.conveyorTime_pub.publish(int_msg)
		self.conveyorModePub(0)

		rospy.spin()

	def conveyorModePub(self, data):
		int_msg.data = data
		self.conveyorMode_pub.publish(int_msg)
		rospy.sleep(0.1)
		return

	def conveyorSpeedPub(self, data):
		int_msg.data = data
		self.conveyorSpeed_pub.publish(int_msg)
		rospy.sleep(0.1)
		return

	def conveyorTimePub(self, data):
		int_msg.data = data
		self.conveyorTime_pub.publish(int_msg)
		rospy.sleep(0.1)
		return

	def conveyorControlPub(self, data): 
		bool_msg.data = data
		self.conveyorControl_pub.publish(bool_msg)
		self.conveyorFlag = False
		rospy.sleep(0.1)
		return

	def conveyorDelayPub(self, data):
		int_msg.data = data
		self.conveyorDelay_pub.publish(int_msg)
		rospy.sleep(0.1)
		return

	def iframePub(self, data):
		int_msg.data = data
		self.iframe_sequence_pub.publish(int_msg)
		rospy.sleep(0.1)
		return

	def sawyerPub(self, data):
		int_msg.data = data
		self.sawyer_pub.publish(int_msg)
		self.sawyerStatus = False
		rospy.sleep(0.1)
		return

	def user_inputCallback(self, data):
		rospy.sleep(0.1)
		print(data.data)
		if(self.sequenceNum == 4):
			print(data.data)
			self.conveyorTimePub(int(float(data.data)*1000))
			self.conveyorModePub(3)
			self.conveyorSpeedPub(4)
			self.conveyorControlPub(False)
			self.sawyerPub(0)

			while(self.conveyorFlag != True or self.sawyerStatus != True):
				rospy.sleep(0.01)

			self.sawyerPub(1)
			rospy.sleep(2)
			self.iframePub(4)
			rospy.sleep(0.1)
			self.sawyerPub(0)
			self.sawyerPub(101)


		elif(self.sequenceNum == 5):
			#same as 4 with different speed
			self.conveyorTimePub(int(float(data.data)*1000))
			self.conveyorModePub(3)
			self.conveyorSpeedPub(8)
			self.conveyorControlPub(False)
			self.sawyerPub(0)

			while(self.conveyorFlag != True or self.sawyerStatus != True):
				rospy.sleep(0.01)

			self.sawyerPub(1)
			rospy.sleep(2)
			self.iframePub(5)
			rospy.sleep(0.1)
			self.sawyerPub(0)
			self.sawyerPub(101)

		elif(self.sequenceNum == 6 or self.sequenceNum == 7):
			self.userMovedBoxes = int(data.data)
			while(self.sawyerStatus != True):
				pass

			self.sawyerPub(0)

			while(self.sawyerStatus != True):
				pass

			# grab box
			self.sawyerPub(1)

			while(self.sawyerStatus != True):
				pass
			
			# move box onto table
			self.sawyerPub(3)

			while(self.sawyerStatus != True):
				pass

			# go back over conveyor belt
			self.sawyerPub(0)

			if(self.userMovedBoxes == 2):
				# proceed in script
				self.userBoolFlag = False
				self.iframePub(6)

			elif(self.userMovedBoxes == 3):
				# proceed to next step
				self.userBoolFlag = False
				self.iframePub(7)
		
		elif(self.sequenceNum == 9):
			print(data.data)
			temp_string = data.data




		elif(self.sequenceNum == 10):
			print(data.data)
			print(temp_string)
			temp_string = data.data
			temp_string.split('\n')
			if(temp_string == self.correct_answer):
				self.iframePub(10)

			elif(length(temp_string) != 10):
				print("wrong number of actions")
				self.iframePub(12)

			else:
				self.iframePub(13) 
				if(temp_string[0] != 'Move conveyor belt'):
					print("first command should be move conveyor belt")

				elif(temp_string[1] != 'Stop conveyor belt when box passes the light sensor'):
					print('second command: Stop conveyor belt when box passes the light sensor')

				elif(temp_string[2] != 'Move arm down'): 
					print('third command: Move arm down')

				elif(temp_string[3] != 'Close robot arm gripper'):
					print('fourth command: Close robot arm gripper')

				elif(temp_string[4] != 'Move arm up'):
					print("fifth command: Move arm up")

				elif(temp_string[5] != 'Move arm to drop off location'):
					print("sixth command: Move arm to drop off location")

				elif(temp_string[6] != 'Move arm down'):
					print("seventh command: Move arm down")

				elif(temp_string[7] != 'Open robot arm gripper'):
					print("eigth command: Open robot arm gripper")

				elif(temp_string[8] != 'Move arm up'):
					print("ninth command: Move arm up")

				elif(temp_string[9] != 'Move arm to conveyor pickup location'):
					print("10th command: Move arm to conveyor pickup location")




	def photoInterruptor_callback(self, data):
		# False for box is ready to pick up. True for when box is not.
		self.photoFlag = data.data

	def slideSequence(self):
		#roughly 1/4 travel distance. Needs calibrate
		slideCount_msg.data = -slideCount_msg.data
		self.slideInput_pub.publish(slideCount_msg)
		while(self.slideEvent):
			rospy.sleep(1)
		if(slideCount_msg.data > 0):
			# self.moveCounter = self.moveCounter + 1
			# if(self.moveCounter == 1):
			# 	self.moveCounter = 0
			sequence_msg.data = sequence_msg.data + 1
			self.sawyer_pub.publish(sequence_msg)
			rospy.sleep(0.1)
			slideLED_msg.data = 1
			self.slideLED_pub.publish(slideLED_msg)
		self.slideEvent = True
		rospy.sleep(3)
		slideLED_msg.data = 0
		self.slideLED_pub.publish(slideLED_msg)


	def save_info_callback(self,data):
		if data.data == 'time_start':
			self.old_time = time.time()

		elif data.data == 'time_end':
			self.new_time = time.time()
			self.time_elapsed = self.new_time - self.old_time
			self.file = open(self.file_path, 'a')
			self.file.write(str(self.time_elapsed) + '\r\n')
			self.file.close()


		else:
			self.file = open(self.file_path, 'a')
			self.file.write(str(data.data) + '\r\n')
			self.file.close()

	def slideCallback(self, data):
		if(data.data == 1):
			self.slideEvent = False
			if(slideCount_msg.data > 0):
				self.slideClose = True

			else:
				self.slideClose = False

	def conveyorCallback(self,data):
		if(data.data == 1):

			self.conveyorFlag = True
		else:
			self.conveyorFlag = False
		# if(conveyorFlag):
		# 	#do something
		# 	rospy.sleep(0.1)
		# else:
		# 	rospy.sleep(0.1)

	def return_callback(self, data):
		print("sawyer_slider return heard: " + str(data.data))
		if(data.data == 0):
			self.sawyerStatus = False
			#Sawyer is not ready
		else: 
			self.sawyerStatus = True
			#Sawyer is ready



	def callback(self,data):
		rospy.loginfo(rospy.get_caller_id() + "I heard" + str(data.data))
		self.sequenceNum = data.data
		if(data.data == 1):
			rospy.sleep(0.1)
			#set conveyor to automatically run
			self.conveyorSpeedPub(10)
			self.conveyorTimePub(2000)
			self.conveyorModePub(1)
			rospy.sleep(3)
			self.iframePub(1)

		elif(data.data == 2):
			rospy.sleep(0.1)
			#set conveyor to stop for each box while maintaining the same speed
			self.conveyorModePub(3)
			self.conveyorDelayPub(0)
			self.conveyorControlPub(True)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)
			
			self.sawyerPub(0)
			self.photoFlag = False

			while(self.sawyerStatus != True or self.photoFlag != True):
				rospy.sleep(0.01)
			
			self.sawyerPub(1)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)

			self.sawyerPub(3)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)

			self.sawyerPub(0)

			self.iframePub(2)

		elif(data.data == 3):
			self.conveyorModePub(3)
			self.conveyorSpeedPub(4)
			self.conveyorDelayPub(800)
			self.conveyorControlPub(False)
			self.photoFlag = False
			self.conveyorFlag = False

			while(self.sawyerStatus != True or self.conveyorFlag != True):
				rospy.sleep(0.01)

			rospy.sleep(0.8)
			#Go down to pick up
			self.sawyerPub(1)

			while(self.sawyerStatus != True):
				rospy.sleep(0.1)

			self.sawyerPub(0)
			self.sawyerPub(101)
			self.iframePub(3)

		elif(data.data == 4):
			#in progress
			while(self.sawyerStatus != True):
				rospy.sleep(0.01)

			self.sawyerPub(0)
		
		elif(data.data == 5):
			rospy.sleep(0.1)

		elif(data.data == 6):
			self.userBoolFlag = True
			while(self.userBoolFlag):
				self.conveyorModePub(3)
				self.conveyorSpeedPub(10)
				self.conveyorTimePub(2000)
				self.conveyorControlPub(True)
				while(self.conveyorFlag != True):
					pass
				rospy.sleep(4)

		elif(data.data == 7):
			self.userBoolFlag = True
			while(self.userBoolFlag):
				self.conveyorModePub(3)
				self.conveyorSpeedPub(10)
				self.conveyorTimePub(2000)
				self.conveyorControlPub(True)
				while(self.conveyorFlag != True):
					pass
				rospy.sleep(4)

		elif(data.data == 8):
			#in progress
			#Conveyor + Sawyer sequence
			#Change conveyor mode
			self.sawyerPub(0)

			self.conveyorModePub(3)
			self.conveyorSpeedPub(8)
			self.conveyorTimePub(2000)
			self.conveyorControlPub(True)


			while(self.conveyorFlag != True or self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			if(self.sawyerStatus):
				self.sawyerPub(1)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			if(self.sawyerStatus):
				self.sawyerPub(3)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			self.sawyerPub(0)

			self.iframePub(8)

			
		elif(data.data == 9):
			rospy.sleep(0.1)
			self.sawyerPub(0)

			self.conveyorModePub(3)
			self.conveyorSpeedPub(8)
			self.conveyorTimePub(2000)
			self.conveyorControlPub(True)


			while(self.conveyorFlag != True or self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			if(self.sawyerStatus):
				self.sawyerPub(1)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			if(self.sawyerStatus):
				self.sawyerPub(3)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			self.sawyerPub(0)
			
		elif(data.data == 10):
			rospy.sleep(0.1)


		elif(data.data == 11):
			rospy.sleep(0.1)
			self.sawyerPub(0)

			self.conveyorModePub(3)
			self.conveyorSpeedPub(8)
			self.conveyorTimePub(2000)
			self.conveyorControlPub(True)


			while(self.conveyorFlag != True or self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			if(self.sawyerStatus):
				self.sawyerPub(1)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			if(self.sawyerStatus):
				self.sawyerPub(3)

			while(self.sawyerStatus != True):
				rospy.sleep(0.01)
				pass

			self.sawyerPub(0)

			self.iframePub(11)

		elif(data.data == 12):
			rospy,sleep(0.1)


		# if (data.data == 0):
		# 	rospy.sleep(1)
		# 	int_msg.data = 0
		# 	self.iframe_sequence_pub.publish(int_msg)

		# elif(data.data == 1):
		# 	rospy.sleep(0.1)
		# 	self.moveCounter = 0
		# 	self.slideSequence()
		# 	rospy.sleep(0.1)
		# 	int_msg.data = 1
		# 	self.iframe_sequence_pub.publish(int_msg)
		# 	rospy.sleep(0.1)
		# 	self.slideSequence() #calls 1
		# 	self.slideSequence()


		# elif(data.data == 2):
		# 	rospy.sleep(0.1)
		# 	self.slideSequence() #calls 2
		# 	self.slideSequence()
		# 	rospy.sleep(0.1)
		# 	int_msg.data = 2
		# 	self.iframe_sequence_pub.publish(int_msg)
		# 	rospy.sleep(0.1)
		# 	self.slideSequence() #calls 3

		# elif(data.data == 3):
		# 	rospy.sleep(0.1)
		# 	slideDelay_msg.data = 800
		# 	self.slideDelay_pub.publish(slideDelay_msg)
		# 	self.slideSequence()
		# 	sequence_msg.data = 4
		# 	self.sawyer_pub.publish(sequence_msg) #calls 4
		# 	self.moveCounter = 0			rospy.sleep(0.1)
		# 	self.slideSequence() #calls 5
		# 	rospy.sleep(0.1)
		# 	int_msg.data = 3
		# 	self.iframe_sequence_pub.publish(int_msg)

		# elif(data.data == 4):
		# 	rospy.sleep(0.1)
		# 	self.slideSequence()
		# 	self.slideSequence() #calls 6
		# 	self.slideSequence()
		# 	rospy.sleep(0.1)
		# 	int_msg.data = 4
		# 	self.iframe_sequence_pub.publish(int_msg)



if __name__ == '__main__':
	empty_angle_arg = go_to.joint_angle_arg()
	empty_cartesian_arg = go_to.cartesian_pose_arg()

	#variables to be used
	init_cartesian_arg = go_to.cartesian_pose_arg(joint_angles = [-0.438147460938,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625])
	init_joint_arg = go_to.joint_angle_arg(joint_angles =[-0.438147460938,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625] )
	
	joint_buttons = go_to.joint_angle_arg(joint_angles = [-0.0393427734375,-0.71621875,0.022359375,1.811921875,-0.116017578125,2.77036035156,-4.48708789063], speed_ratio = 0.9, accel_ratio = 0.5)

	above_carriage_joint_arg = go_to.joint_angle_arg(joint_angles = [-1.48183984375,0.371584960937,-1.21172949219,-1.08480664063,-2.05691894531,-1.68737109375,-0.3142734375])
	#position: [0.465337449515,-0.803569147609,0.228977848984]
	#orientation: [0.708525180417,0.705653350811,0.00665425574536,-0.0010668106934]

	above_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.429515625,-0.492512695313,-1.25894921875,1.58936425781,-2.12879492187,-1.21486914063,2.99199902344])
	rotated_above_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.239127929687,-0.440267578125,-0.618359375,2.16329882812,1.75038769531,0.603390625,0.984127929687])
	into_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-2.04193945312,0.905513671875,-2.16442285156,-2.058984375,0.59423828125,1.62292871094,-1.11868945313])

	into_second_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.89046875,0.309813476563,-1.65483398437,1.14237109375,-1.32817382812,-1.48642578125,2.85633007813])
	above_second_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.941543945312,0.193456054687,-1.71885546875,1.12710351562,-1.39799609375,-1.59924121094,2.80375683594])

	above_second_box_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.729661132813,0.12510546875,-1.78256054688,1.094546875,-1.35895410156,-1.71990722656,4.66472753906])
	above_third_box_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-1.95087109375,-1.03140625,-2.33362109375,-1.74688183594,-0.453393554687,-1.055703125,3.88])
	above_fourth_box_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-1.51795410156,0.228708984375,-0.52629296875,-0.585658203125,0.442354492188,1.78565820312,0.4718984375])

	zeroG_endpoint_constraints = zeroG.constrained_arg(orientation_z=True, in_endpoint_frame = True)
	zeroG_all_constraints = zeroG.constrained_arg(orientation_x = False, orientation_y = False, orientation_z = False, position_x = False, position_y = False, position_z = False)
	zeroG_ori_constraints = zeroG.constrained_arg(orientation_x=False, orientation_y=False, orientation_z=False, position_x=True, position_y=True, position_z=True)
	zeroG_pos_constraints = zeroG.constrained_arg(orientation_x=True, orientation_y=True, orientation_z=True, position_x=False, position_y=False, position_z=False)
	zeroG_xyplane = zeroG.constrained_arg(plane_horizontal = True)
	slideDistance_count = 25000
	z_table = 0.127974138485
	gripper_to_base_length = .08
	box1_height = 0.110
	box2_height = 0.110
	box3_height = 0.110
	bin_depth = 0.005
	bin_height = 0.085
	half_gripper_width = 0.015
	gripper_width = 0.03
	height_slider = 0.10100371049
	carriage_detph = .025
listener();