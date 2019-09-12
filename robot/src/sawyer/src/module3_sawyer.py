#! /usr/bin/env python
#testing various python functions and ros communication via rosbridge
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
module_number = 3
sequence_msg = Int32()
sequence_msg.data = 0
grip_msg = Int32()
grip_msg.data = 1
unique_msg = String()
unique_msg.data = ''	
status_msg = Int32()
status_msg.data = 0
int_msg = Int32()
int_msg.data = 0
#rosrun rosserial_python serial_node.py /dev/ttyUSB0
class cartPose(object):
	def __init__(self, pos_x=None, pos_y=None, pos_z=None, ori_x=None, ori_y=None, ori_z=None, ori_w = None):
		self.position_x = pos_x
		self.position_y = pos_y
		self.position_z = pos_z
		self.orientation_x = ori_x
		self.orientation_y = ori_y
		self.orientation_z = ori_z
		self.orientation_w = ori_w

class jointAngles(object):
	def __init__(self, j0 = None, j1 = None, j2 =None, j3=None, j4 = None, j5 = None, j6 = None):
		self.j0 = j0
		self.j1 = j1
		self.j2 = j2
		self.j3 = j3
		self.j4 = j4
		self.j5 = j5
		self.j6 = j6

class listener():
	def __init__(self):
		self.slideEvent = True
		self.slideClose = False
		self.movedBoxes = 0
		self.ready = False
		self.last_msg = Int32()
		self.zeroG_pub = rospy.Publisher('/zeroG_topic', String, queue_size = 10)
		self.gripper_pub = rospy.Publisher('/gripper_control', Int32, queue_size = 10)
		self.lights_pub = rospy.Publisher('/cuff_light', String, queue_size = 10)
		self.return_pub = rospy.Publisher('/sawyer_slider_return', Int32, queue_size = 2)
		rospy.Subscriber('/sawyer_sequence', Int32, self.callback)
		#rospy.Subscriber('/sawyer_mode', Int32, self.sawyerMode_callback)
		#rospy.Subscriber('/sawyer_time', Float32, self.sawyerTime_callback)
		rospy.init_node('Sawyer_node', anonymous=True)

		self.limb = intera_interface.Limb('right')
		self._gripper = get_current_gripper_interface()
		self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)

		self.startPose = self.limb.endpoint_pose() 
		self.startPose_container = cartPose()
		self.startPose_arg = go_to.cartesian_pose_arg()
		self.startJointAngles = self.limb.joint_angles()
		self.newCartPose_container = cartPose()
		self.newJointAngles_arg = go_to.joint_angle_arg()
		self.newCartPose_arg = go_to.cartesian_pose_arg()
		self.newCartPose_arg2 = go_to.cartesian_pose_arg()
		self.newCartPose = cartPose()
		self.newJointAngles = jointAngles()
		self.waypoint_init_container = cartPose()
		self.waypoint_init_arg = go_to.cartesian_pose_arg()
		#Check if gripper is attached properly
		self.grip = gripper_cuff.GripperConnect()
		#Open gripper
		grip_msg.data = 1
		self.gripper_pub.publish(grip_msg)
		if(go_to.joint_angles(empty_angle_arg)):
			int_msg.data = 1
			self.return_pub.publish(int_msg)
		# go_to.joint_angles(step2_conveyor_arg)
		# self.startPose = self.limb.endpoint_pose() 
		# self.startPose_container = self.pose_to_cartclass(self.startPose, self.startPose_container)
		# self.startPose_arg = self.cartclass_to_cartarg(self.startPose_container, self.startPose_arg)
		# self.startPose_container.position_z = z_conveyor + box_height - gripper_width
		# self.newCartPose_arg = self.cartclass_to_cartarg(self.startPose_container, self.newCartPose_arg)
		# if(go_to.cartesian_pose(self.newCartPose_arg)):
		# 	grip_msg.data = 0 
		# 	self.gripper_pub.publish(grip_msg)

		# rospy.sleep(1)
		# go_to.cartesian_pose(self.startPose_arg)
		# go_to.cartesian_pose(self.newCartPose_arg)
		# grip_msg.data = 1
		# self.gripper_pub.publish(grip_msg)
		# go_to.cartesian_pose(self.startPose_arg)
		navigator.right()

	def pose_to_cartclass(self, pose, cartclass):
		cartclass.position_x = pose['position'].x
		cartclass.position_y = pose['position'].y
		cartclass.position_z = pose['position'].z
		cartclass.orientation_x = pose['orientation'].x
		cartclass.orientation_y = pose['orientation'].y
		cartclass.orientation_z = pose['orientation'].z
		cartclass.orientation_w = pose['orientation'].w
		return cartclass

	def cartclass_to_cartarg(self, cartclass, cartarg):
		cartarg = go_to.cartesian_pose_arg(position = [cartclass.position_x, cartclass.position_y, cartclass.position_z],
			orientation = [cartclass.orientation_x, cartclass.orientation_y, cartclass.orientation_z, cartclass.orientation_w])
		return cartarg

	def limbangles_to_angleclass(self, limbangles, angleclass):
		angleclass.j0 = limbangles['right_j0']
		angleclass.j1 = limbangles['right_j1']
		angleclass.j2 = limbangles['right_j2']
		angleclass.j3 = limbangles['right_j3']
		angleclass.j4 = limbangles['right_j4']
		angleclass.j5 = limbangles['right_j5']
		angleclass.j6 = limbangles['right_j6']
		return angleclass

	def sawyer_motion_sequence(self, msg):
		int_msg.data = 0
		self.return_pub.publish(int_msg)
		print("I heard: " + str(msg.data))

		#if/elseif sequence of motion
		if(msg.data == 0):
			#go above conveyor belt
			go_to.joint_angles(j_abv_conveyor_arg)

		elif(msg.data == 1):
			#Go to pick up level, grip, and pick up
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)
			self.startPose = self.limb.endpoint_pose() 
			self.startPose_container = self.pose_to_cartclass(self.startPose, self.startPose_container)
			self.startPose_arg = self.cartclass_to_cartarg(self.startPose_container, self.startPose_arg)

			self.newCartPose_container = self.startPose_container
			
			if(self.last_msg == 0):
				self.newCartPose_container.position_z = z_conveyor + box_height - gripper_width
			
			elif(self.last_msg == 3):
				self.newCartPose_container.position_z = z_table + box_height - gripper_width
			
			elif(self.last_msg == 4):
				self.newCartPose_container.position_z = z_slider + box_height - gripper_width
			
			self.newCartPose_arg = self.cartclass_to_cartarg(self.newCartPose_container, self.newCartPose_arg)
			if(go_to.cartesian_pose(self.newCartPose_arg)):
				grip_msg.data = 0 
				self.gripper_pub.publish(grip_msg)
				rospy.sleep(0.5)
				go_to.cartesian_pose(self.startPose_arg)

		elif(msg.data == 2):
			#put object back down
			self.startPose = self.limb.endpoint_pose() 
			self.startPose_container = self.pose_to_cartclass(self.startPose, self.startPose_container)
			
			if(self.last_msg == 1):
				self.startPose_container.position_z = z_conveyor + box_height - gripper_width - 0.01
			
			elif(self.last_msg == 3):
				self.startPose_container.position_z = z_table + box_height - gripper_width - 0.01

			elif(self.last_msg == 4):
				self.newCartPose_container.position_z = z_slider + box_height - gripper_width

			self.startPose_arg = self.cartclass_to_cartarg(self.startPose_container, self.startPose_arg)
			if(go_to.cartesian_pose(self.startPose_arg)):
				grip_msg.data = 1
				self.gripper_pub.publish(grip_msg)

		elif(msg.data == 3):
			#put object on table. increment up number of boxes on the table. Moves boxes based on how many there are on the table for placement (Rows of 4)
			if(go_to.joint_angles(init_boxtable_arg)):
				pass

			self.startPose = self.limb.endpoint_pose()
			self.startPose_container = self.pose_to_cartclass(self.startPose, self.startPose_container)
			self.startPose_arg = self.cartclass_to_cartarg(self.startPose_container, self.startPose_arg)

			self.newCartPose_container = self.startPose_container
			self.newCartPose_container.position_x = self.newCartPose_container.position_x - ((self.movedBoxes%2)*(box_width+0.05)) - .15
			self.newCartPose_container.position_y = self.newCartPose_container.position_y - ((self.movedBoxes//2)*(box_length+.05))
			self.newCartPose_arg = self.cartclass_to_cartarg(self.newCartPose_container, self.newCartPose_arg)

			if(go_to.cartesian_pose(self.newCartPose_arg)):
				pass

			self.newCartPose_container.position_z = z_table + box_height - gripper_width
			self.newCartPose_arg2 = self.cartclass_to_cartarg(self.newCartPose_container, self.newCartPose_arg2)

			if(go_to.cartesian_pose(self.newCartPose_arg2)):
				pass

			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)
			if(go_to.cartesian_pose(self.newCartPose_arg)):
				pass

			self.movedBoxes = self.movedBoxes + 1


		elif(msg.data == 4):
			go_to.cartesian_pose(linear_slider_arg)

		#FOR USER INPUT SECTION:
		elif(msg.data == 5):
			pass

		elif(msg.data == 101):
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)
			
		#Tell module3.py that sawyer is ready
		int_msg.data = 1
		self.return_pub.publish(int_msg)
		self.last_msg = msg.data

	def angleclass_to_anglearg(self, angleclass, anglearg):
		anglearg = go_to.joint_angle_arg(join ,t_angles = [angleclass.j0, angleclass.j1, angleclass.j2, angleclass.j3, angleclass.j4, angleclass.j5, angleclass.j6])
		return anglearg

	def callback(self,data):
		rospy.loginfo(rospy.get_caller_id() + "I heard" + str(data.data))
		self.sawyer_motion_sequence(data)

		# if (data.data == 0):
		# 	rospy.sleep(1)

		# elif(data.data == 1):
		# 	rospy.sleep(0.1)
		# 	status_msg.data = 0
		# 	self.return_pub.publish(status_msg)
		# 	go_to.joint_angles(init_joint_arg)
		# 	go_to.joint_angles(above_carriage_joint_arg)
		# 	status_msg.data = 1
		# 	self.return_pub.publish(status_msg)

		# elif(data.data == 2):
		# 	rospy.sleep(0.1)
		# 	status_msg.data = 0
		# 	self.return_pub.publish(status_msg)
		# 	go_to.cartesian_pose(pickup_carriage_cart_arg)
		# 	grip_msg.data = 0
		# 	self.gripper_pub.publish(grip_msg)
		# 	go_to.cartesian_pose(above_carriage_cart_arg)
		# 	status_msg.data = 1
		# 	self.return_pub.publish(status_msg)

		# elif(data.data == 3):
		# 	#put the box back in the cart
		# 	rospy.sleep(0.1)
		# 	status_msg.data = 0
		# 	self.return_pub.publish(status_msg)
		# 	go_to.cartesian_pose(pickup_carriage_cart_arg)
		# 	grip_msg.data = 1
		# 	self.gripper_pub.publish(grip_msg)
		# 	go_to.cartesian_pose(above_carriage_cart_arg)
		# 	status_msg.data = 1
		# 	self.return_pub.publish(status_msg)

		# elif(data.data == 4):
		# 	#fail at pick up
		# 	rospy.sleep(0.1)
		# 	status_msg.data = 0
		# 	self.return_pub.publish(status_msg)
		# 	go_to.cartesian_pose(pickup_carriage_cart_arg)
		# 	grip_msg.data = 0
		# 	self.gripper_pub.publish(grip_msg)
		# 	go_to.cartesian_pose(above_carriage_cart_arg)
		# 	status_msg.data = 1
		# 	self.return_pub.publish(status_msg)
		# 	grip_msg.data = 1
		# 	self.gripper_pub.publish(grip_msg)

		# elif(data.data == 5):
		# 	rospy.sleep(0.1)

		# elif(data.data == 6):
		# 	rospy.sleep(0.1)
		# 	status_msg.data = 0
		# 	self.return_pub.publish(status_msg)
		# 	go_to.cartesian_pose(pickup_carriage_cart_arg)
		# 	grip_msg.data = 0
		# 	self.gripper_pub.publish(grip_msg)
		# 	go_to.cartesian_pose(above_carriage_cart_arg)
		# 	status_msg.data = 1
		# 	self.return_pub.publish(status_msg)






if __name__ == '__main__':
	empty_angle_arg = go_to.joint_angle_arg()
	empty_cartesian_arg = go_to.cartesian_pose_arg()

	#variables to be used
	init_cartesian_arg = go_to.cartesian_pose_arg(joint_angles = [-0.438147460938,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625])
	init_joint_arg = go_to.joint_angle_arg(joint_angles = [-0.33585546875,-0.379596679687,-1.60570117188,1.12274511719,-1.9501328125,-1.73130761719,1.83596582031])
	
	joint_buttons = go_to.joint_angle_arg(joint_angles = [-0.0393427734375,-0.71621875,0.022359375,1.811921875,-0.116017578125,2.77036035156,-4.48708789063], speed_ratio = 0.9, accel_ratio = 0.5)

	above_carriage_joint_arg = go_to.joint_angle_arg(joint_angles = [-1.20277050781,-0.0404033203125,-1.97000390625,0.374342773437,-1.24470507812,-1.71349023438,1.74451757812])
	above_carriage_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-1.20277050781,-0.0404033203125,-1.97000390625,0.374342773437,-1.24470507812,-1.71349023438,1.74451757812])
	pickup_carriage_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-1.14610644531,0.174032226563,-1.930796875,0.511778320313,-1.21001367188,-1.55299414063,1.68880664062])
	#position: [0.465337449515,-0.803569147609,0.228977848984]
	#orientation: [0.708525180417,0.705653350811,0.00665425574536,-0.0010668106934]
	step1_conveyor_arg = go_to.cartesian_pose_arg(joint_angles = [0.406725585938,-0.584879882813,-0.0846240234375,1.52141796875,0.56179296875,-0.817509765625,-0.129595703125])
	j_abv_conveyor_arg = go_to.joint_angle_arg(joint_angles = [-0.0522529296875,0.320400390625,-2.41225976562,0.325688476562,2.52433398438,1.56921972656,1.47036621094])

	c_abv_conveyor_arg = go_to.cartesian_pose_arg(joint_angles = [-0.0522529296875,0.320400390625,-2.41225976562,0.325688476562,2.52433398438,1.56921972656,1.47036621094])

	#add in the corner of the table.
	init_boxtable_arg = go_to.joint_angle_arg(joint_angles =[-0.155546875,-0.275838867188,-1.47023242188,1.48976367188,1.26449707031,1.45892089844,-3.00016210938])
	zeroG_endpoint_constraints = zeroG.constrained_arg(orientation_z=True, in_endpoint_frame = True)
	zeroG_all_constraints = zeroG.constrained_arg(orientation_x = False, orientation_y = False, orientation_z = False, position_x = False, position_y = False, position_z = False)
	zeroG_ori_constraints = zeroG.constrained_arg(orientation_x=False, orientation_y=False, orientation_z=False, position_x=True, position_y=True, position_z=True)
	zeroG_pos_constraints = zeroG.constrained_arg(orientation_x=True, orientation_y=True, orientation_z=True, position_x=False, position_y=False, position_z=False)
	zeroG_xyplane = zeroG.constrained_arg(plane_horizontal = True)
	
	slideDistance_count = 30000

	z_table = 0.11
	z_conveyor = -0.038
	gripper_to_base_length = .08

	box_height = 0.112
	box_width = 0.060
	box_length = 0.148

	bin_depth = 0.005
	bin_height = 0.085
	
	half_gripper_width = 0.015
	gripper_width = 0.03
	height_slider = 0.10100371049
	carriage_depth = .03
listener();