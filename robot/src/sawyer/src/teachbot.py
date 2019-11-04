#! /usr/bin/env python

## IMPORTS ##
# Basic
import rospy, actionlib, numpy, math
from pygame import mixer
import cv2
import apriltag
# Intera
import intera_interface
from std_msgs.msg import (
	Bool,
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16,
	Empty
)
import sensor_msgs
import intera_core_msgs

# Custom
from limb_plus import LimbPlus
from sawyer.msg import *
from sawyer.srv import *
from cv_bridge import CvBridge, CvBridgeError

# Action messages
import actionlib

class Module():
	FORCE2VELOCITY = {'right_j0': 0.06, 'right_j1': 0.06, 'right_j2': 0.4, 'right_j3': 0.2, 'right_j4': 1, 'right_j5': 0.9, 'right_j6': 2}
	VERBOSE = True
	JOINTS = 7
	BUTTON = {'back': 0, 'show': 1, 'circle': 2, 'square': 3, 'triangle': 4}	# Triangle is the X button. An earlier version of Sawyer had a triangle and Rethink never updated the SDK.
	CUFF_BUTTON = {'upper': 0, 'lower': 1}
	Z_TABLE = 0.12
	BOX_HEIGHT = 0.11
	BIN_HEIGHT = 0.005

	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Initialize Sawyer
		rospy.init_node('Sawyer_comm_node', anonymous=True)
		intera_interface.HeadDisplay().display_image('logo.png')

		# Publishing topics
		suppress_cuff_interaction = rospy.Publisher('/robot/limb/right/suppress_cuff_interaction', Empty, queue_size=1)
		self.position_topic = rospy.Publisher('/teachbot/position', JointInfo, queue_size=1)
		self.velocity_topic = rospy.Publisher('/teachbot/velocity', JointInfo, queue_size=1)
		self.effort_topic = rospy.Publisher('/teachbot/effort', JointInfo, queue_size=1)
		self.scroll_wheel_button_topic = rospy.Publisher('/teachbot/scroll_wheel_button_topic', Empty, queue_size = 10)
		self.command_complete_topic = rospy.Publisher('/teachbot/command_complete', Empty, queue_size=1)
		self.wheel_delta_topic = rospy.Publisher('/teachbot/wheel_delta', Int32, queue_size=10)
		self.clicked = rospy.Publisher('/teachbot/scroll_wheel_pressed', Bool, queue_size=10)
		self.endpoint_topic = rospy.Publisher('/teachbot/EndpointInfo', EndpointInfo, queue_size=10)

		# Subscribing topics
		rospy.Subscriber('/robot/joint_states', sensor_msgs.msg.JointState, self.forwardJointState)
		rospy.Subscriber('/teachbot/JointAngle', String, self.cb_joint_angle)
		rospy.Subscriber('/teachbot/JointImpedance', JointImpedance, self.cb_impedance)
		rospy.Subscriber('/teachbot/multiple_choice', Bool, self.cb_multiple_choice)
		rospy.Subscriber('/robot/limb/right/endpoint_state', intera_core_msgs.msg.EndpointState, self.forwardEndpointState)
		rospy.Subscriber('/teachbot/camera', Bool, self.cb_camera)
		#rospy.Subscriber('/robot/digital_io/right_lower_button/state', intera_core_msgs.msg.DigitalIOState, self.cb_cuff_lower)
		#rospy.Subscriber('/robot/digital_io/right_upper_button/state', intera_core_msgs.msg.DigitalIOState, self.cb_cuff_upper)

		# Service Servers
		rospy.Service('/teachbot/audio_duration', AudioDuration, self.rx_audio_duration)
		rospy.Service('/teachbot/wheel_subscription', ScrollWheelSubscription, self.cb_WheelSubscription)
		# Service Clients
		self.DevModeSrv = rospy.ServiceProxy('/teachbot/dev_mode', DevMode)

		# Actions
		self.CuffInteractionAct = actionlib.SimpleActionServer('/teachbot/CuffInteraction', CuffInteractionAction, execute_cb=self.cb_CuffInteraction, auto_start=True)
		self.GoToJointAnglesAct = actionlib.SimpleActionServer('/teachbot/GoToJointAngles', GoToJointAnglesAction, execute_cb=self.cb_GoToJointAngles, auto_start=True)
		self.JointMoveAct = actionlib.SimpleActionServer('/teachbot/JointMove', JointMoveAction, execute_cb=self.cb_joint_move, auto_start=True)
		self.InteractionControlAct = actionlib.SimpleActionServer('/teachbot/InteractionControl', InteractionControlAction, execute_cb=self.cb_interaction, auto_start=True)
		self.AdjustPoseToAct = actionlib.SimpleActionServer('/teachbot/AdjustPoseTo', AdjustPoseToAction, execute_cb=self.cb_AdjustPoseTo, auto_start=True)
		self.GripperAct = actionlib.SimpleActionServer('/teachbot/Gripper', GripperAction, execute_cb=self.cb_Gripper, auto_start=True)
		self.GoToCartesianPoseAct = actionlib.SimpleActionServer('/teachbot/GoToCartesianPose', GoToCartesianPoseAction, execute_cb=self.cb_GoToCartesianPose, auto_start=True)
		self.PickUpBoxAct = actionlib.SimpleActionServer('/teachbot/PickUpBox', PickUpBoxAction, execute_cb=self.cb_PickUpBox, auto_start=True)
		self.AdjustPoseByAct = actionlib.SimpleActionServer('/teachbot/AdjustPoseBy', AdjustPoseByAction, execute_cb=self.cb_AdjustPoseBy, auto_start=True)
		self.HighTwoAct = actionlib.SimpleActionServer('/teachbot/HighTwo', HighTwoAction, execute_cb=self.cb_HighTwo, auto_start=True)

		# Global Vars
		self.audio_duration = 0
		self.finished = False
		self.startPos = 0
		self.devMode = False
		self.seqArr = []

		self.p_command = lambda self: self.pub_num(180/math.pi*self.limb.joint_angle(shoulder))
		wheel = self.finished
		b_less = lambda self : self.limb.joint_angle(shoulder)<point_b[0]
		b_more = lambda self : self.limb.joint_angle(shoulder)>point_b[0]
		dot_2 = lambda self : self.limb.joint_angle(shoulder)<joint_dot_2[0]
		#endpoint = lambda self : self.endpoints_equal(self.limb.endpoint_pose(),dot_3,tol) or self.finished

		# Limb
		self.limb = LimbPlus()
		self.limb.go_to_joint_angles()

		# Lights
		self.lights = intera_interface.Lights()
		for light in self.lights.list_all_lights():
			self.lights.set_light_state(light,False)

		# Navigator
		self.navigator = intera_interface.Navigator()
		self.multi_choice_callback_ids = self.BUTTON.copy()
		for key in self.multi_choice_callback_ids:
			self.multi_choice_callback_ids[key] = -1
		self.wheel_callback_id = -1
		self.wheel_state = self.navigator.get_wheel_state('right_wheel')
		self.navigator.register_callback(self.rx_finished, 'right_button_ok')
		self.navigator.register_callback(self.rx_cheat_code, 'head_button_ok')

		# Gripper
		self.gripper = intera_interface.get_current_gripper_interface()
		if isinstance(self.gripper, intera_interface.SimpleClickSmartGripper):
			if self.gripper.needs_init():
				self.gripper.initialize()
		else:
			if not (self.gripper.is_calibrated() or self.gripper.calibrate() == True):
				raise
		self.open_gripper()

		# Cuff
		self.cuff = intera_interface.Cuff()
		self.cuff_callback_ids = self.CUFF_BUTTON.copy()
		for key in self.cuff_callback_ids:
			self.cuff_callback_ids[key] = -1

		# Initialization complete. Spin.
		rospy.loginfo('Ready.')
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			suppress_cuff_interaction.publish()
			r.sleep()

	## HELPER FUNCTIONS ##
	# Returns true if two positions equal each other or are at least within a given tolerance
	def endpoints_equal(self, pose1, pose2, tol=0):
		equality = True
		if 'position' in pose1 and 'position' in pose2:
			equality &= abs(pose1['position'].x-pose2['position'].x)<=tol \
			   and abs(pose1['position'].y-pose2['position'].y)<=tol \
			   and abs(pose1['position'].z-pose2['position'].z)<=tol
		if 'orientation' in pose1 and 'orientation' in pose2:
			equality &= abs(pose1['orientation'].x-pose2['orientation'].x)<=tol \
			   and abs(pose1['orientation'].y-pose2['orientation'].y)<=tol \
			   and abs(pose1['orientation'].z-pose2['orientation'].z)<=tol \
			   and abs(pose1['orientation'].w-pose2['orientation'].w)<=tol
		return equality

	# Allows user to move arm in zero G mode
	def user_move(self):
		startPose = self.limb.endpoint_pose()
		self.limb.interaction_control(orientation_x=False, orientation_y=False, orientation_z=False, position_x=True, position_y=True, position_z=True)
		while(self.endpoints_equal(startPose,self.limb.endpoint_pose(),tol=0.01)):	# Wait for user to begin moving arm
			pass
		rospy.sleep(0.5)
		while(not self.endpoints_equal(startPose,self.limb.endpoint_pose())):		# Wait for user to stop moving arm
			startPose = self.limb.endpoint_pose()
			rospy.sleep(1)
		self.limb.position_mode()

	# Individual joint constraint mode
	def joint_move(self,joints,terminatingCondition,resetPos,pCMD=lambda self: None, rateNom=10, tics=15, min_thresh=0, bias=0):
		self.limb.set_command_timeout(2)																# Set command timeout to be much greater than the command period
		rate = rospy.Rate(rateNom)																		# Define rate to send commands
		self.finished = False																			# Initialized finished variable for terminating condition

		# Initialize joint dicts of all zeros
		zeroVec = self.limb.joint_velocities()
		effortVec = {}
		min_thresh_vec = {}
		bias_vec = {}
		for joint in zeroVec.keys():
			zeroVec[joint] = 0
			effortVec[joint] = numpy.zeros(tics)
			if isinstance(min_thresh, dict):
				if joint in min_thresh.keys():
					min_thresh_vec[joint] = min_thresh[joint]
				else:
					min_thresh_vec[joint] = 0
			else:
				min_thresh_vec[joint] = min_thresh
			if isinstance(bias, dict):
				if joint in bias.keys():
					bias_vec[joint] = bias[joint]
				else:
					bias_vec[joint] = 0
			else:
				bias_vec[joint] = bias
		lastEffort = zeroVec.copy()																		# Initialize two effort dicts
		thisEffort = zeroVec.copy()
		
		# Main loop
		i = 0
		while not terminatingCondition(self):
			self.joint_safety_check(lambda self : self.limb.go_to_joint_angles(resetPos), lambda self : None)
			pCMD(self)																					# Publish whatever the user wants

			rospy.Publisher('robot/joint_state_publish_rate',UInt16,queue_size=10).publish(rateNom)		# Set publish rate

			# Measure effort
			while thisEffort==lastEffort:
				thisEffort = self.limb.joint_efforts()
			lastEffort = thisEffort
			for joint in effortVec.keys():
				effortVec[joint][i] = thisEffort[joint]
			i = i+1 if i+1<tics else 0

			# Filter effort and convert into velocity
			velocities = zeroVec.copy()
			for joint in velocities.keys():
				if joint in joints:
					filteredForce = numpy.mean(effortVec[joint])+bias_vec[joint]
					#rospy.loginfo(joint + ': ' + str(filteredForce))
					if abs(filteredForce) < min_thresh_vec[joint]:
						velocities[joint] = 0
					else:
						velocities[joint] = -self.FORCE2VELOCITY[joint]*filteredForce
			self.limb.set_joint_velocities(velocities)

			rate.sleep()
		rospy.loginfo('Joint move completed')
		self.command_complete_topic.publish()
		self.limb.exit_control_mode()

	def joint_impedance_move(self,b,k,terminatingCondition,pCMD=lambda self: None, rateNom=50, tics=1):
		self.limb.set_command_timeout(2)																# Set command timeout to be much greater than the command period
		rate = rospy.Rate(rateNom)																		# Define rate to send commands
		self.finished = False																			# Initialized finished variable for terminating condition
		effortVec = {}
		for joint in self.limb.joint_angles().keys():
			effortVec[joint] = numpy.zeros(tics)
		startPos = self.limb.joint_angles()
		i=0
		while not terminatingCondition(self):
			pCMD(self)																					# Publish whatever the user wants

			rospy.Publisher('robot/joint_state_publish_rate',UInt16,queue_size=10).publish(rateNom)		# Set publish rate

			# Calculate effort
			thisPos = self.limb.joint_angles();
			thisVel = self.limb.joint_velocities();
			for joint in startPos.keys():
				effortVec[joint][i] = 0
				if b.has_key(joint):
					effortVec[joint][i] += -b[joint]*thisVel[joint]
				if k.has_key(joint):
					effortVec[joint][i] += -k[joint]*(thisPos[joint]-startPos[joint])
			i = i+1 if i+1<tics else 0

			# Filter and set
			efforts = self.limb.joint_efforts();
			for joint in efforts:
				efforts[joint] = numpy.mean(effortVec[joint])
			self.limb.set_joint_torques(efforts)

			rate.sleep()
		self.limb.exit_control_mode()

	def set_joint_imp(self, springs, damping, ref_pos, ref_vel):
		cmd = dict()
		cur_pos = self.joint_angles()
		cur_vel = self.joint_velocities()
		for j,joint in enumerate(cur_pos.keys()):
			cmd[joint] = springs[j]*(ref_pos[joint]-cur_pos[joint])
			cmd[joint] += damping[j]*(ref_vel[joint]-cur_vel[joint])
		self.set_joint_torques(cmd)

	# Returns true iff the current joint position will not collide with the table
	def joint_safety_check(self, resetFn, returnFn):
		#rospy.loginfo('in safety zone')
		pos = self.limb.endpoint_pose()['position']
		if pos.z<0 or pos.y<-0.2:
			rospy.loginfo('oops outside safety zone')
			self.limb.position_mode()
			rospy.loginfo('position_mode entered')
			mixer.init()
			mixer.music.load('safety1.mp3')
			mixer.music.play()
			#playsound('safety1.mp3')
			rospy.loginfo('Audio file played')
			rospy.sleep(6.5)
			resetFn(self)
			rospy.loginfo('reset position done')
			mixer.init()
			mixer.music.load('safety2.mp3')
			mixer.music.play()
			#playsound('safety2.mp3')
			rospy.loginfo('Audio file 2 played')
			returnFn(self)
			rospy.loginfo('Ready to be moved again')

	# Open gripper
	def open_gripper(self):
		if self.VERBOSE: rospy.loginfo('Gripper open triggered')
		if isinstance(self.gripper, intera_interface.SimpleClickSmartGripper):
			self.gripper.set_ee_signal_value('grip', False)
		else:
			self.gripper.open()

	# Close gripper
	def close_gripper(self):
		if self.VERBOSE: rospy.loginfo('Gripper close triggered')
		if isinstance(self.gripper, intera_interface.SimpleClickSmartGripper):
			self.gripper.set_ee_signal_value('grip', True)
		else:
			self.gripper.close()

	# Lower the gripper, close gripper, raise gripper, check effort, return whether or not an object is being supported 
	def cb_PickUpBox(self, goal, lift_position=Z_TABLE+BOX_HEIGHT+0.1, effort_tol=5):
		success = True
		result_PickUpBox = PickUpBoxResult()

		# Leave the box and lift upward to measure no-load effort
		self.limb.adjustPoseTo('position','z',lift_position)
		rospy.sleep(1)
		effort_nobox = sum(abs(effort) for effort in self.limb.joint_efforts().values())

		# Grab and lift box to measure effort under load
		self.limb.adjustPoseTo('position','z',self.Z_TABLE+self.BOX_HEIGHT/2)
		self.close_gripper()
		rospy.sleep(2)
		self.limb.adjustPoseTo('position','z',lift_position)
		rospy.sleep(1)
		effort_box = sum(abs(effort) for effort in self.limb.joint_efforts().values())
		print('no box: ' + str(effort_nobox))
		print('w/ box: ' + str(effort_box))

		hasBox = effort_box-effort_nobox>effort_tol
		if not hasBox:
			self.open_gripper()
			if self.VERBOSE: rospy.loginfo('Failed to pick up box.')
		elif self.VERBOSE:
			rospy.loginfo('Successfully picked up box.')
			#self.limb.adjustPoseTo('position','z',self.Z_TABLE+self.BOX_HEIGHT/2)
			# self.open_gripper()
			# self.limb.adjustPoseTo('position','z',self.Z_TABLE+self.BOX_HEIGHT+0.1)

		if success:
			result_PickUpBox.is_picked = hasBox
			self.PickUpBoxAct.set_succeeded(result_PickUpBox)

	# Lower the gripper, close gripper, raise gripper, check effort, return whether or not an object is being supported 

	# Comments inside check_pickup 
		'''
		del self.effort[:]

		self.startPose = self.limb.endpoint_pose()
		self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)

		self.startPose_container.position_z = lift_position + .1
		self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)

		self.newCartPose = self.pose_to_cartclass(pose = self.startPose, cartclass = self.newCartPose)
		self.newCartPose.position_z = lift_position
		self.newCartPose_arg = self.cartclass_to_cartarg(cartclass = self.newCartPose, cartarg = self.newCartPose_arg)
		
		self.limb.go_to_cartesian_pose(joint_angles = self.startPose_arg)
		self.effort.append(self.limb.joint_efforts())


		self.limb.go_to_cartesian_pose(joint_angles = self.newCartPose_arg)
		self.close_gripper()
		rospy.sleep(2)
		self.limb.go_to_cartesian_pose(joint_angles = self.startPose_arg)
		self.effort.append(self.limb.joint_efforts())
		if(not self.compare_efforts(effort1 = self.effort[0], effort2 = self.effort[1])):
			self.limb.go_to_cartesian_pose(joint_angles = self.newCartPose_arg)
		'''

	def display_camera_callback(self, img_data):
		bridge = CvBridge()
		try:
			cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')
		except CvBridgeError, err:
			rospy.logerr(err)

		img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		detector = apriltag.Detector()
		detections = detector.detect(img)
		# rospy.loginfo(detections[0].corners)

		cv2.line(cv_image, (int(detections[0].corners[0][0]),int(detections[0].corners[0][1])), (int(detections[0].corners[1][0]),int(detections[0].corners[1][1])), (255,255,51), 3)
		cv2.line(cv_image, (int(detections[0].corners[1][0]),int(detections[0].corners[1][1])), (int(detections[0].corners[2][0]),int(detections[0].corners[2][1])), (255,255,51), 3)
		cv2.line(cv_image, (int(detections[0].corners[3][0]),int(detections[0].corners[3][1])), (int(detections[0].corners[2][0]),int(detections[0].corners[2][1])), (255,255,51), 3)
		cv2.line(cv_image, (int(detections[0].corners[3][0]),int(detections[0].corners[3][1])), (int(detections[0].corners[0][0]),int(detections[0].corners[0][1])), (255,255,51), 3)

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5,5), 0)
		ret, thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY_INV)

		im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
		cnt = contours[0]
		max_area = cv2.contourArea(cnt)

		for contour in contours:
			if cv2.contourArea(contour) > max_area:
				cnt = contour
				max_area = cv2.contourArea(contour)

		rect = cv2.minAreaRect(cnt)
		box = cv2.boxPoints(rect)
		box = numpy.int0(box)

		cv2.drawContours(cv_image, [box], -1, (0,255,0), 3)

		# a = sum(map(lambda x: x[0], box))/4
		# b = sum(map(lambda x: x[1], box))/4

		#img = cv_image
		# rospy.loginfo('Creating image')
		cv2.imwrite('/home/albertgo/TeachBot/browser/public/images/cv_image.png', cv_image)
		self.command_complete_topic.publish()
	


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
		msg = Int32()
		msg.data = num
		self.wheel_delta_topic.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(num))
	def pub_arr(self, arr):
		msg = Float64MultiArray()
		msg.data = arr
		self.array_topic.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(arr))

	## ROS SUBSCRIBERS ##
	def rx_audio_duration(self,data):
		self.audio_duration = data.audio_duration
		return True

	def forwardJointState(self, data):
		position = JointInfo()
		velocity = JointInfo()
		effort = JointInfo()
		for j in range(Module.JOINTS):
			setattr(position, 'j'+str(j), data.position[j+1])
			setattr(velocity, 'j'+str(j), data.velocity[j+1])
			setattr(effort, 'j'+str(j), data.effort[j+1])
		self.position_topic.publish(position)
		self.velocity_topic.publish(velocity)
		self.effort_topic.publish(effort)

	def forwardEndpointState(self, data):
		endpoint_msg = EndpointInfo()
		endpoint_msg.position.x = data.pose.position.x
		endpoint_msg.position.y = data.pose.position.y
		endpoint_msg.position.z = data.pose.position.z
		endpoint_msg.orientation.x = data.pose.orientation.x
		endpoint_msg.orientation.y = data.pose.orientation.y
		endpoint_msg.orientation.z = data.pose.orientation.z
		endpoint_msg.orientation.w = data.pose.orientation.w
		self.endpoint_topic.publish(endpoint_msg)

	def rx_finished(self,data):
		if self.navigator.button_string_lookup(data) == 'OFF':
			self.finished = True
			self.scroll_wheel_button_topic.publish()
			self.clicked.publish(True)
		if self.VERBOSE: rospy.loginfo('Rx from arm scroll wheel button: ' + self.navigator.button_string_lookup(data) + '. finished = ' + str(self.finished))

	def rx_cheat_code(self,data):
		if self.navigator.button_string_lookup(data) == 'OFF':
			if self.devMode:
				self.devMode = False
				self.DevModeSrv(0)
				self.lights.set_light_state('head_blue_light',False)
				if self.VERBOSE: rospy.loginfo('Exiting dev mode')
			else:
				self.devMode = True
				self.DevModeSrv(1)
				self.lights.set_light_state('head_blue_light',True)
				if self.VERBOSE: rospy.loginfo('Entering dev mode')

	def subscribe_to_wheel_move(self):
		def rx_wheel_move(data):
			if data-self.wheel_state==0:
				delta=0
			elif data-self.wheel_state>255/2 or (data-self.wheel_state<0 and data-self.wheel_state>-255/2):
				delta=1
			else:
				delta=-1
			self.wheel_state = data
			req = Int32()
			req.data = delta
			self.wheel_delta_topic.publish(req)
			if self.VERBOSE: rospy.loginfo('Wheel value: {0}. Wheel delta: {1}.'.format(data, delta))

		self.wheel_callback_id = self.navigator.register_callback(rx_wheel_move, 'right_wheel')

	def unsubscribe_from_wheel_move(self):
		if self.wheel_callback_id==-1:
			rospy.loginfo('Already unsubscribed from wheel movement.')
		self.navigator.deregister_callback(self.wheel_callback_id)
		self.wheel_callback_id = -1

	def subscribe_to_multi_choice(self):
		def button_pressed(name, data):
			if self.navigator.button_string_lookup(data) == 'OFF':
				self.pub_num(self.BUTTON[name])
			if self.VERBOSE: rospy.loginfo(name + ' button pressed.')

		for key in self.BUTTON:
			self.multi_choice_callback_ids[key] = self.navigator.register_callback(lambda data, key=key : button_pressed(key, data), 'right_button_' + key)

	def unsubscribe_from_multi_choice(self):
		if self.multi_choice_callback_ids[self.BUTTON.keys()[0]]==-1:
			rospy.loginfo('Already unsubscribed from multiple choice callbacks.')
		self._unsubscribe_from(self.navigator, self.multi_choice_callback_ids)

	def _unsubscribe_from(self, obj, callback_ids):
		notified = False
		for key in callback_ids:
			if self.VERBOSE and not notified:
				rospy.loginfo('Already unsubscribed from ' + str(obj))
				notified = True
			obj.deregister_callback(callback_ids[key])
			callback_ids[key] = -1

	def rx_command(self, data):
		pass

	def cb_AdjustPoseBy(self, goal):
		if self.VERBOSE: rospy.loginfo('Adjusting pose by')

		success = True
		result_AdjustPoseBy = AdjustPoseByResult()
		result_AdjustPoseBy.is_done = False

		self.limb.adjustPoseBy(goal.geometry, goal.axis, eval(goal.amount))

		if success:
			result_AdjustPoseBy.is_done = True
			self.AdjustPoseByAct.set_succeeded(result_AdjustPoseBy)

	def cb_AdjustPoseTo(self, goal):
		if self.VERBOSE: rospy.loginfo('Adjusting pose to')

		success = True
		result_AdjustPoseTo = AdjustPoseToResult()
		result_AdjustPoseTo.is_done = False

		if self.AdjustPoseToAct.is_preempt_requested():
			rospy.loginfo("%s: Preempted", 'n/a')
			self.AdjustPoseToAct.set_preempted()
			success = False

		self.limb.adjustPoseTo(goal.geometry, goal.axis, eval(goal.amount))

		if success:
			result_AdjustPoseTo.is_done = True
			self.AdjustPoseToAct.set_succeeded(result_AdjustPoseTo)

	def cb_Gripper(self,goal):

		success = True
		result_Gripper = GripperResult()
		result_Gripper.is_done = False

		if self.GripperAct.is_preempt_requested():
			rospy.loginfo("%s: Preempted", 'n/a')
			self.GripperAct.set_preempted()
			success = False

		if (goal.todo=='open'):
			if self.VERBOSE: rospy.loginfo('Opening gripper')
			self.open_gripper()
		elif (goal.todo=='close'):
			if self.VERBOSE: rospy.loginfo('Closing gripper')
			self.close_gripper()
		else:
			if self.VERBOSE: rospy.loginfo('Gripper doing nothing')
			pass

		if success:
			result_Gripper.is_done = True
			self.GripperAct.set_succeeded(result_Gripper)

	def cb_camera(self, data):
		if data.data == True:
			if self.VERBOSE: rospy.loginfo('Accessing camera')

			rp = intera_interface.RobotParams()
			valid_cameras = rp.get_camera_names()
			camera = intera_interface.Cameras()
			camera.start_streaming('right_hand_camera')
			rectify_image = False
			camera.set_callback('right_hand_camera', self.display_camera_callback)
		else:
			if self.VERBOSE: rospy.loginfo('Shutting down camera')
			rp = intera_interface.RobotParams()
			valid_cameras = rp.get_camera_names()
			camera = intera_interface.Cameras()
			camera.stop_streaming('right_hand_camera')

	# When the lower cuff button is pressed, allow 2D position movement.
	def cb_cuff_lower(self, data):
		rospy.loginfo('Cuff lower button pressed.')
		self.limb.interaction_control(position_x=True, position_y=True)
		feedback = sawyer.msg.CuffInteractionFeedback()
		feedback.mode = True
		self.CuffInteractionAct.publish_feedback(feedback)

	# When the upper cuff button is pressed, allow 1D orientation movement.
	def cb_cuff_upper(self, data):
		rospy.loginfo('Cuff upper button pressed.')
		self.limb.interaction_control(orientation_z=True, in_endpoint_frame=True)
		feedback = sawyer.msg.CuffInteractionFeedback()
		feedback.mode = False
		self.CuffInteractionAct.publish_feedback(feedback)

	def cb_CuffInteraction(self, goal):
		if self.VERBOSE: rospy.loginfo('Cuff interaction engaged')
		terminatingCondition = eval(goal.terminatingCondition)
		ways = goal.ways

		# Initialize action objects
		result = sawyer.msg.CuffInteractionResult()

		
		self.cuff_callback_ids['lower'] = self.cuff.register_callback(self.cb_cuff_lower, 'right_button_lower')
		self.cuff_callback_ids['upper'] = self.cuff.register_callback(self.cb_cuff_upper, 'right_button_upper')
		
		# Continue until terminating condition is met.
		# If in waypoint mode, store each waypoint.
		self.finished = False
		if ways:
			if terminatingCondition is not None:
				while not terminatingCondition(self):
					pass
				waypoints.append(self.limb.joint_angles())
		else:
			if terminatingCondition is not None:
				while not terminatingCondition(self):
					pass
		
		# After the terminating condition is met, reset to position mode and complete action.
		rospy.loginfo('Done with cuff interaction')
		self.unsubscribe_from_cuff_interaction()
		result.done = True
		self.CuffInteractionAct.set_succeeded(result)

	def cb_GoToCartesianPose(self, goal):
		if self.VERBOSE: rospy.loginfo('Going to cartesian pose')

		success = True

		result_GoToCartesianPose = GoToCartesianPoseResult()
		result_GoToCartesianPose.is_done = False

		if self.GoToCartesianPoseAct.is_preempt_requested():
			rospy.loginfo("%s: Preempted", 'n/a')
			self.GoToCartesianPoseAct.set_preempted()
			success = False

		self.limb.go_to_cartesian_pose(position=eval(goal.position), orientation=eval(goal.orientation), relative_pose=eval(goal.relative_pose), joint_angles=eval(goal.joint_angles), endpoint_pose=eval(goal.endpoint_pose))

		if success:
			result_GoToCartesianPose.is_done = True
			self.GoToCartesianPoseAct.set_succeeded(result_GoToCartesianPose)

	def cb_GoToJointAngles(self, goal):
		if self.VERBOSE: rospy.loginfo('Going to joint angles')

		speed_ratio = 0.5 if goal.speed_ratio == 0 else goal.speed_ratio

		ways = False
		if goal.name == 'waypoints.pop(0)':
			ways = True

		result = sawyer.msg.GoToJointAnglesResult()

		if goal.name is '':
			self.limb.go_to_joint_angles([goal.j0pos, goal.j1pos, goal.j2pos, goal.j3pos, goal.j4pos, goal.j5pos, goal.j6pos], speed_ratio=speed_ratio)
			result.success = True
			self.GoToJointAnglesAct.set_succeeded(result)
		else:
			if goal.wait == True:
				startTime = rospy.get_time()
				goto = self.limb.go_to_joint_angles(eval(goal.name), speed_ratio=speed_ratio, ways = ways)

				if goto == False:
					rospy.loginfo('correct me please')

					mixer.init()
					mixer.music.load('safety3.mp3')
					mixer.music.play()
					rospy.loginfo('Audio file played')
					rospy.sleep(10.5)

					msg = InteractionControl()
					msg.position_only = False
					msg.orientation_x = True
					msg.orientation_y = True
					msg.orientation_z = True
					msg.position_x = True
					msg.position_y = True
					msg.position_z = True
					msg.PASS = False
					msg.ways = False
					self.interaction_control_topic.publish(msg)

					self.finished = False
					while (not self.finished):
						pass

					mixer.init()
					mixer.music.load('safety4.mp3')
					mixer.music.play()
					rospy.loginfo('Audio file played')
					rospy.sleep(4.5)
					self.limb.go_to_joint_angles(default)

				while rospy.get_time()-startTime<self.audio_duration:
					pass
			else:
				goto = self.limb.go_to_joint_angles(eval(goal.name), speed_ratio=speed_ratio, ways = ways)
				rospy.loginfo(goto)
				if goto == False:
					rospy.loginfo('correct me please')

					mixer.init()
					mixer.music.load('safety3.mp3')
					mixer.music.play()
					rospy.loginfo('Audio file played')
					rospy.sleep(10.5)

					msg = InteractionControl()
					msg.position_only = False
					msg.orientation_x = True
					msg.orientation_y = True
					msg.orientation_z = True
					msg.position_x = True
					msg.position_y = True
					msg.position_z = True
					msg.PASS = False
					msg.ways = False
					self.interaction_control_topic.publish(msg)

					self.finished = False
					while (not self.finished):
						pass

					mixer.init()
					mixer.music.load('safety4.mp3')
					mixer.music.play()
					rospy.loginfo('Audio file played')
					rospy.sleep(4.5)
					self.limb.go_to_joint_angles(default)
			result.success = True
			self.GoToJointAnglesAct.set_succeeded(result)

	def cb_HighTwo(self, goal):
		if self.VERBOSE: rospy.loginfo('High two attempted')

		success = True
		result_HighTwo = HighTwoResult()
		result_HighTwo.is_success = False

		if goal.high_two == True:
			startEffort = sum(abs(effort) for effort in self.limb.joint_efforts().values())
			startTime = rospy.get_time()
			while abs(startEffort-sum(abs(effort) for effort in self.limb.joint_efforts().values()))<0.1*startEffort and rospy.get_time()-startTime<8:
				rospy.sleep(0.1)
			if rospy.get_time()-startTime<8:
				result_HighTwo.is_success = True
			else:
				result_HighTwo.is_success = False

		if success:
			this.HighTwoAct.set_succeeded(result_HighTwo)

	def cb_impedance(self, req):
		if self.VERBOSE: rospy.loginfo('Impedance activated')
		k = self.limb.joint_angles()
		b = k.copy()
		for joint in k.keys():
			k[joint] = 160 if joint=='right_j1' else 10
			b[joint] = 5 if joint=='right_j1' else 10
		self.finished = False
		self.joint_impedance_move(b,k,eval(req.terminatingCondition), tics=req.tics)

	def cb_interaction(self, goal):
		if self.VERBOSE: rospy.loginfo('Free motion is on')

		result = sawyer.msg.InteractionControlResult()
		
		self.limb.interaction_control(position_only=goal.position_only, orientation_x=goal.orientation_x, orientation_y=goal.orientation_y, orientation_z=goal.orientation_z, position_x=goal.position_x, position_y=goal.position_y, position_z=goal.position_z, in_endpoint_frame=goal.in_end_point_frame)

		if goal.PASS == False:
			ang_arr = [0]*self.JOINTS
			self.finished = False
			while(not self.finished):													# Wait for user to stop moving arm
				startPose = self.limb.endpoint_pose()
				for m in range(0,10):
					for j in range(0,self.JOINTS):
						ang_arr[j] = self.limb.joint_angles()['right_j' + str(j)]
					rospy.sleep(0.1)
			self.limb.position_mode()
			result.done = True
			self.InteractionControlAct.set_succeeded(result)
		elif goal.ways == False:
			self.finished = False
			while(not self.finished):
				pass
			self.limb.position_mode()
			result.done = True
			self.InteractionControlAct.set_succeeded(result)
		else:
			self.limb.interaction_control(position_only=goal.position_only, orientation_x=goal.orientation_x, orientation_y=goal.orientation_y, orientation_z=goal.orientation_z, position_x=goal.position_x, position_y=goal.position_y, position_z=goal.position_z, in_endpoint_frame=goal.in_end_point_frame)
			self.finished = False
			while not self.finished:
				pass
			waypoints.append(self.limb.joint_angles())
			self.limb.position_mode()
			result.done = True
			self.InteractionControlAct.set_succeeded(result)

	def cb_joint_angle(self, req):
		if self.VERBOSE: rospy.loginfo('First example of impedance')
		self.startPos = self.limb.joint_angle(req.data)					# Record starting position
		while(abs(self.startPos-self.limb.joint_angle('right_j1'))<0.01):	# Wait for user to begin moving arm
			pass
		self.command_complete_topic.publish()

	def cb_joint_move(self, req):
		self.finished = False
		if self.VERBOSE: rospy.loginfo('joint able to be moved')
		
		result = sawyer.msg.JointMoveResult()

		self.joint_move(eval(req.joints), eval(req.terminatingCondition), eval(req.resetPOS), min_thresh=eval(req.min_thresh), bias=eval(req.bias))

		result.done = True
		self.JointMoveAct.set_succeeded(result);

	def cb_multiple_choice(self, req):
		if self.VERBOSE: rospy.loginfo('Multiple choice ready')
		if req.data:
			self.subscribe_to_multi_choice()
		else:
			self.unsubscribe_from_multi_choice()

	def cb_WheelSubscription(self, req):
		if req.subscribe:
			self.subscribe_to_wheel_move()
		else:
			self.unsubscribe_from_wheel_move()
		return True

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
	# Commonly used objects
	joint_buttons = [-0.0393427734375+math.pi/2,-0.71621875,0.022359375,1.811921875,-0.116017578125,2.77036035156,-4.48708789063]
	default = [math.pi/2, -0.9, 0.0, 1.8, 0.0, -0.9, 0.0]

	BIAS_SHOULDER = 0#-0.5
	BIAS_WRIST = -0.2
	#shoulder_wrist_bias = {shoulder: BIAS_SHOULDER, wrist: BIAS_WRIST}
	#shoulder_wrist_thresh = {shoulder: 1.0, wrist: 0.5}

	# Joint Names
	shoulder = 'right_j0'
	elbow = 'right_j3'
	wrist = 'right_j5'

	# Joint Positions
	no_hit = -0.04 # Maximum j1 to avoid scraping table in SCARA
	DSP = -0.10    # Default shoulder position in SCARA
	j4max = 2.99   # The maximum joint angle achievable by right_j4 and _j5
	j2scara = math.pi/2 - math.pi+j4max
	j6scara = -3*math.pi/2 + math.pi-j4max

	# 1
	joint_motor_animation_0 = [0]*Module.JOINTS
	joint_motor_animation_0[0] = math.pi/2
	joint_motor_animation_1 = joint_motor_animation_0[:]
	joint_motor_animation_1[4] = j4max

	# 4
	joint_test = [0]*Module.JOINTS
	for j in range(0,Module.JOINTS):
		joint_angle_arr = joint_motor_animation_0[:]
		if (j==0):
			joint_angle_arr[j] = 0.5+math.pi/2
		elif (j==1 or j==3):
			joint_angle_arr[j] = -1.0
		else:
			joint_angle_arr[j] = 1.0
		joint_test[j] = joint_angle_arr
	
	# 6-15
	joint_dof_start = [DSP,no_hit,j2scara,0,-j4max,0,j6scara]
	joint_dof_shoulder = [-0.3,no_hit,j2scara,0,-j4max,0,j6scara]
	joint_dof_elbow = [DSP,no_hit,j2scara,-0.46,-j4max,0,j6scara]
	joint_dof_wrist = [DSP,no_hit,j2scara,0,-j4max,1.45,j6scara]
	joint_dof_up = [DSP,-0.3,j2scara,0,-j4max,0,j6scara]

	# 18-24
	point_a = joint_dof_start[:]
	point_b = joint_dof_start[:]
	point_c = joint_dof_start[:]
	point_a[0] = 0.12
	point_b[0] = -0.06
	point_c[0] = -0.21

	# 25
	joint_push_down = [math.pi/2,0,0,math.pi/2,0,0,0]

	# 32-37
	joint_dot_1 = [DSP,no_hit,j2scara,0,-j4max,-1.83,j6scara]
	joint_dot_2 = [-0.21,no_hit,j2scara,0,-j4max,0,j6scara]
	joint_arc_wrist = [joint_dot_2[0],no_hit,j2scara,0,-j4max,-j4max,j6scara]
	joint_ortho_kin = joint_arc_wrist[:]
	joint_ortho_kin[5] = -math.pi/2
	joint_arc_shoulder = joint_ortho_kin[:]
	joint_arc_shoulder[0] = -0.45

	# 42
	dot_3 = {'position': intera_interface.Limb.Point(x=1.00,y=0.31,z=0.06)}
	joint_dot_3 = [0.13,no_hit,j2scara,0,-j4max,-0.07,j6scara]
	joint_dot_3_2_1 = joint_ortho_kin[:]
	joint_dot_3_2_1[0] = joint_dot_3[0]

	# 46
	joint_dot_3_4_2 = joint_ortho_kin[:]
	joint_dot_3_4_2[0] = (joint_ortho_kin[0]+joint_dot_3[0])/2
	joint_dot_3_4_2[5] = (joint_ortho_kin[5]+joint_dot_3[5])/2
	joint_dot_3_4_1 = joint_ortho_kin[:]
	joint_dot_3_4_1[0] = joint_dot_3_4_2[0]
	joint_dot_3_4_3 = joint_dot_3_4_2[:]
	joint_dot_3_4_3[0] = joint_dot_3[0]

	# 47
	joint_dot_3_8_4 = joint_dot_3_4_2[:]
	joint_dot_3_8_2 = joint_ortho_kin[:]
	joint_dot_3_8_2[0] = (joint_ortho_kin[0]+joint_dot_3_8_4[0])/2
	joint_dot_3_8_2[5] = (joint_ortho_kin[5]+joint_dot_3_8_4[5])/2
	joint_dot_3_8_6 = joint_dot_3_8_4[:]
	joint_dot_3_8_6[0] = (joint_dot_3_8_4[0]+joint_dot_3[0])/2
	joint_dot_3_8_6[5] = (joint_dot_3_8_4[5]+joint_dot_3[5])/2
	joint_dot_3_8_1 = joint_ortho_kin[:]
	joint_dot_3_8_1[0] = joint_dot_3_8_2[0]
	joint_dot_3_8_3 = joint_dot_3_8_2[:]
	joint_dot_3_8_3[0] = joint_dot_3_8_4[0]
	joint_dot_3_8_5 = joint_dot_3_8_4[:]
	joint_dot_3_8_5[0] = joint_dot_3_8_6[0]
	joint_dot_3_8_7 = joint_dot_3_8_6[:]
	joint_dot_3_8_7[0] = joint_dot_3[0]

	# 49-
	waypoints = []

	# 58
	joint_high_two = [1.39222949219,0.655348632812,-0.064970703125,-1.86494433594,0.156983398438,-0.364296875,3.24111523438]

	## MODULE 2 ##
	init_joint_arg                = [1.133,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625]
	fail_init_joint_arg           = [0.934929668903,0.21894530952,-1.86988866329,1.08379101753,-1.21311235428,-1.71679496765,1.97222]
	success_pickup_box1           = [0.934929668903,0.21894530952,-1.86988866329,1.08379101753,-1.21311235428,-1.71679496765,3.25700879097]

	above_first_bin_joint_arg     = [0.407034188509,0.148331061006,-1.81420600414,0.988864243031,-1.31577932835,-1.66601657867,2.78698539734]

	above_second_box_joint_arg    = [0.570015609264,0.121503904462,-1.91593551636,0.632845699787,-1.22943162918,-1.65171098709,3.31128621101]
	above_third_box_joint_arg     = [0.236366212368,0.0262167975307,-1.7436337471,-0.0858056619763,-1.42195904255,-1.49928414822,2.0856685]
	above_fourth_box_joint_arg    = [0.109769530594,0.200036138296,-1.93925189972,1.00660443306,-1.18624413013,-1.72986137867,2.499067143]

	camera_pos                    = [0.314609375, 0.19108984375, -1.9322587890625, 1.64379296875, 1.7396455078125, 0.37216796875, 0.183701171875]
	# fail_init_joint_arg           = [1.033235,-0.629208007812,-1.01547070313,1.05442871094,-2.38241699219,-1.48850390625,-1.18359277344]
	# fail_pickup_cart_arg          = [1.08375488281,0.175158203125,-1.53774609375,1.02942480469,-1.45563085938,-1.45510351563,1.86297558594]
	# fail_above_pickup_cart_arg    = [1.06155175781,-0.26884765625,-1.4501015625,0.838840820312,-1.88630175781,-1.62122851562,1.9701875]

	# success_init_joint_arg        = [1.05569238281,-0.176821289062,-1.62947851563,0.904107421875,-1.66179296875,-1.63658007812,0.266885742187]
	# success_pickup_joint_arg      = [1.06658789062,0.264276367188,-1.62918945312,1.05704492187,-1.31784570312,-1.50530078125,0.266266601562]
	# success_pickup_cart_arg       = [1.06658789062,0.264276367188,-1.62918945312,1.05704492187,-1.31784570312,-1.50530078125,0.266266601562]
	# success_above_pickup_cart_arg = [1.05569238281,-0.176821289062,-1.62947851563,0.904107421875,-1.66179296875,-1.63658007812,0.266885742187]

	# above_bin_cart_arg            = [0.935443359375,-0.182680664062,-1.34264160156,1.20748339844,-1.85780664063,-1.45737597656,3.1062734375]
	# rotated_above_bin_cart_arg    = [0.938936523438,-0.166234375,-1.35050976563,1.20908984375,-1.8427265625,-1.403859375,1.51214648438]
	# into_bin_cart_arg             = [0.943045898437,0.233333984375,-1.50463964844,1.30310449219,-1.3893203125,-1.43581445312,1.4121640625]
	# hit_bin_joint_arg             = [0.291505873203,0.107424803078,-1.5040615797,0.564673841,-1.64092874527,-1.56688666344,3.12259268761]
	
	# into_second_bin_cart_arg      = [0.561075195313,0.2966484375,-1.81974609375,0.759317382813,-1.23418261719,-1.49326855469,0.08444140625]
	# above_second_bin_cart_arg     = [0.5035625,0.0966318359375,-1.882125,0.744194335937,-1.22447363281,-1.62413378906,0.002185546875]

	# above_second_bin_joint_arg    = [0.832271456718,0.251014649868,-1.81858205795,1.50893843174,-1.30731058121,-1.79992866516,2.6581864357]

	# above_second_box_cart_arg     = [math.pi/2-0.729661132813,0.12510546875,-1.78256054688,1.094546875,-1.35895410156,-1.71990722656,4.66472753906]
	
	# above_third_box_cart_arg      = [0.317999023437,0.0488310546875,-1.646234375,-0.249951171875,-1.51264550781,-1.61024121094,-2.44349414062]
	
	# above_third_box_joint_res     = [0.3603066504, -0.0363896489143, -1.34424805641, -0.101894527674, -1.77538383007, -1.57726657391, 2.27713108]
	# above_fourth_box_cart_arg     = [math.pi/2-1.482765625,0.17649609375,-2.27079101562,0.363586914062,-0.855272460937,-1.6052578125,1.5651328125]
	
	# low_fourth_box_joint_arg      = [0.0681884735823,0.202602535486,-1.48116016388,0.113159179688,-1.62357616425,-1.34329497814,3.34635257721]

	# success_pickup_box1           = [1.033235,-0.629208007812,-1.01547070313,1.05442871094,-2.38241699219,-1.48850390625,0.38779632679]


	Module()