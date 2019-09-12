#! /usr/bin/env python

import math, rospy
from intera_interface import Limb
from module import (Module, Sequence)

BIAS_SHOULDER = 0#-0.5
BIAS_WRIST = 0#-1.0

class listener(Module):
	## LISTENER EVENTS ##
	# Runs whenever message is received on main ROSTopic from JavaScript
	def rx_command(self, data):
		seq = Sequence(data.data, rospy.get_time())
		if self.VERBOSE: rospy.loginfo("I heard: " + str(seq.idn))

		if seq.idn==0:
			self.limb.go_to_joint_angles(joint_motor_animation_0)

		elif seq.idn==1:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_angles=joint_motor_animation_1, speed_ratio=.225)
				self.limb.go_to_joint_angles(joint_angles=joint_motor_animation_0, speed_ratio=.225)
				self.limb.go_to_joint_angles(joint_angles=joint_motor_animation_1, speed_ratio=.225)
				self.limb.go_to_joint_angles(joint_angles=joint_motor_animation_0, speed_ratio=.225)

		elif seq.idn==2:
			self.limb.go_to_joint_angles(joint_buttons)
			self.subscribe_to_wheel_move()

		elif seq.idn==3:
			self.unsubscribe_from_wheel_move()
			self.limb.go_to_joint_angles(joint_motor_animation_0)

		elif seq.idn==4:
			if not self.devMode:
				for joint in joint_test:
					self.limb.go_to_joint_angles(joint)
					self.limb.go_to_joint_angles(joint_motor_animation_0)
			
		elif seq.idn==5:
			self.limb.go_to_joint_angles(joint_buttons)
			self.subscribe_to_wheel_move()

		elif seq.idn==6:
			self.unsubscribe_from_wheel_move()
			self.limb.go_to_joint_angles()
			self.limb.go_to_joint_angles(joint_dof_start)

		elif seq.idn==7:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_dof_shoulder)
				self.limb.go_to_joint_angles(joint_dof_start)

		elif seq.idn==8:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_dof_elbow)
				self.limb.go_to_joint_angles(joint_dof_start)

		elif seq.idn==9:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_dof_wrist)
				self.limb.go_to_joint_angles(joint_dof_start)

		elif seq.idn==10:
			rospy.sleep(0.5)
			if not self.devMode:
				self.finished = False
				self.joint_move(shoulder, lambda self: self.finished, joint_dof_start, lambda self: self.pub_num(180/math.pi*self.limb.joint_angle(shoulder)), min_thresh=1.0, bias=BIAS_SHOULDER)

		elif seq.idn==11:
			self.limb.go_to_joint_angles()

		elif seq.idn==12:
			self.limb.go_to_joint_angles(joint_dof_start)

		elif seq.idn==13:
			rospy.sleep(1.0)
			if not self.devMode:
				self.finished = False
				self.joint_move(wrist, lambda self: self.finished, joint_dof_start, lambda self: self.pub_num(180/math.pi*self.limb.joint_angle(wrist)), min_thresh=0.5, bias=BIAS_WRIST)

		elif seq.idn==14:
			rospy.sleep(0.5)
			if not self.devMode:
				self.finished = False
				self.joint_move([shoulder,wrist], lambda self: self.finished, joint_dof_start, lambda self: self.pub_arr([180/math.pi*self.limb.joint_angle(shoulder),180/math.pi*self.limb.joint_angle(wrist)]), min_thresh={shoulder: 1.0, wrist: 0.5}, bias={shoulder: BIAS_SHOULDER, wrist: BIAS_WRIST})

		elif seq.idn==15:
			self.limb.go_to_joint_angles(joint_dof_up)
			self.limb.go_to_joint_angles(joint_buttons)
			self.subscribe_to_wheel_move()

		elif seq.idn==16:
			self.unsubscribe_from_wheel_move()
			self.limb.go_to_joint_angles()

		elif seq.idn==17:
			if not self.devMode:
				self.limb.interaction_control(orientation_x=True, orientation_y=True, orientation_z=True, position_x=True, position_y=True, position_z=True)
				ang_arr = [0]*self.JOINTS
				self.finished = False
				while(not self.finished):													# Wait for user to stop moving arm
					startPose = self.limb.endpoint_pose()
					for m in range(0,10):
						for j in range(0,self.JOINTS):
							ang_arr[j] = self.limb.joint_angles()['right_j' + str(j)]
						self.pub_arr(ang_arr)
						rospy.sleep(0.1)
				self.limb.position_mode()

		elif seq.idn==18:
			if not self.devMode:
				self.limb.go_to_joint_angles()
			self.limb.go_to_joint_angles(point_a)

		elif seq.idn==19:
			if not self.devMode:
				self.joint_move(shoulder,lambda self : self.limb.joint_angle(shoulder)<point_b[0], point_a, min_thresh=1.0, bias=BIAS_SHOULDER)

		elif seq.idn==20:
			if not self.devMode:
				self.limb.go_to_joint_angles(point_a)
			self.limb.go_to_joint_angles(point_c)

		elif seq.idn==21:
			if not self.devMode:
				self.joint_move(shoulder,lambda self : self.limb.joint_angle(shoulder)>point_b[0], point_c, min_thresh=1.0, bias=BIAS_SHOULDER)

		elif seq.idn==22:
			self.limb.go_to_joint_angles(point_a)

		elif seq.idn==23:
			self.limb.go_to_joint_angles(point_b)

		elif seq.idn==24:
			self.limb.go_to_joint_angles(point_c)
		
		elif seq.idn==25:
			self.limb.go_to_joint_angles()
			self.limb.go_to_joint_angles(joint_push_down)

		elif seq.idn==26:
			if not self.devMode:
				self.startPos = self.limb.joint_angle('right_j1')					# Record starting position
				while(abs(self.startPos-self.limb.joint_angle('right_j1'))<0.01):	# Wait for user to begin moving arm
					pass

		elif seq.idn==27:
			if not self.devMode:
				k = self.limb.joint_angles()
				b = k.copy()
				for joint in k.keys():
					k[joint] = 160 if joint=='right_j1' else 10
					b[joint] = 5 if joint=='right_j1' else 10
				self.finished = False
				self.joint_impedance_move(b,k,lambda self: self.finished, lambda self: self.pub_arr([self.startPos-self.limb.joint_angle('right_j1'), abs(self.limb.joint_effort('right_j1'))]),tics=2)
			
		elif seq.idn==28:
			self.limb.go_to_joint_angles(joint_buttons)
			self.subscribe_to_multi_choice()

		elif seq.idn==29:
			self.unsubscribe_from_multi_choice()
			self.limb.go_to_joint_angles()

		elif seq.idn==30:
			startTime = rospy.get_time()
			self.limb.go_to_joint_angles(joint_dof_start)
			while rospy.get_time()-startTime<self.audio_duration:
				pass
		
		elif seq.idn==31:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_dof_shoulder)
				self.limb.go_to_joint_angles(joint_dof_start)
			
		elif seq.idn==32:
			self.limb.go_to_joint_angles(joint_dot_1)

		elif seq.idn==33:
			self.limb.go_to_joint_angles(joint_dof_start)

		elif seq.idn==34:
			if not self.devMode:
				self.joint_move(shoulder, lambda self : self.limb.joint_angle(shoulder)<joint_dot_2[0], joint_dof_start, min_thresh=1.0, bias=BIAS_SHOULDER)

		elif seq.idn==35:
			self.limb.go_to_joint_angles(joint_dot_2)

		elif seq.idn==36:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_arc_wrist)
				self.limb.go_to_joint_angles(joint_ortho_kin)

		elif seq.idn==37:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_arc_shoulder)
			self.limb.go_to_joint_angles(joint_ortho_kin)

		elif seq.idn==38:
			if not self.devMode:
				self.finished = False
				self.joint_move(shoulder, lambda self : self.finished, joint_dot_2, min_thresh=1.0, bias=BIAS_SHOULDER)

		elif seq.idn==39:
			if not self.devMode:
				tol = 0.05
				while not self.endpoints_equal(self.limb.endpoint_pose(),dot_3,tol):
					self.finished = False
					self.joint_move(wrist, lambda self : self.endpoints_equal(self.limb.endpoint_pose(),dot_3,tol) or self.finished, joint_dot_2, min_thresh=0.5, bias=BIAS_WRIST)
					if not self.endpoints_equal(self.limb.endpoint_pose(),dot_3):
						self.finished = False
						self.joint_move(shoulder, lambda self : self.endpoints_equal(self.limb.endpoint_pose(),dot_3,tol) or self.finished, joint_dot_2, min_thresh=1.0, bias=BIAS_WRIST)

		elif seq.idn==40:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_ortho_kin)
				self.limb.go_to_joint_angles(joint_dot_3_2_1)
				self.limb.go_to_joint_angles(joint_dot_3)

		elif seq.idn==41:
			if not self.devMode:
				startTime = rospy.get_time()
				self.limb.go_to_joint_angles(joint_ortho_kin)
				while rospy.get_time()-startTime<self.audio_duration:
					pass

		elif seq.idn==42:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_dot_3_2_1)

		elif seq.idn==43:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_dot_3)

		elif seq.idn==44:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_ortho_kin)
				self.limb.go_to_joint_angles(joint_dot_3_4_1)
				self.limb.go_to_joint_angles(joint_dot_3_4_2)
				self.limb.go_to_joint_angles(joint_dot_3_4_3)
				self.limb.go_to_joint_angles(joint_dot_3)

		elif seq.idn==45:
			if not self.devMode:
				self.limb.go_to_joint_angles(joint_ortho_kin)
				self.limb.go_to_joint_angles(joint_dot_3_8_1)
				self.limb.go_to_joint_angles(joint_dot_3_8_2)
				self.limb.go_to_joint_angles(joint_dot_3_8_3)
				self.limb.go_to_joint_angles(joint_dot_3_8_4)
				self.limb.go_to_joint_angles(joint_dot_3_8_5)
				self.limb.go_to_joint_angles(joint_dot_3_8_6)
				self.limb.go_to_joint_angles(joint_dot_3_8_7)
				self.limb.go_to_joint_angles(joint_dot_3)

		elif seq.idn==46:
			if not self.devMode:
				startTime = rospy.get_time()
				self.limb.go_to_joint_angles(joint_ortho_kin)
				while rospy.get_time()-startTime<self.audio_duration:
					pass
				self.limb.go_to_joint_angles(joint_dot_3)

		elif seq.idn==47:
			if not self.devMode:
				self.limb.interaction_control(orientation_x=True, orientation_y=True, orientation_z=True, position_x=True, position_y=True, position_z=True)
				self.finished = False
				while(not self.finished):
					pass
				self.limb.position_mode()

		elif seq.idn==48:
			self.limb.go_to_joint_angles()

		elif seq.idn>=49 and seq.idn<=53:
			self.limb.interaction_control(position_only=True)
			self.finished = False
			while not self.finished:
				pass
			waypoints.append(self.limb.joint_angles())
			self.limb.position_mode()

		elif seq.idn>=54 and seq.idn<=56:
			startTime = rospy.get_time()
			self.limb.go_to_joint_angles(waypoints.pop(0))
			while rospy.get_time()-startTime<self.audio_duration:
				pass

		elif seq.idn==57:
			startTime = rospy.get_time()
			self.limb.go_to_joint_angles(waypoints.pop(0))
			self.limb.go_to_joint_angles(waypoints.pop(0))
			while rospy.get_time()-startTime<self.audio_duration:
				pass

		elif seq.idn==58:
			self.limb.go_to_joint_angles(joint_high_two)
			rospy.sleep(0.5)
			startEffort = sum(abs(effort) for effort in self.limb.joint_efforts().values())
			startTime = rospy.get_time()
			while abs(startEffort-sum(abs(effort) for effort in self.limb.joint_efforts().values()))<0.1*startEffort and rospy.get_time()-startTime<8:
				rospy.sleep(0.1)
			if rospy.get_time()-startTime<8:
				seq.idn+=2

		elif seq.idn==59:
			startEffort = sum(abs(effort) for effort in self.limb.joint_efforts().values())
			startTime = rospy.get_time()
			while abs(startEffort-sum(abs(effort) for effort in self.limb.joint_efforts().values()))<0.1*startEffort and rospy.get_time()-startTime<8:
				rospy.sleep(0.1)
			if rospy.get_time()-startTime<8:
				seq.idn+=1

		elif seq.idn==60:
			pass

		elif seq.idn==61:
			rospy.loginfo('Module Complete.')
			rospy.signal_shutdown('Module Complete.') 

		self.pub_cmd(seq.idn)

## DEFINE IMPORTANT CONSTANTS ##
if __name__ == '__main__':
	# Commonly used objects
	joint_buttons = [-0.0393427734375+math.pi/2,-0.71621875,0.022359375,1.811921875,-0.116017578125,2.77036035156,-4.48708789063]

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
	joint_dof_up = [DSP,-0.4,j2scara,0,-j4max,0,j6scara]

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
	joint_dot_1 = [DSP,no_hit,j2scara,0,-j4max,-2.02,j6scara]
	joint_dot_2 = [-0.21,no_hit,j2scara,0,-j4max,0,j6scara]
	joint_arc_wrist = [joint_dot_2[0],no_hit,j2scara,0,-j4max,-j4max,j6scara]
	joint_ortho_kin = joint_arc_wrist[:]
	joint_ortho_kin[5] = -math.pi/2
	joint_arc_shoulder = joint_ortho_kin[:]
	joint_arc_shoulder[0] = -0.45

	# 42
	dot_3 = {'position': Limb.Point(x=1.00,y=0.31,z=0.06)}
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

	listener()