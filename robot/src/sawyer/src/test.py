#! /usr/bin/env python

#testing various python functions and ros communication via rosbridge
import os, rospy, math, pexpect, copy
from std_msgs.msg import (
	String,
	Int32
)

# Intera
import intera_interface 
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Lights,
    Cuff,
    RobotParams,
    Limb
)

# Custom
from module import (Module, Sequence)

# Constants
Z_TABLE = 0.12
BOX_HEIGHT = 0.110
BIN_HEIGHT = 0.005

class listener(Module):
	def __init__(self):
		Module.__init__(self)

	def rx_command(self, data):
		seq = Sequence(data.data, rospy.get_time())
		if self.VERBOSE: rospy.loginfo("I heard: " + str(seq.idn))

		if seq.idn==0:
			pass

		self.addSeq(seq)
		self.pub_cmd(seq.idn)

	def post_init(self):
		self.limb.go_to_joint_angles(above_bin_cart_arg)
		self.limb.adjustOrientationTo(x=-0.707,y=0.707,z=0,w=0)

if __name__ == '__main__':
	init_joint_arg                = [1.133,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625]
	fail_init_joint_arg           = [1.06155175781,-0.26884765625,-1.4501015625,0.838840820312,-1.88630175781,-1.62122851562,1.9701875]
	fail_pickup_cart_arg          = [1.08375488281,0.175158203125,-1.53774609375,1.02942480469,-1.45563085938,-1.45510351563,1.86297558594]
	fail_above_pickup_cart_arg    = [1.06155175781,-0.26884765625,-1.4501015625,0.838840820312,-1.88630175781,-1.62122851562,1.9701875]

	joint_buttons                 = [math.pi/2-0.0393427734375,-0.71621875,0.022359375,1.811921875,-0.116017578125,2.77036035156,-4.48708789063]

	success_init_joint_arg        = [1.05569238281,-0.176821289062,-1.62947851563,0.904107421875,-1.66179296875,-1.63658007812,0.266885742187]
	success_pickup_joint_arg      = [1.06658789062,0.264276367188,-1.62918945312,1.05704492187,-1.31784570312,-1.50530078125,0.266266601562]
	success_pickup_cart_arg       = [1.06658789062,0.264276367188,-1.62918945312,1.05704492187,-1.31784570312,-1.50530078125,0.266266601562]
	success_above_pickup_cart_arg = [1.05569238281,-0.176821289062,-1.62947851563,0.904107421875,-1.66179296875,-1.63658007812,0.266885742187]

	above_bin_cart_arg            = [0.618388671875,-0.048005859375,-1.47402246094,1.26808496094,-1.69567089844,-1.5480234375,2.70501367188]
	rotated_above_bin_cart_arg    = [0.938936523438,-0.166234375,-1.35050976563,1.20908984375,-1.8427265625,-1.403859375,1.51214648438]
	into_bin_cart_arg             = [0.943045898437,0.233333984375,-1.50463964844,1.30310449219,-1.3893203125,-1.43581445312,1.4121640625]

	into_second_bin_cart_arg      = [0.561075195313,0.2966484375,-1.81974609375,0.759317382813,-1.23418261719,-1.49326855469,0.08444140625]
	above_second_bin_cart_arg     = [0.5035625,0.0966318359375,-1.882125,0.744194335937,-1.22447363281,-1.62413378906,0.002185546875]

	above_second_box_cart_arg     = [math.pi/2-0.729661132813,0.12510546875,-1.78256054688,1.094546875,-1.35895410156,-1.71990722656,4.66472753906]
	above_third_box_cart_arg      = [0.317999023437,0.0488310546875,-1.646234375,-0.249951171875,-1.51264550781,-1.61024121094,-2.44349414062]
	above_fourth_box_cart_arg     = [math.pi/2-1.482765625,0.17649609375,-2.27079101562,0.363586914062,-0.855272460937,-1.6052578125,1.5651328125]

	listener()