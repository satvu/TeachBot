#! /usr/bin/env python

## IMPORTS ##
# Basic
import rospy, numpy, math
from pygame import mixer
from playsound import playsound

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

class Module():
	
	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Initialize Sawyer
		rospy.init_node('Sawyer_comm_node', anonymous=True)
		intera_interface.HeadDisplay().display_image('logo.png')

		# Publishing topics
		suppress_cuff_interaction = rospy.Publisher('/robot/limb/right/suppress_cuff_interaction', Empty, queue_size=1)
		self.endpoint_topic = rospy.Publisher('/EndpointInfo', EndpointInfo, queue_size=10)

		# Subscribing topics
		rospy.Subscriber('/robot/limb/right/endpoint_state', intera_core_msgs.msg.EndpointState, self.forwardEndpointState)

		# Lights
		self.lights = intera_interface.Lights()
		for light in self.lights.list_all_lights():
			self.lights.set_light_state(light,False)

		# Initialization complete. Spin.
		rospy.loginfo('Ready.')
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			suppress_cuff_interaction.publish()
			r.sleep()

	## HELPER FUNCTIONS ##
	# Returns true if two positions equal each other or are at least within a given tolerance

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


## DEFINE IMPORTANT CONSTANTS ##
if __name__ == '__main__':

	Module()
