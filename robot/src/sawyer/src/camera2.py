#! /usr/bin/env python

## IMPORTS ##
# Basic
import rospy, math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

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
from sensor_msgs.msg import Image
import intera_core_msgs

# Custom
from limb_plus import LimbPlus
from image import Produce
from sawyer.msg import *

class Camera():
	
	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Initialize Sawyer
		rospy.init_node('camera', anonymous=True)
	
		# Publishing topics
		suppress_cuff_interaction = rospy.Publisher('/robot/limb/right/suppress_cuff_interaction', Empty, queue_size=1)
		self.image_topic = rospy.Publisher('/image_rect', Image)

		self.limb = LimbPlus()
		self.produce = Produce()
		self.heights = []
		self.widths = []

		self.limb.go_to_joint_angles([0.314609375, 0.19108984375, -1.9322587890625, 1.64379296875, 1.7396455078125, 0.37216796875, 0.183701171875])

		# rospy.sleep(3)

		self.start()

		# Initialization complete. Spin.
		rospy.loginfo('Ready.')
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			suppress_cuff_interaction.publish()
			r.sleep()

	def display_camera_callback(self, img_data): 
		bridge = CvBridge()
		# rospy.loginfo(img_data.height)
		try:
			cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')
			# self.image_topic.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
		except CvBridgeError, err:
			rospy.logerr(err)

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5,5), 0)
		ret, thresh = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY_INV)

		im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

		cnt = contours[0]
		max_area = cv2.contourArea(cnt)

		for contour in contours:
			if cv2.contourArea(contour) > max_area:
				cnt = contour
				max_area = cv2.contourArea(contour)

		rect = cv2.minAreaRect(cnt)
		box = cv2.boxPoints(rect)
		box = np.int0(box)

		cv2.drawContours(cv_image, [box], -1, (0,255,0), 3)

		a = sum(map(lambda x: x[0], box))/4
		b = sum(map(lambda x: x[1], box))/4

		# cv2.circle(cv_image, (a,b), 8, (255,0,0), -1)

		cv2.imwrite('/home/albertgo/TeachBot/browser/public/images/cv_image.png', cv_image)

		cv2.imshow('new', cv_image)

		# cv2.imshow('binary', thresh)
		
		cv2.waitKey(5)
		
		# msg = Image()
		# msg.height       = img_data.height
		# msg.width        = img_data.width
		# msg.encoding     = img_data.encoding
		# msg.is_bigendian = img_data.is_bigendian
		# msg.step         = img_data.step
		# msg.data         = img_data.data 

		#self.image_topic.publish(msg)

	def start(self):
		rp = intera_interface.RobotParams()
		valid_cameras = rp.get_camera_names()
		camera = intera_interface.Cameras()
		if not camera.verify_camera_exists('right_hand_camera'):
			rospy.logerr('Invalid camera name')
		camera.start_streaming('right_hand_camera')
		rectify_image = False
		camera.set_callback('right_hand_camera', self.display_camera_callback)

	def clean_shutdown(self):
		rospy.loginfo('Shutting down')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	Camera()