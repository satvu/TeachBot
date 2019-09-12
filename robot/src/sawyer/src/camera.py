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

		#self.image_topic = rospy.Publisher('/Image', Image, queue_size=10)

		self.limb = LimbPlus()
		self.produce = Produce()
		self.heights = []
		self.widths = []

		#self.navigator = intera_interface.Navigator()
		#self.navigator.register_callback(self.rx_finished, 'right_button_ok')

		# self.finished = False
		# self.limb.interaction_control(orientation_x=True, orientation_y=True, orientation_z=True, position_x=True, position_y=True, position_z=True)
		# while not self.finished:
		# 	pass

		# self.limb.interaction_control(position_only=False)
		# self.limb.go_to_joint_angles([0.126298828125, 0.184927734375, -2.04283203125, 1.1082080078125, 2.0063212890625, 0.343841796875, -0.0108408203125])

		# rospy.sleep(3)

		self.start()

		# Initialization complete. Spin.
		rospy.loginfo('Ready.')
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			suppress_cuff_interaction.publish()
			r.sleep()
	## HELPER FUNCTIONS ##
	# Returns true if two positions equal each other or are at least within a given tolerance

	def display_camera_callback(self, img_data): 
		bridge = CvBridge()
		try:
			cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')
		except CvBridgeError, err:
			rospy.logerr(err)

		#rospy.loginfo('Ready to show')

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5,5), 0)
		ret, thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

		# height = thresh.shape[0]
		# width  = thresh.shape[1]

		# heights, widths = self.two_lists(height, width, thresh)
		# heights = []
		# widths = []
		# for i in range(height):
		# 	if sum(thresh[i]) > 255:
		# 		for j in range(width):
		# 			if thresh[i][j] == 255:
		# 				heights.append(i)
		# 				widths.append(j)

		# heights = [v for v in range(height) if sum(thresh[v]) > 255 for w in range(width) if thresh[v][w] == 255]
		# widths = [w for v in range(height) if sum(thresh[v]) > 255 for w in range(width) if thresh[v][w] == 255]

		# img = np.zeros((480, 752,3), np.uint8)

		# combined = list(map(lambda x, y: (y,x), heights, widths))

		# filtered = filter(lambda x: (x[0], x[1]+1) not in combined or (x[0], x[1]-1) not in combined, combined)

		# for i in filtered:
		# 	map(lambda x: cv2.line(img,i,x,(0,255,255),2) if self.distance(i,x) < 12 else None, filtered)

		# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# ret, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

		im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

		cnt = contours[0]
		max_area = cv2.contourArea(cnt)

		for contour in contours:
			#cv2.drawContours(cv_image, contour, -1, (0,255,0), 1)
			if cv2.contourArea(contour) > max_area:
				cnt = contour
				max_area = cv2.contourArea(contour)

		# epsilon = 0.01*cv2.arcLength(cnt,True)
		# approx = cv2.approxPolyDP(cnt,epsilon,True)

		rect = cv2.minAreaRect(cnt)
		box = cv2.boxPoints(rect)
		box = np.int0(box)

		#rospy.loginfo(box)

		# cv2.drawContours(img, [approx], -1, (0,255,0), 3)
		cv2.drawContours(cv_image, [box], -1, (0,255,0), 3)

		# a = sum(map(lambda x: x[0][0], approx))/approx.shape[0]
		# b = sum(map(lambda x: x[0][1], approx))/approx.shape[0]

		a = sum(map(lambda x: x[0], box))/4
		b = sum(map(lambda x: x[1], box))/4

		cv2.circle(cv_image, (a,b), 8, (255,0,0), -1)

		cv2.imshow('new', cv_image)


		#self.produce.produce_image(heights, widths)
		# msg = Image()
		# msg.heights = heights
		# msg.widths = widths
		# self.image_topic.publish(msg)
		
		#cv2.imshow('live_image', cv_image)
		#cv2.imshow('filter', thresh)
		
		cv2.waitKey(3)

	def start(self):
		rp = intera_interface.RobotParams()
		valid_cameras = rp.get_camera_names()
		camera = intera_interface.Cameras()
		if not camera.verify_camera_exists('right_hand_camera'):
			rospy.logerr('Invalid camera name')
		camera.start_streaming('right_hand_camera')
		rectify_image = False
		camera.set_callback('right_hand_camera', self.display_camera_callback)

	def rx_finished(self,data):
		if self.navigator.button_string_lookup(data) == 'OFF':
			self.finished = True

	def distance(self, a, b):
		dist = np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
		return dist

	def two_lists(self, height, width, thresh):
		heights = []
		widths  = []
		for i in range(height):
			if sum(thresh[i]) > 255:
				for j in range(width):
					if thresh[i][j] == 255:
						heights.append(i)
						widths.append(j)
		return heights, widths

	def clean_shutdown(self):
		rospy.loginfo('Shutting down')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	Camera()
