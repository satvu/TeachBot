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
from sawyer.msg import *

class Produce():
	
	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Initialize Sawyer
		#rospy.init_node('image', anonymous=True)
	
		# Publishing topics
		#suppress_cuff_interaction = rospy.Publisher('/robot/limb/right/suppress_cuff_interaction', Empty, queue_size=1)

		self.received = False
		# rospy.Subscriber('/Image', Image, self.get_image)

		self.img = np.zeros((480, 752,3), np.uint8)

		self.navigator = intera_interface.Navigator()

		# Initialization complete. Spin.
		# rospy.loginfo('Ready.')
		# r = rospy.Rate(10)
		# while not rospy.is_shutdown():
		# 	suppress_cuff_interaction.publish()
		# 	r.sleep()
	## HELPER FUNCTIONS ##
	# Returns true if two positions equal each other or are at least within a given tolerance

	# def get_image(self, data):

	# 	if self.received == False:
	# 		rospy.loginfo('Received') 
	# 		self.heights = data.heights
	# 		self.widths  = data.widths
	# 	#rospy.loginfo(self.widths)
	# 	self.received = True

	# 	self.produce_image(self.heights, self.widths)

	def produce_image(self, heights, widths):
		combined    = []
		filtered    = []
		
		img = np.zeros((480, 752,3), np.uint8)

		combined = list(map(lambda x, y: (y,x), heights, widths))

		# for j in combined:
		# 	if (j[0], j[1]+1) not in combined or (j[0], j[1]-1) not in combined:
		# 		filtered.append((j[1], j[0]))

		filtered = filter(lambda x: (x[0], x[1]+1) not in combined or (x[0], x[1]-1) not in combined, combined)
				
		#rospy.loginfo(self.distance(filtered[0], filtered[1]))

		# for i in filtered:
		# 	for l in filtered:
		# 		if self.distance(i, l) < 11:
		# 			cv2.line(img, i, l, (0,255,255), 2)

		for i in filtered:
			map(lambda x: cv2.line(img,i,x,(0,255,255),2) if self.distance(i,x) < 28 else None, filtered)

		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		ret, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

		im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

		cnt = contours[0]
		max_area = cv2.contourArea(cnt)

		for contour in contours:
			#cv2.drawContours(img2, contour, -1, (0,255,0), 1)
			if cv2.contourArea(contour) > max_area:
				cnt = contour
				max_area = cv2.contourArea(contour)

		epsilon = 0.01*cv2.arcLength(cnt,True)
		approx = cv2.approxPolyDP(cnt,epsilon,True)

		#rospy.loginfo(approx[0][2])

		cv2.drawContours(img, [approx], -1, (0,255,0), 3)


		a = sum(map(lambda x: x[0][0], approx))/approx.shape[0]
		b = sum(map(lambda x: x[0][1], approx))/approx.shape[0]

		#rospy.loginfo(a)
		cv2.circle(img, (a,b), 3, (255,0,0), -1)

		cv2.imshow('new', img)
		
		cv2.waitKey(3)

	def distance(self, a, b):
		dist = np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
		return dist

	def clean_shutdown(self):
		rospy.loginfo('Shutting down')
		cv2.destroyAllWindows()

## DEFINE IMPORTANT CONSTANTS ##
if __name__ == '__main__':
	Produce()
