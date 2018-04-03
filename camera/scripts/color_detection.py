#!/usr/bin/env python

import cv2
import numpy as np
import time
import rospy
from geometry_msgs.msg import Vector3

class ColorDetection():

	camera = None
	_, frame = None, None
	red_mask = None

	def __init__(self):
		print('Initializing camera..')
		self.camera = cv2.VideoCapture(1)

	# Function to set color masks for the different colors used
	def set_color_mask(self, mask, lower, upper):
		mask = cv2.inRange(self.hsv, lower, upper)
		mask = cv2.erode(mask, None, iterations = 50)
		mask = cv2.dilate(mask, None, iterations = 50)
		
		return mask
	
	def set_red_mask(self):
		lower_red = np.array([0, 50, 50])
		upper_red = np.array([10, 255, 255])
		mask0 = cv2.inRange(self.hsv, lower_red, upper_red)
		lower_red = np.array([170, 50, 50])
		upper_red = np.array([180, 255, 255])
		mask1 = cv2.inRange(self.hsv, lower_red, upper_red)
		mask = mask0 + mask1

		return mask

	# Function to define the four corners of the coordinate frame based on four colors
	def calibrate_coord_frame(self, mask):
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
		
		if len(contours) > 0:
			c = max(contours, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			if (int(M["m00"])) != 0:
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			if radius > 10:
				cv2.circle(self.frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
		
                return center
	
	# Function to map coordinates from OpenCV frame to real coordinates
	def map_coordinate(self, desired, cv_min, cv_max, real_min, real_max):
		cv_span = cv_max - cv_min
		real_span = real_max - real_min
		desired_scaled = float(desired - cv_min) / float(cv_span)

		return real_min + (desired_scaled * real_max)

	def run(self):
		x1, y1 = 0, 0
		x2, y2 = 0, 0
		x3, y3 = 0, 0
		tracker_x, tracker_y = 0, 0
		count1, count2, count3 = 0, 0, 0
		tracker_count = 0
		
		# Stabilize colors in picture
		print("Stabilizing picture..")
		for x in range(0, 50):
			self._, self.frame = self.camera.read()
			self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
			cv2.imshow('frame', self.frame)
			k = cv2.waitKey(5)
			if k == 27:
				break;
		print("Stabilizing done..")	
		print("Reading reference location for origo..")

		# Read reference for origo
		for x in range(0, 50):
			self._, self.frame = self.camera.read()
			self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
			self.red_mask = self.set_red_mask()
			temp = self.calibrate_coord_frame(self.red_mask)

			if temp != None:
				x1 = x1 + temp[0]
				y1 = y1 + temp[1]
				count1 = count1 + 1

			cv2.imshow('frame', self.frame)
			k = cv2.waitKey(5)
			
                        if k == 27:
				break;

		if count1 != 0:
			x1 = x1 / count1
			y1 = y1 / count1
	
		print("Place marker for x-reference")
		time.sleep(5000.0 / 1000.0)
		
		for x in range(0, 50):
			self._, self.frame = self.camera.read()
			self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
			self.red_mask = self.set_red_mask()
			temp = self.calibrate_coord_frame(self.red_mask)

			if temp != None:
				x2 = x2 + temp[0]
				y2 = y2 + temp[1]
				count2 = count2 + 1

			cv2.imshow('frame', self.frame)
			k = cv2.waitKey(5)
			if k == 27:
				break;

		if count2 != 0:
			x2 = x2 / count2
			y2 = y2 / count2

		print("Place marker for y-reference")
		time.sleep(5000.0 / 1000.0)

		for x in range(0, 50):
			self._, self.frame = self.camera.read()
			self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
			self.red_mask = self.set_red_mask()
			temp = self.calibrate_coord_frame(self.red_mask)

			if temp != None:
				x3 = x3 + temp[0]
				y3 = y3 + temp[1]
				count3 = count3 + 1

			cv2.imshow('frame', self.frame)
			k = cv2.waitKey(5)
			if k == 27:
				break;

		if count3 != 0:
			x3 = x3 / count3
			y3 = y3 / count3

		print("Origo-reference: ", x1, y1)
		print("X-reference: ", x2, y2)
		print("Y-reference: ", x3, y3)
		pub = rospy.Publisher('coordinates', Vector3, queue_size = 10)
		rospy.init_node('camera_feed', anonymous = True)
		rate = rospy.Rate(10)
		

		while not rospy.is_shutdown():
			self._, self.frame = self.camera.read()
			self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
			self.red_mask = self.set_red_mask()
			temp = self.calibrate_coord_frame(self.red_mask)

			if temp != None:
				tracker_x = temp[0]
				tracker_y = temp[1]
			
			scaled_x = self.map_coordinate(tracker_x, x1, x2, 0, 0.19)
			scaled_y = self.map_coordinate(tracker_y, y1, y3, 0, 0.26)
			coordinates = Vector3()
			coordinates.x = scaled_x
			coordinates.y = scaled_y
			coordinates.z = 0.15
			rospy.loginfo(coordinates)
			pub.publish(coordinates)
			cv2.imshow('frame', self.frame)
			rate.sleep()
			k = cv2.waitKey(5)

			if k == 27:
				break;

		cv2.destroyAllWindows()
		self.camera.release()

if __name__ == '__main__':
	cd = ColorDetection()
	cd.run()
