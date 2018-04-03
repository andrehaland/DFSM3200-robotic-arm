#!/usr/bin/env python

import cv2
import numpy as np
import time
import rospy
from geometry_msgs.msg import Vector3


'''
Class used for marker detection for robotic arm. Markers are given as red dots/circles
and are masked in OpenCV in order to detect contours of the detected objects. The
location of the detected objects are published to a ROS topic after being mapped
from the x, y coordinate system of the OpenCV frame to a coordinate system fitting
the physical robot and rig.
'''
class ColorDetection():

	camera = None
	_, frame = None, None
	red_mask = None
	
	def __init__(self):
		rospy.loginfo("Initializing camera ..")
		self.camera = cv2.VideoCapture(0)
	
	def set_red_mask(self):
		lower_red = np.array([0, 50, 50])
		upper_red = np.array([10, 255, 255])
		mask0 = cv2.inRange(self.hsv, lower_red, upper_red)
		lower_red = np.array([170, 50, 50])
		upper_red = np.array([180, 255, 255])
		mask1 = cv2.inRange(self.hsv, lower_red, upper_red)
		mask = mask0 + mask1
		
		return mask

	# Function to detect objects in the OpenCV frame
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

	# Function to ready camera functionality
	def ready_camera(self):
		self._, self.frame = self.camera.read()
		self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		self.red_mask = self.set_red_mask()

	# Function to collect x, y coordinates while reading from OpenCV
	def collect_xy(self, x, y, count, temp):
		x = x + temp[0]
		y = y + temp[0]
		
		if count is not None:
			count = count + 1
		
		return x, y, count

	# Function to calculate the average of the collected set of x, y
	def calculate_xy(self, x, y, count):
		if count != 0:
			x = x / count
			y = y / count

		return x, y

	# Wrapping function to determine markers for the reference frame
	def set_reference_frame(self, x, y, count):
		for x in range(0, 50):
			self.ready_camera()
			temp = self.calibrate_coord_frame(self.red_mask)
			if temp != None:
				x, y, count = self.collect_xy(x, y, count, temp)
			cv2.imshow('frame', self.frame)
			k = cv2.waitKey(5)
			if k == 27:
				break;
			x, y = self.calculate_xy(x, y, count)

		return x, y

	# Wrapping function to run the script
	def run(self):
		x1, y1 = 0, 0
		x2, y2 = 0, 0
		x3, y3 = 0, 0
		tracker_x, tracker_y = 0, 0
		count1, count2, count3 = 0, 0, 0
		tracker_count = 0
		rospy.loginfo("Stabilizing picture ..")
		for x in range(0, 50):
			self._, self.frame = self.camera.read()
			self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
			cv2.imshow('frame', self.frame)
			k = cv2.waitKey(5)
			if k == 27:
				break;
		rospy.loginfo("Done stabilizing ..")
		
		# Set reference frame for origo
		rospy.loginfo("Reading reference location for origo ..")
		x1, y1 = self.set_reference_frame(x1, y1, count1)

		# Set reference marker for X-max
		rospy.loginfo("Place marker for x-reference ..")
		time.sleep(5000.0 / 1000.0)
		x2, y2 = self.set_reference_frame(x2, y2, count2)

		# Set reference marker for Y-max
		rospy.loginfo("Place marker for y-reference ..")
		time.sleep(5000.0 / 1000.0)		
		x3, y3 = self.set_reference_frame(x3, y3, count3)

		pub = rospy.Publisher('coordinates', Vector3, queue_size = 10)
		rospy.init_node('camera_feed', anonymous = True)
		rate = rospy.Rate(10)
		
		while not rospy.is_shutdown():
			self.ready_camera()
			temp = self.calibrate_coord_frame(self.red_mask)
			if temp != None:
				tracker_x = temp[0]
				tracker_y = temp[1]
			
			scaled_x = self.map_coordinate(tracker_x, x1, x2, 0, 0.19)
			scaled_y = self.map_coordinate(tracker_y, y1, y3, 0, 0.26)
			coordinates = Vector3()
			coordinates.x = scaled_x
			coordinates.y = scaled_y
			coordinates.z = 0.10
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
