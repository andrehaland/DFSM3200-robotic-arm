#!/usr/bin/env python

import argparse
import cv2
import numpy as np


class ColorDetection():

	camera = None

	_ = None
	frame = None

	ap = None
	args = None
	hsv = None
	
	lower_green = None
	upper_green = None

	lower_blue = None
	upper_blue = None
	
	lower_yellow = None
	upper_yellow = None

	green_mask = None
	red_mask = None
	blue_mask = None
	yellow_mask = None
	
	first = None
	second = None
	third = None
	fourth = None

	def __init__(self):
		print('Initializing camera..')
		self.camera = cv2.VideoCapture(0)
		print('Initializing argument parser..')
		self.ap = argparse.ArgumentParser()
		print('Setting colorspace..')
		self.set_hsv_colorspace()
		self.run()

	# Function to define the colorspace in hsv
	def set_hsv_colorspace(self):
		self.lower_green = np.array([36, 0, 0])
		self.upper_green = np.array([86, 255, 255])

		self.lower_blue = np.array([110, 50, 50])
		self.upper_blue = np.array([130, 255, 255])

		self.lower_yellow = np.array([20, 100, 100])
		self.upper_yellow = np.array([30, 255, 255])

	# Function to set color masks for the different colors used
	def set_color_mask(self, mask, lower, upper):
		mask = cv2.inRange(self.hsv, lower, upper)
		mask = cv2.erode(mask, None, iterations = 10)
		mask = cv2.dilate(mask, None, iterations = 10)
		
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
		green_x = 0
		green_y = 0
		green_count = 0
		blue_x = 0
		blue_y = 0
		blue_count = 0
		red_x = 0
		red_y = 0
		red_count = 0
		yellow_x = 0
		yellow_y = 0
		yellow_count = 0
		corner_4 = None
		done = False

		for x in range(0, 100):
			self._, self.frame = self.camera.read()
			self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
#			self.green_mask = self.set_color_mask(self.green_mask, self.lower_green, self.upper_green)
			self.blue_mask = self.set_color_mask(self.blue_mask, self.lower_blue, self.upper_blue)
			self.red_mask = self.set_red_mask()
			
#			green_temp = self.calibrate_coord_frame(self.green_mask)
#			if green_temp != None:
#				green_x = green_x + green_temp[0]
#				green_y = green_y + green_temp[1]
#				green_count = green_count + 1

			blue_temp = self.calibrate_coord_frame(self.blue_mask)
			if blue_temp != None:
				blue_x = blue_x + blue_temp[0]
				blue_y = blue_y + blue_temp[1]
				blue_count = blue_count + 1

			red_temp = self.calibrate_coord_frame(self.red_mask)
			if red_temp != None:
				red_x = red_x + red_temp[0]
				red_y = red_y + red_temp[1]
				red_count = red_count + 1

#			yellow_temp = self.calibrate.coord_frame(self.yellow_mask)
#			if yellow_temp != None:
#				yellow_x = yellow_x + yellow_temp[0]
#				yellow_y = yellow_y + yellow_temp[0]
#				yellow_count = yellow_count + 1

			cv2.imshow('frame', self.frame)

			k = cv2.waitKey(5)
			if k == 27:
				break;

#		if green_count != 0:
#			green_x = green_x / green_count
#			green_y = green_y / green_count

		if blue_count != 0:
			blue_x = blue_x / blue_count
			blue_y = blue_y / blue_count
		
		if red_count != 0:
			red_x = red_x / red_count
			red_y = red_y / red_count

#		if yellow_count != 0:
#			yellow_x = yellow_x / yellow_count
#			yellow_y = yellow_y / yellow_count

		#print("Green: ", green_x, green_y)
		print("Blue: ", blue_x, blue_y)
		print("Red: ", red_x, red_y)
		#print("Yellow: ", yellow_x, yellow_y)

		while True:
			self._, self.frame = self.camera.read()
			cv2.imshow('frame', self.frame)

			k = cv2.waitKey(5)
			if k == 27:
				break;

		cv2.destroyAllWindows()
		self.camera.release()

cd = ColorDetection()
