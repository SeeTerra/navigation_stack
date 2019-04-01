#!/usr/bin/env python

import tf
import atexit
import math
import yaml
import sys
import rospy
import numpy as np
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from fiducial_msgs.msg import *
from tf import transformations as t

class CamSlam:

	def __init__(self, cam_number, setup_mode):

		##ROS
		topic = "camera_%s/fiducial_transforms" % cam_number

		rospy.init_node("cam_slam_%s" % cam_number, anonymous=False, log_level=rospy.DEBUG)
		self.sub = rospy.Subscriber("camera_1/fiducial_transforms", FiducialTransformArray, self.updateCamLocation)
		self.rate = rospy.Rate(60) # in hz

		##Localization var
		self.cam_position = []
		self.tag_buffer = None
		self.setup_mode = setup_mode

		##Loop
		while not rospy.is_shutdown():
			self.main()
			self.rate.sleep()

	def main():
		""" Main Loop """

		#Grab recent tags detected
		fiducial_array = self.

		if self.setup_mode:
			self.setup_loop(fiducial_array)
		else:
			self.nav_loop(fiducial_array)

	def nav_loop(fiducial_array):
		""" Loop for navigation at runtime """
		print "unfinished"

	def setup_loop(fiducial_array):
		""" Loop for map generation """

		for tag in fiducial_array:
			print tag

	def tag_log(self):
		"""Retrieve a tag's world position"""
		print "unfinished"

	def tag_lookup(self):
		"""Log a tag's position into the database"""
		print "unfinished"

	def updateBuffer(self, msg):
		"""Update the camera's location in space based on the tags detected"""

		#TODO: Mutex Lock
		if msg.transforms:
			self.tag_buffer = msg

if __name__ == "__main__":
	cam_slam = CamSlam(1,True)
	rospy.spin()
