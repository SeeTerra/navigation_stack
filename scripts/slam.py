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

class Slam:

	def __init__(self, setup_mode):

		##ROS
		rospy.init_node("slam", anonymous=False, log_level=rospy.DEVEL)
		self.sub_one = rospy.Subscriber("camera_1/transforms", FiducialTransformArray)
		self.sub_two = rospy.Subscriber("camera_2/transforms", FiducialTransformArray)
		self.rate = rospy.Rate(60) # in hz

		self.tag_log_service = rospy.ServiceProxy('tag_log', TagLog)
		self.tag_lookup_service = rospy.ServiceProxy('tag_lookup', TagLookup)

		##Localization var
		self.origin_set = False
		self.position = None
		self.tag_array_tmp = None
		self.camera_1_to_base = None
		self.camera_2_to_base = None
		self.setup_mode = setup_mode
		self.tag_known_buffer = []
		self.tag_unknown_buffer = []
		self.new_tag_buffer = []

		##Loop
		while not rospy.is_shutdown():
			self.main()
			self.rate.sleep()

	def main(self):
		""" Main Loop """
