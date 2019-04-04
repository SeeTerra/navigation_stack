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
from navigation_stack.srv import *
from tf import transformations as t

class CamSlam:

	def __init__(self, cam_number, setup_mode):

		##ROS
		topic = "camera_%s/fiducial_transforms" % cam_number

		rospy.init_node("cam_slam_%s" % cam_number, anonymous=False, log_level=rospy.DEBUG)
		self.sub = rospy.Subscriber("camera_1/fiducial_transforms", FiducialTransformArray, self.updateBuffer, queue_size=1)
		self.rate = rospy.Rate(60) # in hz

		self.tag_log_service = rospy.ServiceProxy('tag_log',TagLog)
		self.tag_lookup_service = rospy.ServiceProxy('tag_lookup',TagLookup)

		##Localization var
		self.origin_set = False
		self.cam_position = None
		self.tag_array_tmp = None
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

		#Grab recent tags detected
		fiducial_array = self.tag_array_tmp

		if fiducial_array is None:
			return

		if self.setup_mode:
			self.setup_loop(fiducial_array)
		else:
			self.nav_loop(fiducial_array)

	def nav_loop(self, fiducial_array):
		""" Loop for navigation at runtime """
		print "unfinished"

	def origin_loop(self, fiducial_array):
		""" Loop for setting origin """

		origin_tags = [0,1,2,3]
		origin_tags_info = [None,None,None,None]
		tag_count = 0
		for tag in fiducial_array:
			if tag.fiducial_id in origin_tags:
				tag_count = tag_count + 1
				origin_tags_info[tag.fiducial_id] = tag.transform
			if tag_count == 4:
				rospy.loginfo("Setting Origin...")
				self.tag_log_service(0,Transform())
				self.origin_set = True

		print tag_count

	def setup_loop(self, fiducial_array, tracking_threshold=1, new_tag_threshold=10):
		""" Loop for map generation """

		if not self.origin_set:
			rospy.loginfo("Origin Setup: Focus Tags 0-3")
			self.origin_loop(fiducial_array)
			return

		# Sort based on whether it's in database
		for tag in fiducial_array:
			try:
				response = self.tag_lookup_service(tag.fiducial_id)
				if response.id != -1: #In database
					self.tag_known_buffer.append(response)
				else:
					self.tag_unknown_buffer.append(tag)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

		# Localize based on known tags
		if len(self.tag_known_buffer) >= tracking_threshold:
			tracking = True
		else:
			tracking = False
		localize_tmp = []
		#TEMP: This needs to be fixed and moved to sthe main slam node
		for tag in self.tag_known_buffer:
			localize_tmp.append(inverseTransform(tag.transform))
		if tracking:
			sum(localize_tmp)/len(localize_tmp)

		# Localize unknown tags
		if tracking:
			for tag in self.tag_unknown_buffer:
				tag_position = addTransforms(self.cam_position, inverseTransform(tag.transform))
				if self.new_tag_buffer[0] == None:
					self.new_tag_buffer.append(tag.id)
				if self.new_tag_buffer[0] == tag.id:
					self.new_tag_buffer.append(tag_position)
					if len(self.new_tag_buffer) >= new_tag_threshold:
						self.tag_log_service(tag.fiducial_id, tag_position)
						self.new_tag_buffer = []

		# Reset buffer
		self.tag_known_buffer = []
		self.tag_unknown_buffer = []

	def tag_log(self):
		"""Log a tag's position into the database"""
		print "unfinished"

	def tag_lookup(self):
		"""Retrieve a tag's world position, returns None if nothing found"""
		print "unfinished"

	def updateBuffer(self, msg):
		"""Update the camera's location in space based on the tags detected"""

		#TODO: Mutex Lock
		if msg.transforms:
			self.tag_array_tmp = msg.transforms

#####################TEMP#######################################################
def addTransforms(frame1, frame2):
	""" Adds two transform objects to get a resulting transform. """

	t1 = np.array([frame1.translation.x,frame1.translation.y,frame1.translation.z])
	t2 = np.array([frame2.translation.x,frame2.translation.y,frame2.translation.z])

	r1 = np.array([frame1.rotation.x, frame1.rotation.y, frame1.rotation.z, frame1.rotation.w])
	r2 = np.array([frame2.rotation.x, frame2.rotation.y, frame2.rotation.z, frame2.rotation.w])

	f1 = t.compose_matrix(translate=t1,angles=t.euler_from_quaternion(r1))
	f2 = t.compose_matrix(translate=t2,angles=t.euler_from_quaternion(r2))

	f = np.matmul(f1,f2)

	trans = t.translation_from_matrix(f)
	rot = t.quaternion_from_matrix(f)

	trans = Vector3(trans[0],trans[1],trans[2])
	rot = Quaternion(rot[0],rot[1],rot[2],rot[3])

	return Transform(trans, rot)

def arrayify(n):
	""" Turn the transform into two nparrays, translation and rotation """

	trans = np.array([n.translation.x, n.translation.y, n.translation.z])
	rot = np.array([ n.rotation.x, n.rotation.y, n.rotation.z, n.rotation.w])

	return trans, rot

def inverseTransform(input_transform):

	translation, rotation = arrayify(input_transform)

	transform = t.compose_matrix(translate=translation,angles=t.euler_from_quaternion(rotation))
	inversed_transform = t.inverse_matrix(transform)

	translation = t.translation_from_matrix(inversed_transform)
	quaternion = t.quaternion_from_matrix(inversed_transform)

	final_transform = transformify(translation, quaternion)

	return final_transform

def transformify(t, r):
	""" Turns a translation and rotation to a transform object """

	translation = Vector3(t[0],t[1],t[2])
	rotation= Quaternion(r[0],r[1],r[2],r[3])
	transform = Transform(translation, rotation)

	return transform
################################################################################

if __name__ == "__main__":
	cam_slam = CamSlam(1,True)
	rospy.spin()
