#!/usr/bin/env python

import tf
import atexit
import math
import yaml
import sys
import rospy
import signal
import numpy as np
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from fiducial_msgs.msg import *
from navigation_stack.srv import *
from tf import transformations as t
from scipy import stats

class Slam:

	def __init__(self, setup_mode):

		##ROS
		rospy.init_node("slam", anonymous=False, log_level=rospy.DEBUG)
		self.sub_one = rospy.Subscriber("camera_1/transform_array", FiducialTransformArray, self.updateBuffer1)
		self.sub_two = rospy.Subscriber("camera_2/transform_array", FiducialTransformArray, self.updateBuffer2)
		self.rate = rospy.Rate(20) # in hz

		self.tag_log_service = rospy.ServiceProxy('tag_log', TagLog)
		self.tag_lookup_service = rospy.ServiceProxy('tag_lookup', TagLookup)
		self.tag_save_service = rospy.ServiceProxy('tag_save', TagSave)
		self.tag_load_service = rospy.ServiceProxy('tag_load', TagLoad)

		self.br = tf.TransformBroadcaster()
		self.ls = tf.TransformListener()

		##Localization var
		#Origin
		self.origin_set = False
		self.origin_temp_1 = []
		self.origin_temp_2 = []
		self.origin_temp_3 = []

		tmp = Transform()
		tmp.rotation.w = 1
		self.position = tmp
		self.position_rolling_avg = []
		self.position_buffer_size = 3
		self.current_row = 0
		self.buffer1 = None
		self.buffer2 = None
		self.buffer1_old = None
		self.buffer2_old = None
		self.camera_1_to_base = None
		self.camera_2_to_base = None
		self.setup_mode = setup_mode
		self.tag_known_buffer = []
		self.tag_unknown_buffer = []
		self.new_tag_buffer_1 = []
		self.new_tag_buffer_2 = []
		self.new_tag_buffer_3 = []
		self.new_tag_db = {1: None,
							2: None,
							3: None}

		signal.signal(signal.SIGINT, self.shutdown)

		if not setup_mode:
			rospy.wait_for_service("tag_load")
			self.tag_load_service()

		print "Setup Mode: %s" % setup_mode

		##Loop
		while not rospy.is_shutdown():
			self.main()
			self.rate.sleep()

	def determine_row(self, tags, starting_tag=101, tags_per_row=2):
		"""Determine the current row algorithmically"""

		tmp_row = []

		for tag in tags:
			id = int(tag.fiducial_id)
			if id < starting_tag:
				continue
			even = (int(id)%2 == 0)
			row = (id-starting_tag+1)/tags_per_row
			if not even:
				row = row + 1
			tmp_row.append(row)

		self.current_row = stats.mode(tmp_row)[0]

		#DEBUG:
		print self.current_row

	def main(self):
		""" Main Loop """

		#Grab recent tags detected
		fiducial_array = self.mergeBuffers()

		if fiducial_array == None:
			return

		# #DEBUG: (check camera calibration + tag reception)
		try:
			for tag in fiducial_array:

				t,r = arrayify(tag.transform)
				print(r)
				self.br.sendTransform(t,r,rospy.Time.now(), "/tags/" + str(tag.fiducial_id), "/world")
		except Exception as e:
			rospy.logerr(e)
			return

		if self.setup_mode:
			self.setup_loop(fiducial_array)
		else:
			self.nav_loop(fiducial_array)

	def nav_loop(self, fiducial_array, tracking_threshold=1):
		""" Loop for navigation at runtime """

		# Sort based on whether it's in database
		absolute_tag_transforms = {}
		for tag in fiducial_array:
			try:
				response = self.tag_lookup_service(tag.fiducial_id)
				if response.id != -1: #In database
					self.tag_known_buffer.append(tag)
					absolute_tag_transforms[response.id] = response.transform
				else:
					self.tag_unknown_buffer.append(tag)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

		## Localize based on known tags
		# Determine if we have enough known tags to start tracking
		if len(self.tag_known_buffer) >= tracking_threshold:
			tracking = True
		else:
			tracking = False

		# Determine row
		self.determine_row(self.tag_known_buffer)

		# Average at each timestep
		localize_tmp = []
		for tag in self.tag_known_buffer:
			t, r = arrayify(addTransforms(absolute_tag_transforms[tag.fiducial_id],inverseTransform(tag.transform)))
			localize_tmp.append([t,r])
		if tracking:
			localize_tmp = np.average(localize_tmp,axis=0)
			if len(self.position_rolling_avg) < self.position_buffer_size:
				self.position_rolling_avg.append(localize_tmp)
			else:
				avg = np.average(self.position_rolling_avg, axis=0)
				self.position = transformify(avg[0],avg[1])
				t,r = arrayify(self.position)
				#self.br.sendTransform(t,r,rospy.Time.now(),"robot","/world")
				self.position_rolling_avg = self.position_rolling_avg[1:]
				self.position_rolling_avg.append(localize_tmp)

		# DEBUG: Broadcast position & known tags
		try:
			t,r = arrayify(self.position)
			self.br.sendTransform(t,r,rospy.Time.now(), "robot", "/world")
		except:
			rospy.logwarn("Position not yet localized")

		for tag in absolute_tag_transforms: #for tag in absolute_tag_transforms
			t,r = arrayify(absolute_tag_transforms[tag])
			self.br.sendTransform(t,r,rospy.Time.now(),"/tags/%s" % tag, "/world")

		# Flush buffer
		self.tag_known_buffer = []
		self.tag_unknown_buffer = []

	def origin_loop(self, fiducial_array, samples=20):
		""" Loop for establishing origin """

		origin_tags = [0,1,2,3]
		origin_tags_info = [None,None,None,None]
		tag_count = 0
		for tag in fiducial_array:
			if tag.fiducial_id in origin_tags:
				tag_count = tag_count + 1
				origin_tags_info[tag.fiducial_id] = tag.transform
		if tag_count == 4:
			t1, r1 = arrayify(addTransforms(inverseTransform(origin_tags_info[0]),origin_tags_info[1]))
			t2, r2 = arrayify(addTransforms(inverseTransform(origin_tags_info[0]),origin_tags_info[2]))
			t3, r3 = arrayify(addTransforms(inverseTransform(origin_tags_info[0]),origin_tags_info[3]))
			self.origin_temp_1.append([t1,r1])
			self.origin_temp_2.append([t2,r2])
			self.origin_temp_3.append([t3,r3])
			if len(self.origin_temp_1) == samples:
				rospy.loginfo("Setting Origin...")
				temp_1 = np.array(self.origin_temp_1)
				temp_2 = np.array(self.origin_temp_2)
				temp_3 = np.array(self.origin_temp_3)
				origin_transform = Transform()
				origin_transform.rotation.w = 1
				self.tag_log_service(0,origin_transform)
				transform_1 = np.average(temp_1,axis=0)
				transform_2 = np.average(temp_2,axis=0)
				transform_3 = np.average(temp_3,axis=0)
				self.tag_log_service(1,transformify(transform_1[0], transform_1[1]))
				self.tag_log_service(2,transformify(transform_2[0], transform_2[1]))
				self.tag_log_service(3,transformify(transform_3[0], transform_3[1]))
				self.origin_set = True


		print tag_count

	def setup_loop(self, fiducial_array, tracking_threshold=1, new_tag_threshold=35):
		""" Loop for map generation """

		if not self.origin_set:
			rospy.loginfo("Origin Setup: Focus Tags 0-3")
			self.origin_loop(fiducial_array)
			return

		# Sort based on whether it's in database
		absolute_tag_transforms = {}
		for tag in fiducial_array:
			try:
				response = self.tag_lookup_service(tag.fiducial_id)
				if response.id != -1: #In database
					self.tag_known_buffer.append(tag)
					absolute_tag_transforms[response.id] = response.transform
				else:
					self.tag_unknown_buffer.append(tag)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

		## Localize based on known tags
		# Determine if we have enough known tags to start tracking
		if len(self.tag_known_buffer) >= tracking_threshold:
			tracking = True
		else:
			tracking = False

		# Average at each timestep
		localize_tmp = []
		for tag in self.tag_known_buffer:
			t, r = arrayify(addTransforms(absolute_tag_transforms[tag.fiducial_id],inverseTransform(tag.transform)))
			localize_tmp.append([t,r])
		rospy.logerr(localize_tmp)
		if tracking:
			localize_tmp = np.average(localize_tmp,axis=0)
			if len(self.position_rolling_avg) < self.position_buffer_size:
				self.position_rolling_avg.append(localize_tmp)
			else:
				avg = np.average(self.position_rolling_avg, axis=0)
				self.position = transformify(avg[0],avg[1])
				#t,r = arrayify(self.position)
				#self.br.sendTransform(t,r,rospy.Time.now(),"robot","/world")
				self.position_rolling_avg = self.position_rolling_avg[1:]
				self.position_rolling_avg.append(localize_tmp)


		## Localize unknown tags
		if tracking:
			for tag in self.tag_unknown_buffer:
				# tag_position = addTransforms(self.position,tag.transform)
				# t, r = arrayify(tag_position)
				# if not self.new_tag_buffer:
				# 	self.new_tag_buffer.append(tag.fiducial_id)
				# if self.new_tag_buffer[0] == tag.fiducial_id:
				# 	self.new_tag_buffer.append([t,r])
				# 	if len(self.new_tag_buffer) >= new_tag_threshold:
				# 		avg = np.average(self.new_tag_buffer[1:], axis=0)
				# 		self.tag_log_service(tag.fiducial_id, tag_position)
				# 		self.new_tag_buffer = []

				tag_position = addTransforms(self.position, tag.transform)
				t, r = arrayify(tag_position)

				if not self.new_tag_buffer_1 or self.new_tag_db[1] == tag.fiducial_id:
					rospy.logdebug("LOGGING %s IN new_tag_buffer_1" % tag.fiducial_id)
					self.new_tag_buffer_1.append([t,r])
					self.new_tag_db[1] = tag.fiducial_id
				elif not self.new_tag_buffer_2 or self.new_tag_db[2] == tag.fiducial_id:
					rospy.logdebug("LOGGING %s IN new_tag_buffer_2" % tag.fiducial_id)
					self.new_tag_buffer_2.append([t,r])
					self.new_tag_db[2] = tag.fiducial_id
				elif not self.new_tag_buffer_3 or self.new_tag_db[3] == tag.fiducial_id:
					rospy.logdebug("LOGGING %s IN new_tag_buffer_3" % tag.fiducial_id)
					self.new_tag_buffer_3.append([t,r])
					self.new_tag_db[3] = tag.fiducial_id
				else:
					rospy.logwarn("All temporary arrays in use.")

				if len(self.new_tag_buffer_1) == new_tag_threshold:
					rospy.logdebug("AVERAGING new_tag_buffer_1")
					avg = np.average(self.new_tag_buffer_1, axis=0)
					tag_position = transformify(avg[0],avg[1])
					self.new_tag_buffer_1 = []
					self.new_tag_db[1] = None
					self.tag_log_service(tag.fiducial_id, tag_position)
					rospy.logdebug("new_tag_buffer_1 DUMPED")
				elif len(self.new_tag_buffer_2) == new_tag_threshold:
					rospy.logdebug("AVERAGING new_tag_buffer_2")
					avg = np.average(self.new_tag_buffer_2, axis=0)
					tag_position = transformify(avg[0],avg[1])
					self.new_tag_buffer_2 = []
					self.new_tag_db[2] = None
					self.tag_log_service(tag.fiducial_id, tag_position)
					rospy.logdebug("new_tag_buffer_2 DUMPED")
				elif len(self.new_tag_buffer_3) == new_tag_threshold:
					rospy.logdebug("AVERAGING new_tag_buffer_3")
					avg = np.average(self.new_tag_buffer_3, axis=0)
					tag_position = transformify(avg[0],avg[1])
					self.new_tag_buffer_3 = []
					self.new_tag_db[3] = None
					self.tag_log_service(tag.fiducial_id, tag_position)
					rospy.logdebug("new_tag_buffer_3 DUMPED")

		# DEBUG: Broadcast position & known tags
		try:
			t,r = arrayify(self.position)
			self.br.sendTransform(t,r,rospy.Time.now(), "robot", "/world")
		except:
			rospy.logwarn("Position not yet localized")

		for tag in absolute_tag_transforms: #for tag in absolute_tag_transforms
			t,r = arrayify(absolute_tag_transforms[tag])
			self.br.sendTransform(t,r,rospy.Time.now(),"/tags/%s" % tag, "/world")

		# Flush buffer
		self.tag_known_buffer = []
		self.tag_unknown_buffer = []

	def shutdown(self, sig, frame):
		""" Runs on shutdown """
		if self.setup_mode:
			rospy.wait_for_service("tag_save")
			self.tag_save_service()
			sys.exit(0)


	def updateBuffer1(self, msg):
		if msg.transforms:
			self.buffer1 = msg.transforms

	def updateBuffer2(self, msg):
		if msg.transforms:
			self.buffer2 = msg.transforms

	def mergeBuffers(self):

		buffer1 = self.buffer1
		buffer2 = self.buffer2

		if buffer1 == self.buffer1_old and buffer2 == self.buffer2_old:
			return
		elif buffer1 == self.buffer1_old:
			self.buffer2_old = buffer2
			return buffer2
		elif buffer2 == self.buffer2_old:
			self.buffer1_old = buffer1
			return buffer1


		self.buffer1_old = buffer1
		self.buffer2_old = buffer2
		return (buffer1 + buffer2)

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
	r = r/(np.sqrt(r[0]**2+r[1]**2+r[2]**2+r[3]**2))
	rotation= Quaternion(r[0],r[1],r[2],r[3])
	transform = Transform(translation, rotation)

	return transform

################################################################################

if __name__ == "__main__":
	try:
		if sys.argv[1].lower() == "true":
			setup_mode = True
		elif sys.argv[1].lower() == "false":
			setup_mode = False
		else:
			rospy.logerror("Please enter True or False for setup mode")
			exit()
	except:
		setup_mode = False

	slammer = Slam(setup_mode)
