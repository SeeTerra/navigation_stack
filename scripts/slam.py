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
from wfov_camera_msgs.msg import WFOVImage
from tf import transformations as t

class Slam:

	def __init__(self, setup_mode=True, rate=90):
		""" Initialize slam node """

		rospy.init_node("slam", anonymous=False, log_level=rospy.DEBUG)
		#####################TEMP: VICON ERROR ANALYSIS#########################
		#self.sub_2 = rospy.Subscriber("/vicon/muskrat_24/muskrat_24", TransformStamped, self.cb_test)
		########################################################################
		self.sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.cb)
		self.rate = rospy.Rate(rate) # in hz
		self.br = tf.TransformBroadcaster()
		self.ls = tf.TransformListener()

		self.id_db = dict()
		if setup_mode:
			rospy.logdebug("SETUP MODE")
		else:
			rospy.logdebug("NAVIGATION MODE")
			self.loadTagConfiguration()
			rospy.logdebug(self.id_db)

		self.position = Transform()
		self.tracking = False

	def cb(self, msg):
		""" Callback loop """

		# If we have a database, check tags against it
		if self.id_db:
			self.errorCheck(msg)

		#    If id_db exists   if we have a message
		if (not self.id_db) and msg.transforms:
			self.startDatabase(msg.transforms[0].fiducial_id)

		for i in msg.transforms:
			t, r = self.arrayify(i.transform)
			if i.fiducial_id in self.id_db: # ..THEN UPDATE POSITION
				self.tracking = True
				t1,r1 = self.inverseTransform(t,r)
				t1 = Vector3(t1[0],t1[1],t1[2])
				r1 = Quaternion(r1[0],r1[1],r1[2],r1[3])
				f = Transform(t1,r1) # Transform must be in Vector3 and Quaternion
				self.position = self.addTransforms(self.id_db[i.fiducial_id], f)
				t1, r1 = self.arrayify(self.position)
				self.br.sendTransform(t1,r1,rospy.Time.now(), "/camera", "/world")
			else: # UPDATE DATABASE
				if self.tracking:
					self.id_db[i.fiducial_id] = self.addTransforms(self.position,i.transform)

		for key in self.id_db:
			t,r = self.arrayify(self.id_db[key])
			self.br.sendTransform(t,r,rospy.Time.now(), "/tags/" + str(key), "/world")

		#rospy.logdebug("TRACKING: " + str(self.tracking))
		self.tracking = False

	def cb_test(self, msg):
		""" Temporary function for VICON position error analysis """

		vicon_position = np.array([msg.transform.translation.x,
								msg.transform.translation.y,
								msg.transform.translation.z])

		print(vicon_position)

	def startDatabase(self, id):
		""" Populate database with initial tag """

		init = Transform()
		init.rotation.w = 1

		self.id_db[id] = init

	def addTransforms(self, frame1, frame2):
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

	def inverseTransform(self, trans, rot):

		transform = t.compose_matrix(translate=trans,angles=t.euler_from_quaternion(rot))
		inversed_transform = t.inverse_matrix(transform)

		tran = t.translation_from_matrix(inversed_transform)
		quat = t.quaternion_from_matrix(inversed_transform)

		return tran, quat

	def arrayify(self, n):
		""" Turn the transform into two nparrays, translation and rotation """

		trans = np.array([n.translation.x, n.translation.y, n.translation.z])
		rot = np.array([ n.rotation.x, n.rotation.y, n.rotation.z, n.rotation.w])

		return trans, rot

	def saveTagConfiguration(self):
		""" Save tag configuration to yaml file """

		rospy.loginfo("Saving tag configuration..")
		#TEMP: Temporary file location, link dynamically
		with open('/home/nuc/catkin_ws/src/navigation_stack/config/id_db.yml', 'w') as yaml_file:
			yaml.dump(self.id_db, yaml_file)

	def loadTagConfiguration(self):
		""" Load tag configuration from yaml file """

		rospy.logdebug("Loading tag configuration")
		with open('/home/nuc/catkin_ws/src/navigation_stack/config/id_db.yml', 'r') as yaml_file:
			self.id_db = yaml.load(yaml_file)

	def errorCheck(self, msg):
		""" Uses the camera's current position as well as any detected tags
		to check against the database and determine whether there is an error."""

		# Camera's world transform from tf
		try:
			t, r = self.ls.lookupTransform('/camera', '/world', rospy.Time())
			t = Vector3(t[0],t[1],t[2])
			r = Quaternion(r[0],r[1],r[2],r[3])
			camera_position = Transform(t,r)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			rospy.logwarn("Camera not found in lookup!")
			rospy.logwarn(str(e))
			return 0

		tags = []
		for tag in msg.transforms:
			if tag.fiducial_id in self.id_db:
				tags.append(tag)

		# Do error detection
		for tag in tags:
			tag_from_cam = self.addTransforms(camera_position,tag.transform)
			t, r = self.ls.lookupTransform('/tags/' + str(tag.fiducial_id), '/world', rospy.Time())
			t = Vector3(t[0],t[1],t[2])
			r = Quaternion(r[0],r[1],r[2],r[3])
			tag_from_tf = Transform(t,r) # Transform must be in Vector3 and Quaternion

			# Compare transforms
			tf_t, tf_r = self.arrayify(tag_from_tf)
			tf_t, tf_r = self.inverseTransform(tf_t,tf_r)
			tf_t = Vector3(tf_t[0],tf_t[1],tf_t[2])
			tf_r = Quaternion(tf_r[0],tf_r[1],tf_r[2],tf_r[3])
			tag_from_tf = Transform(tf_r,tf_r) # Transform must be in Vector3 and Quaternion
			transform_difference = self.addTransforms(tag_from_cam,tag_from_tf)
			rospy.logdebug(str(tag.fiducial_id) + "ERROR: \n" + str(transform_difference))

if __name__== "__main__":

	try:
		if sys.argv[1].lower() == "true":
			setup_mode = True
		elif sys.argv[1].lower() == "false":
			setup_mode = False
		else:
			rospy.logerror("Please enter True or False for the first argument (setup_mode)")
			exit()
	except:
		setup_mode = False

	slam = Slam(setup_mode)

	if setup_mode == True:
		atexit.register(slam.saveTagConfiguration)

	rospy.spin()
