#!/usr/bin/env python

import tf
import atexit
import math
import yaml
import sys
import numpy as np
import rospy as ros
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from fiducial_msgs.msg import *
from wfov_camera_msgs.msg import WFOVImage
from tf import transformations as t

class Slam:

	def __init__(self, setup_mode=True, rate=90):
		""" Initialize slam node """

		ros.init_node("slam", anonymous=False, log_level=ros.DEBUG)
		#####################TEMP: VICON ERROR ANALYSIS#########################
		self.sub_2 = ros.Subscriber("/vicon/omni3/omni3", TransformStamped, self.cb_test)
		self.sub = ros.Subscriber("/fiducial_transforms", FiducialTransformArray, self.cb)
		self.rate = ros.Rate(rate) # in hz
		self.br = tf.TransformBroadcaster()
		self.ls = tf.TransformListener()

		self.id_db = dict()
		if setup_mode:
			ros.logdebug("SETUP MODE")
		else:
			ros.logdebug("NAVIGATION MODE")
			self.loadTagConfiguration()
			ros.logdebug(self.id_db)

		self.position = Transform()
		self.tracking = False

	def cb(self, msg):
		""" Callback loop """

		if (not self.id_db) and msg.transforms:
			self.startDatabase(msg.transforms[0].fiducial_id)

		for i in msg.transforms:
			t, r = self.arrayify(i.transform)
			if i.fiducial_id in self.id_db: #UPDATE POSITION
				self.tracking = True
				t1,r1 = self.inverseTransform(t,r)
				t1 = Vector3(t1[0],t1[1],t1[2])
				r1 = Quaternion(r1[0],r1[1],r1[2],r1[3])
				f = Transform(t1,r1)
				self.position = self.addTransforms(self.id_db[i.fiducial_id], f)
				t1, r1 = self.arrayify(self.position)
				self.br.sendTransform(t1,r1,ros.Time.now(), "/camera", "/world")
			else: #UPDATE DATABASE
				if self.tracking:
					self.id_db[i.fiducial_id] = self.addTransforms(self.position,i.transform)

		for key in self.id_db:
			t,r = self.arrayify(self.id_db[key])
			self.br.sendTransform(t,r,ros.Time.now(), "/tags/" + str(key), "/world")

		ros.logdebug("TRACKING: " + str(self.tracking))
		self.tracking = False

	def cb_test(self, msg):
		print(msg)

	def startDatabase(self, id):
		""" Populate database with initial tag """

		init = Transform()
		init.rotation.w = 1

		self.id_db[id] = init

	def addTransforms(self, frame1, frame2):
		""" Adds two transforms to get a resulting transform """

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

	def q_mult(self, q1, q2):
		""" Borrowed from
		https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion """

		w1, x1, y1, z1 = q1
		w2, x2, y2, z2 = q2

		w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
		x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
		y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
		z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

		return w, x, y, z

	def inverseTransform(self, trans, rot):

		#transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
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

		ros.logdebug("Saving tag configuration..")
		#TEMP: Temporary file location, link dynamically
		with open('/home/nuc/catkin_ws/src/navigation_stack/config/id_db.yml', 'w') as yaml_file:
			yaml.dump(self.id_db, yaml_file)

	def loadTagConfiguration(self):
		""" Load tag configuration from yaml file """

		ros.logdebug("Loading tag configuration")
		with open('/home/nuc/catkin_ws/src/navigation_stack/config/id_db.yml', 'r') as yaml_file:
			self.id_db = yaml.load(yaml_file)

if __name__=="__main__":

	try:
		if sys.argv[1].lower() == "true":
			setup_mode = True
		elif sys.argv[1].lower() == "false":
			setup_mode = False
		else:
			print "Please enter True or False for the first argument (setup_mode)"
			exit()
	except:
		setup_mode = False

	slam = Slam(setup_mode)

	if setup_mode == True:
		atexit.register(slam.saveTagConfiguration)

	ros.spin()
