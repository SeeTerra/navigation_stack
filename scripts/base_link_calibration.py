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

class base_link_calibration:

	def __init__(self):

		##ROS
		rospy.init_node("base_link_calibration", anonymous=False)
		self.br = tf.TransformBroadcaster()
		self.ls = tf.TransformListener()

		self.camera_1 = Transform()
		self.camera_2 = Transform()

		self.camera_1.translation.x = -0.05715
		self.camera_2.translation.x = 0.05715


		self.camera_1_rotation = t.quaternion_from_euler(-np.pi/2,0,np.pi/4)
		self.camera_2_rotation = t.quaternion_from_euler(-np.pi/2,0,-np.pi/4)

		q = t.quaternion_from_euler(-np.pi/2,0,-np.pi/4)
		Quaternion(q[0],q[1],q[2],q[3])

		while not rospy.is_shutdown():
			self.main()

	def main(self):
		t1, r1 = arrayify(self.camera_1)
		t2, r2 = arrayify(self.camera_2)

		self.br.sendTransform(t1,self.camera_1_rotation,rospy.Time.now(), "/camera_1", "/world")
		self.br.sendTransform(t2,self.camera_2_rotation,rospy.Time.now(), "/camera_2", "/world")

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
	tester = base_link_calibration()
	rospy.spin()
