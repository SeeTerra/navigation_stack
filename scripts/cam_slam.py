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

	def __init__(self, cam_number, setup_mode, base_transform):

		##ROS
		topic = "camera_%s/fiducial_transforms" % cam_number

		rospy.init_node("cam_slam_%s" % cam_number, anonymous=False, log_level=rospy.DEBUG)
		self.sub = rospy.Subscriber("camera_%s/fiducial_transforms" % cam_number, FiducialTransformArray, self.updateBuffer, queue_size=1)
		self.pub = rospy.Publisher("camera_%s/transform_array" % cam_number, FiducialTransformArray, queue_size=1)
		self.rate = rospy.Rate(20) # in hz

		##Localization var
		self.tag_array_tmp = None
		self.tag_array_old = None
		self.tag_array_time = None
		self.base_transform = base_transform

		##Loop
		while not rospy.is_shutdown():
			self.main()
			self.rate.sleep()

	def main(self):
		""" Main Loop """

		#Grab recent tags detected
		fiducial_array = self.tag_array_tmp
		if self.tag_array_old == None:
			self.tag_array_old = fiducial_array
			return

		if (fiducial_array is None) or (fiducial_array == self.tag_array_old):
			return
		else:
			for tag in fiducial_array:
				tag.transform = addTransforms(self.base_transform, tag.transform)

		self.tag_array_old = fiducial_array

		msg = FiducialTransformArray()
		msg.transforms = fiducial_array

		self.pub.publish(msg)


	def updateBuffer(self, msg):
		"""Update the camera's location in space based on the tags detected"""

		#TODO: Mutex Lock
		if msg.transforms:
			self.tag_array_tmp = msg.transforms
			self.tag_array_time = msg.header.stamp

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
	if int(sys.argv[1]) == 1:
		base_link_test = Transform()
		base_link_test.translation.x = -0.05715
		q = t.quaternion_from_euler(-np.pi/2,0,np.pi/4)
		base_link_test.rotation = Quaternion(q[0],q[1],q[2],q[3])

	if int(sys.argv[1]) == 2:
		base_link_test = Transform()
		base_link_test.translation.x = 0.05715
		q = t.quaternion_from_euler(-np.pi/2,0,-np.pi/4)
		base_link_test.rotation = Quaternion(q[0],q[1],q[2],q[3])

	cam_slam = CamSlam(sys.argv[1],True,base_link_test)
	rospy.spin()
