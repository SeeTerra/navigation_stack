#!/usr/bin/env python

import tf
import atexit
import math
import yaml
import sys
import rospy
import numpy as np
import pyavltree
import os
from fiducial_msgs.msg import *
from geometry_msgs.msg import *
from navigation_stack.srv import *
from tf import transformations as t

class tag_handler:

	def __init__(self):

		##ROS
		rospy.init_node("tag_database", anonymous=True, log_level=rospy.DEBUG)
		self.service_log = rospy.Service('tag_log', TagLog, self.tag_log)
		self.service_lookup = rospy.Service('tag_lookup', TagLookup, self.tag_lookup)

		##Tag
		self.tag_tree = pyavltree.AVLTree()

	def database_load(self):
		"""Load database from file"""
		rospy.logdebug("Loading tag configuration")
		#TEMP: Temporary file location, link dynamically
		with open('id_db.yml', 'r') as yaml_file:
			self.tag_tree = yaml.load(yaml_file)

	def database_save(self):
		"""Saves database to file"""
		rospy.loginfo("Saving tag configuration..")
		#TEMP: Temporary file location, link dynamically
		with open('id_db.yml', 'w') as yaml_file:
			yaml.dump(self.tag_tree, yaml_file)

	def tag_log(self, msg):
		"""Store tag info in database"""
		self.tag_tree.insert(msg.id, msg.transform)
		return True

	def tag_lookup(self, msg):
		"""Retrieve tag info in database, returns id of -1 if false"""
		try:
			transform = self.tag_tree.find(msg.id).data
			return TagLookupResponse(msg.id, transform)
		except:
			return TagLookupResponse(-1, Transform())

if __name__ == "__main__":
	test = tag_handler()
	setup_mode = True
	if setup_mode == True:
		atexit.register(test.database_save)
	rospy.spin()
