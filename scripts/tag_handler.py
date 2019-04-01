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

class tag_handler:

	def __init__(self):
		print "under construction"

		self.id_db[] = {}

	def
