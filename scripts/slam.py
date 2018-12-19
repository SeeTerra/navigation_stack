#!/usr/bin/env python

import tf
import math
import numpy as np
import rospy as ros
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from fiducial_msgs.msg import *
from wfov_camera_msgs.msg import WFOVImage

class Slam:

    def __init__(self, rate=90):
        ''' Initialize slam node '''

        ros.init_node("slam", anonymous=False, log_level=ros.DEBUG)
        self.sub = ros.Subscriber("/fiducial_transforms", FiducialTransformArray, self.cb)
        self.rate = ros.Rate(rate) # in hz

        self.id_db = dict()

    def cb(self, msg):
        ''' Callback loop '''

        ros.logdebug(str(self.id_db) + " and " + str(msg.transforms))
        if (not self.id_db) and msg.transforms:
            ros.logdebug("HEY LETS INIT THE DB!")
            self.startDatabase(msg.transforms[0].fiducial_id)

        for i in msg.transforms:
            ros.logdebug(i.fiducial_id)
            if not self.id_db:
                self.id_db[i.fiducial_id] = Transform()

    def startDatabase(self, id):
        ''' Populate database with initial tag '''

        init = Transform([0,0,0],[0,0,0,0])
        trans = Vector3(0,0,0)
        rot = Quaternion(0,0,0,0)

        #trans.x = 0
        #trans.y = 0
        #trans.z = 0

        #rot.x = 0
        #rot.y = 0
        #rot.z = 0
        #rot.w = 0

        init.translation = trans
        init.rotation = rot

        ros.logdebug(id)

if __name__=="__main__":

    slam = Slam()

    ros.spin()
