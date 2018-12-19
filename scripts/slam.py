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
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()

        self.id_db = dict()

        self.position = Transform()

    def cb(self, msg):
        ''' Callback loop '''

        if (not self.id_db) and msg.transforms:
            self.startDatabase(msg.transforms[0].fiducial_id)

        for i in msg.transforms:
            t, r = self.arrayify(i.transform)
            self.br.sendTransform(
                t, r, ros.Time.now(), "/" + str(i.fiducial_id), "/world")
            test = self.addTransforms(self.id_db[i.fiducial_id], i.transform)

    def startDatabase(self, id):
        ''' Populate database with initial tag '''

        init = Transform()
        init.rotation.w = 1

        self.id_db[id] = init

    def addTransforms(self, frame1, frame2):
        ''' Adds two transforms to get a resulting transform '''

        trans = Vector3(frame1.translation.x + frame2.translation.x,
                            frame1.translation.y + frame2.translation.y,
                            frame1.translation.z + frame2.translation.z)

        rot1 = np.array([frame1.rotation.x, frame1.rotation.y, frame1.rotation.z, frame1.rotation.w])
        rot2 = np.array([frame2.rotation.x, frame2.rotation.y, frame2.rotation.z, frame2.rotation.w])
        rot = self.q_mult(rot1, rot2)
        rot = Quaternion(rot[0],rot[1],rot[2],rot[3])

        return Transform(trans, rot)

    def q_mult(self, q1, q2):
        ''' Borrowed from
        https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion '''

        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        return w, x, y, z

    def q_inv(self, q):
        ''' Adapted from
        https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion '''

        w1, x1, y1, z1 = q

        r = w1^2 + x1^2 + y1^2 + z1^2
        w = w1/r
        x = x1/r
        y = y1/r
        z = z1/r

        return w, x, y, z

    def arrayify(self, n):
        ''' Turn the transform into two nparrays, translation and rotation '''

        trans = np.array([n.translation.x, n.translation.y, n.translation.z])
        rot = np.array([n.rotation.x, n.rotation.y, n.rotation.z, n.rotation.w])

        return trans, rot

if __name__=="__main__":

    slam = Slam()

    ros.spin()
