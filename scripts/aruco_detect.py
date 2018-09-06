#!/usr/bin/env python

import numpy as np
import rospy as ros
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco

class ArucoDetector(object) {

    def __init__(self):
        ''' Initialize the detector '''

        # Initialize ROS node
        ros.init_node('aruco_detector', anonymous=False)

        # Initialize ROS Subscriber
        try:
            image_topic = ros.get_param('image_topic')
        except:
            ros.logerror("Image topic not set in launch file!")
            exit()
        ros.logdebug("Image topic loaded..")
        self.sub = rospy.subscriber(image_topic, Image, self.cbDetect)

        # Configure detector
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

    def extractImage(self, msg):
        ''' Extracts the data from the msg and converts to usable numpy array '''
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, 1)
        ros.loginfo("")
        return image_np

    def detectMarkers(self, frame):
        ''' Detects markers based on default params. '''
        ## TODO: Look into how the parameters are determined
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, self.aruco_dict, parameters = self.parameters)
        return corners, ids

    def drawMarkers(self, corners, frame):
        ''' Draws the detected markers on the frame. '''
        debugFrame = aruco.drawDetectedMarkers(gray, corners)
        cv2.imshow('DEBUG', debugFrame)

    def cbDetect(self, msg):
        ''' The main callback loop that's run whenever a frame is received. '''
        frame = self.extractImage(msg)

        corners, ids = self.detectMarkers(frame)

        ## DEBUG:
        self.drawMarkers(corners, frame)


}

if __name__ == '__main__':
    ad = ArucoDetector()
    rospy.spin()
