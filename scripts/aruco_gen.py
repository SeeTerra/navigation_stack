#!/usr/bin/env python

#TODO: Write the aruco_generator so that we can
# have multiple aruco tags generated at once, and
# the user can either choose which numbers to generate
# or if they'd rather have a number of tags generated
# at random.
#
#TODO: Make sure the files are written to a certain directory


import cv2
import cv2.aruco as aruco
import os

def generateTag(dir):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    img = aruco.drawMarker(aruco_dict, 2, 700)

    write_dir = dir + "/../tags/test_marker.jpg"
    print("Writing to: " + write_dir)
    cv2.imwrite(write_dir, img)

    cv2.imshow('frame', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return img


if (__name__ == "__main__"):
    dir = os.path.dirname(os.path.realpath(__file__))
    marker_gen = generateTag(dir)
