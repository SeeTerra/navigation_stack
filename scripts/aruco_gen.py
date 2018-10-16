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
import sys
import random

def generateTag(dir, id):
    ''' The actual tag generator '''
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    img = aruco.drawMarker(aruco_dict, id, 700)

    write_dir = dir + "/../tags/" + str(id) + ".jpg"
    print("Writing to: " + write_dir)
    cv2.imwrite(write_dir, img)

    return img

def exactNumbers(dir,num):
    ''' Handles the logic for picking the exact numbers '''
    idArr = []
    print("Please type the id and hit enter after each. Repeated ids will be overwritten on generation.")
    for i in range(0,num):
        idArr.append(int(raw_input()))
    print("All " + str(num) + " ids logged, now generating..")
    for i in idArr:
        generateTag(dir,i)


def generateArgs(dir):
    ''' Main program, handles which type of generator to use '''
    print("Hello! No arguments were detected, please answer the following questions:")
    print("How would you like to generate tags?")
    print("1. By exact tag numbers")
    print("2. By a range of tags")
    print("3. By a number of randomly chosen tags")
    ok = False

    while (ok == False):
        selection = raw_input()
        if selection == '1':
            #TODO: create function
            print("How many tags would you like to generate?")
            num = int(raw_input())
            exactNumbers(dir,num)
            ok = True
        elif selection == '2':
            print("Please enter the starting number: ")
            start = raw_input()
            print("Ending number: ")
            end = raw_input()
            if(int(start)<int(end)):
                for i in range(int(start), int(end)+1):
                    generateTag(dir,i)
                ok = True
            else:
                print("Invalid numbers, make sure the end number is higher than the start!")
                ok = False
        elif selection == '3':
            print("Please enter the number of tags you need: ")
            num = int(raw_input())
            nums = random.sample(range(1,250),num)
            for i in nums:
                generateTag(dir,i)
            ok = True
        else:
            print("Sorry, try picking one of the options..")



if (__name__ == "__main__"):
    # Make sure we're in the right working directory!
    dir = os.path.dirname(os.path.realpath(__file__))

    # If the user hasn't given any arguments in the command line, let's ask them!
    if (len(sys.argv) == 1):
        generateArgs(dir)
