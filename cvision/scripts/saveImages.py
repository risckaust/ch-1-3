#!/usr/bin/env python

# Combines blue/green/red

import rospy
import numpy as np
import cv2
import imutils
import time

from math import sqrt
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Check version of OpenCV

if cv2.__version__.startswith('2'):
    OLDCV = True
else:
    OLDCV = False
    
if OLDCV:
    import cv2.cv as cv

import cvisionLib
import cvisionParams


###################################

def main():

    # Initialize node

    rospy.init_node('ImageSaver', anonymous=True)
    
    
    # get namespace
    ns = rospy.get_namespace()
    ns = ns[0:len(ns)-1]

    # get folder path
    if not rospy.has_param(ns+'/ImagesFolderPath'):
     	print '##################3 Folder path is not loaded #####################'
    	return

    if not rospy.has_param(ns+'/ImageSaveRate'):
     	print '################### Save Rate is not loaded ###################3'
    	return

    path = rospy.get_param(ns+'/ImagesFolderPath')
    save_rate = rospy.get_param(ns+'/ImageSaveRate')

    cvisionParams.setParams(ns)

    img_pub	 = 	   rospy.Publisher(ns+'/RawIamge', Image, queue_size=10)
    bridge = CvBridge()
    
    # Create setpoint generator
    spGen = cvisionLib.pix2m(ns) # setpoint generator

    # establish publish rate
    rate = rospy.Rate(save_rate)

    # initialization

    # start video stream and set parameters
    cap = cv2.VideoCapture(0)
    if OLDCV:
        cap.set(cv.CV_CAP_PROP_FPS, rospy.get_param(ns+'/cvision/loopRate'))
        cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, rospy.get_param(ns+'/cvision/LX'))
        cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, rospy.get_param(ns+'/cvision/LY'))
    else:
        cap.set(cv2.CAP_PROP_FPS, rospy.get_param(ns+'/cvision/loopRate'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, rospy.get_param(ns+'/cvision/LX'))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, rospy.get_param(ns+'/cvision/LY'))
     
    # wait a second
    time.sleep(1.0)
    
    if cap.isOpened():
        print 'videoBridge initialized...'
    else:
        print 'videoBridge error...'
    #quadCam = cvisionLib.getCvFrame()
    
    c = 0
    while not rospy.is_shutdown():
        # grab a frame
        _, frame = cap.read()
	
	# save image
	filename = path+'/img'+ str(c) + '.jpg'
	cv2.imwrite(filename, frame)
	print 'Image' + str(c) + '.jpg' + ' is saved in ' + path
	c = c + 1

	img_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
	
	rate.sleep()
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        #cap.release()
        cv2.destroyAllWindows()
pass
