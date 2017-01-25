#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import imutils
import math
import time

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Check version of OpenCV

if cv2.__version__.startswith('2'):
    OLDCV = True
else:
    OLDCV = False
    
if OLDCV:
    import cv2.cv as cv

# Import vision parameters

import cvisionParams

###################################

def videoBridge():

    # initialize node
    rospy.init_node('videoBridge', anonymous=True)

    # get the namespace
    ns = rospy.get_namespace()
    ns = ns[0:len(ns)-1]

    # import vision params
    cvisionParams.setParams(ns)
    
    # Create publishers

    frameBGR  = rospy.Publisher(ns+'/cvision/frameBGR', Image, queue_size=10)
    msgBGR = CvBridge()
    frameGry =  rospy.Publisher(ns+'/cvision/frameGry', Image, queue_size=10)
    msgGry = CvBridge()

    # set publication rate
    rate = rospy.Rate(rospy.get_param(ns+'/cvision/loopRate'))

    # start video stream
    cap = cv2.VideoCapture(0)
    
    if cap.isOpened():
        print 'videoBridge initialized...'
    else:
        print 'videoBridge error...'
        
    while not rospy.is_shutdown():

        # grab a frame
        _, bgr = cap.read()
        
        # convert to grayscale
        
        gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # publish images
        rate.sleep()
        frameBGR.publish(msgBGR.cv2_to_imgmsg(bgr, encoding="bgr8"))
        frameGry.publish(msgGry.cv2_to_imgmsg(gry, encoding="passthrough"))

if __name__ == '__main__':
    try:
        videoBridge()
    except rospy.ROSInterruptException:
        # cleanup the camera and close any open windows
        cap.release()
        cv2.destroyAllWindows()
        pass


