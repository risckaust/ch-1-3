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
cvisionParams.setParams()

###################################

# Create publishers

frameBGR  = rospy.Publisher('frameBGR', Image, queue_size=10)
frameGry =  rospy.Publisher('frameGry', Image, queue_size=10)

msgBGR = CvBridge()
msgGry = CvBridge()

def videoBridge():

    # initialize node & set rate in Hz
    rospy.init_node('videoBridge', anonymous=True)
    rate = rospy.Rate(rospy.get_param('/cvision/loopRate'))

    # start video stream
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():

        # grab a frame
        _, bgr = cap.read()
        
        # convert to grayscale
        
        gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # publish images
        rate.sleep()
        frameBGR.publish(msgBGR.cv2_to_imgmsg(bgr, encoding="passthrough"))
        frameGry.publish(msgGry.cv2_to_imgmsg(gry, encoding="passthrough"))

if __name__ == '__main__':
    try:
        videoBridge()
    except rospy.ROSInterruptException:
        # cleanup the camera and close any open windows
        cap.release()
        cv2.destroyAllWindows()
        pass


