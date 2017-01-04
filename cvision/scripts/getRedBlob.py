#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import imutils

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
cvisionParams.setParams()

###################################

# Flags & Constants

LOOP_RATE = rospy.get_param('/cvision/loopRate')
LX = rospy.get_param('/cvision/LX')
LY = rospy.get_param('/cvision/LY')
CAMROTATE = rospy.get_param('/cvision/camRotate')
FECAMERA = rospy.get_param('/cvision/feCamera')

IMGSHOW = rospy.get_param('/getLaunchPad/imgShow')
IMGSTREAM = rospy.get_param('/getLaunchPad/imgStream')
STREAM_RATE = rospy.get_param('/getLaunchPad/imgStreamRate')

# Create publishers
targetPixel = rospy.Publisher('red_xyPixel', Point32, queue_size=10)
targetSp = rospy.Publisher('red_xySp', Point32, queue_size=10)
img_pub	 = 	rospy.Publisher('processed_Image', Image, queue_size=10)

msgPixel = Point32()
msgSp = Point32()
bridge = CvBridge()

spGen = cvisionLib.pix2m() # setpoint generator

def getLaunchPad():

    # initialize node & set rate in Hz

    rospy.init_node('tracker', anonymous=True)
    rate = rospy.Rate(LOOP_RATE)

    # Initializations

    cxHold = -1.0
    cyHold = -1.0
    Detect = False
    DetectHold = False
    PXMASKON = False
    kc = 0              # iteration counter for downsample image streaming
    morph_width=2       # for erode & dilate
    morph_height=2

    # start video stream: replaces cap = cv2.VideoCapture(0)
    quadCam = cvisionLib.getFrame()

    while not rospy.is_shutdown():

        # grab and resize frames
        frame = quadCam.BGR

        # grab a frame
        frame = quadCam.BGR

        # convert to HSV        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    	# find the red in the image with low "H"
        lower = np.array([0,121,180],np.uint8)
        upper = np.array([10,255,255],np.uint8)
        maskLow = cv2.inRange(hsv, lower, upper)
        
        # find the red in the image with high "H"
        lower = np.array([170,121,180],np.uint8)
        upper = np.array([180,255,255],np.uint8)
        maskHigh = cv2.inRange(hsv, lower, upper)
        
        # merge masks
        mask = cv2.bitwise_or(maskLow,maskHigh)

        # opening
        mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)

    	# closing
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
        mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
        
        # find contours in the masked image
        if OLDCV:
            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        else:
            _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
        # prep messages
        msgPixel.x = -1.0
        msgPixel.y = -1.0
        msgPixel.z = -1.0
        
        center= None
        radius = 0
        if len(cnts) > 0:
            # keep largest contour
            c = max(cnts, key=cv2.contourArea)
            # construct & draw bounding circle
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            msgPixel.z = 1.0
	    M = cv2.moments(c)
	    if M["m00"]>0:
	    	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    else:
	    	center=(int(M["m10"]), int(M["m01"]))

	    # only proceed if the radius meets a minimum size
        if radius > 3:
	    # draw the circle and centroid on the frame,
		# then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

        if center is None:
		    center=(-1.0,-1.0)
		
        msgPixel.x=center[0]
        msgPixel.y=center[1]

        if CAMROTATE and msgPixel.z > 0:                        # rotate camera if needed
            msgPixel.x, msgPixel.y = cvisionLib.camRotate(msgPixel.x, msgPixel.y)

        if FECAMERA:                                            # convert pixels to to meters
            (msgSp.x, msgSp.y, msgSp.z) = spGen.targetFishEye(msgPixel)
        else:
            (msgSp.x, msgSp.y, msgSp.z) = spGen.target(msgPixel)

        rate.sleep()
        targetPixel.publish(msgPixel)
        targetSp.publish(msgSp)

        # show/stream images
        if IMGSHOW:
            cv2.imshow('color',frame)
            key = cv2.waitKey(1) & 0xFF

        if IMGSTREAM: # stream processed image
            if (kc*STREAM_RATE)%LOOP_RATE < STREAM_RATE:
                gray_frame=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray_frame=imutils.resize(gray_frame, width=200)
                img_pub.publish(bridge.cv2_to_imgmsg(gray_frame, encoding="passthrough"))

        kc = kc + 1

if __name__ == '__main__':
    try:
        getLaunchPad()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
