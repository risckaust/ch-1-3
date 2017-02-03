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

def getCircle():

    # Initialize node

    rospy.init_node('circleTracker', anonymous=True)
    
    # Create publishers
    targetPixels = rospy.Publisher('/getLaunchpad/circle/xyPixels', Point32, queue_size=10)
    msgPixels = Point32()
    img_pub	 = 	   rospy.Publisher('/getLaunchpad/circle/processedImage', Image, queue_size=10)
    bridge = CvBridge()
    
    # Subscribe to launchpad for proximity masks
    getLaunchpad = cvisionLib.xyzVar()
    rospy.Subscriber('/getLaunchpad/launchpad/xyPixels', Point32, getLaunchpad.cbXYZ)

    # establish publish rate
    
    rate = rospy.Rate(rospy.get_param('/cvision/loopRate'))

    # initializations
    
    kc = 0              # iteration counter for downsample image streaming
    Detect = False      # for proximity mask
    DetectHold = False
    MaskItNow = False
    
    # Create fisheye mask
    LX = rospy.get_param('/cvision/LX')
    LY = rospy.get_param('/cvision/LY')
    feMask = np.zeros((LY,LX,1), np.uint8)
    cv2.circle(feMask,(LX/2,LY/2),LX/2,(255,255,255),-1)

    # start video stream: Replaces
    #   cap = cv2.VideoCapture(0) or cap = cv2.VideoCapture('file.mp4')
    #   _, frame = cap.read()
    quadCam = cvisionLib.getFrame()
    
    # Code for testing from video file
    if rospy.get_param('/getLaunchpad/testFileOn'):
        cap = cv2.VideoCapture(rospy.get_param('/getLaunchpad/fileName'))

    while not rospy.is_shutdown():

        # grab a frame
        if rospy.get_param('/getLaunchpad/testFileOn'):
            _, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.resize(frame,(LX,LY))
            mask = frame
        else:
            frame = quadCam.Gry
            mask = frame
            
        # blur image
        mask = cv2.blur(mask, (5,5))

        # apply fisheye mask
        if rospy.get_param('/cvision/feCamera'):
            mask = cv2.bitwise_and(mask,feMask)

        # apply proximity mask
        if MaskItNow:
            mask = cv2.bitwise_and(mask,pxMask)
    
        # prep messages
        msgPixels.x = -1.0
        msgPixels.y = -1.0
        msgPixels.z = -1.0
        Detect = False
        
        # extract circles from grayscale

        if OLDCV:
            circles = cv2.HoughCircles(mask,cv.CV_HOUGH_GRADIENT,1,LY,
                param1=50,param2=100,minRadius=LY/50,maxRadius=LY/2)
        else:
            circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,1,LY,
                param1=50,param2=100,minRadius=LY/50,maxRadius=LY/2)

        # assess circles 

        if circles is not None:
            center = circles[0,0]
            Detect = True
            cv2.circle(frame,(center[0],center[1]),center[2],(0,0,0),5)
            cv2.circle(frame, (int(center[0]), int(center[1])), 3, (0, 0, 0), -1)
            msgPixels.x = center[0] # x
            msgPixels.y = center[1] # y
            msgPixels.z = center[2] # radius
        else:
            Detect = False

        # create proximity mask
        # NOTE: Proximity masking is mandatory
        
        pxMask = np.zeros((LY,LX,1), np.uint8)
        if Detect and DetectHold and getLaunchpad.z > 0: # create a proximity mask of radius multiple
            cv2.circle(pxMask,(int(getLaunchpad.x),int(getLaunchpad.y)),int(center[2]*rospy.get_param('/getLaunchpad/pxRadius')),(255,255,255),-1)
            # Deleted code for circle-based proximity
            # cv2.circle(pxMask,(center[0],center[1]),np.uint8(center[2]*rospy.get_param('/getLaunchpad/pxRadius')),(255,255,255),-1)
            MaskItNow = True
        else:
            MaskItNow = False
                
        DetectHold = Detect # hold for next iteration

        # publish target pixels only
        rate.sleep()
        targetPixels.publish(msgPixels)

        # show processed images to screen
        if rospy.get_param('/getLaunchpad/imgShow'):
            cv2.imshow('circle',frame)
            # cv2.imshow('pxMask',pxMask)
            key = cv2.waitKey(1) & 0xFF

        # published downsized/grayscale processed image
        STREAM_RATE = rospy.get_param('/getLaunchpad/imgStreamRate')
        if rospy.get_param('/getLaunchpad/imgStream'): # stream processed image
            if (kc*STREAM_RATE)%rospy.get_param('/cvision/loopRate') < STREAM_RATE:
                frame=imutils.resize(frame, width=LX/2)
                img_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="passthrough"))

        kc = kc + 1

if __name__ == '__main__':
    try:
        getCircle()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
