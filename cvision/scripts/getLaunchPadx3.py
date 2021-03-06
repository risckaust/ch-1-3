#!/usr/bin/env python

#####
# Search for launchpad using
# 1) Bright white thresholding
# 2) Circle detection on grayscale
# 3) Corner detection
# Publish (x,y) of detected center of a 640x480 screen
#####

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

###################################

# ROS parameters

rospy.set_param('/pix2m/LX', 640.0)
rospy.set_param('/pix2m/LY', 480.0)
rospy.set_param('/pix2m/altCal',1.2)
rospy.set_param('/pix2m/m2pix', 0.00104167) # 0.5m = 480pixels

# Flags & Constants
FEMASKON = True     # Use fisheye mask
THRESH = 10000.0    # threshold for positive centroid detection
TOL = 1.5           # radius multiplier for circle inclusion
ERODE = False       # Use erode/dilate vs blur
LIBERAL = True      # Allow lone bright white detection
HOVERLOW = False    # Allow corner only detection override (for temporary testing)
RED = 2             # Image size reduction
DIMX = 640/RED      # Reduced x-dimension
DIMY = 480/RED      # Reduced y-dimension
PXRAD = DIMY/4      # Radius for PXmask
LOOP_RATE = 15      # publishing rate (Hz)

# Image showing/saving/streaming
IMGSHOW = True      # Show images to screen
IMGPUB = False      # Publish raw images
IMGSTREAM = True    # Stream reduced images
PUB_RATE = 3        # save rate (Hz)
STREAM_RATE = 2     # streaming rate (Hz)

# Creat fisheye mask
FEmask = np.zeros((DIMY,DIMX,1), np.uint8)
cv2.circle(FEmask,(DIMX/2,DIMY/2),DIMX/2,(255,255,255),-1)

# Create erosion/dilation kernels
kernelE = np.ones((3,3),np.uint8)
kernelD = np.ones((3,3),np.uint8)

# Create publishers
targetPixel = rospy.Publisher('target_xyPixel', Point32, queue_size=10)
targetSp = rospy.Publisher('target_xySp', Point32, queue_size=10)

img_pub	 = 	rospy.Publisher('image_feed', Image, queue_size=10)
raw_img_pub = 	rospy.Publisher('raw_img', Image, queue_size=10)
msgPixel = Point32()
msgSp = Point32()
bridge = CvBridge()

spGen = cvisionLib.pix2m() # setpoint generator

def getLaunchPadCircles():

    # initialize node & set rate in Hz

    rospy.init_node('tracker', anonymous=True)
    rate = rospy.Rate(LOOP_RATE)

    # Initializations

    cxHold = -1.0
    cyHold = -1.0
    Detect = False
    DetectHold = False
    PXMASKON = False
    kc = 0              # number of iterations
    img_k = 1000		# counter of saved images

    # start video stream
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():

        # grab and resize frame
        _, frame = cap.read()
        frame = imutils.resize(frame, width=DIMX)
        raw_frame=frame.copy()

        # convert to grayscale
        frameGRY = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # apply fisheye mask
        if FEMASKON:
            frameGRY = cv2.bitwise_and(frameGRY,FEmask)
        
        # apply proximity mask
        if PXMASKON:
            frameGRY = cv2.bitwise_and(frameGRY,PXmask)

        # extract superwhite
        _, mask255h = cv2.threshold(frameGRY,225,255,cv2.THRESH_BINARY)

        # filter superwhite using either erode/dilate or blur
        if ERODE:
            mask255h = cv2.erode(mask255h,kernelE,iterations = 1)
            mask255h = cv2.dilate(mask255h,kernelD,iterations = 1)
        else:
            mask255h = cv2.blur(mask255h, (3,3))
            _, mask255h =  cv2.threshold(mask255h,245,255,cv2.THRESH_BINARY)

        # extract circles from grayscale

        if OLDCV:
            circlesGRY = cv2.HoughCircles(frameGRY,cv.CV_HOUGH_GRADIENT,1,DIMY,
                param1=50,param2=80,minRadius=DIMY/50,maxRadius=DIMY/4)
        else:
            circlesGRY = cv2.HoughCircles(frameGRY,cv2.HOUGH_GRADIENT,1,DIMY,
                param1=50,param2=80,minRadius=DIMY/50,maxRadius=DIMY/4)

        # assess circles 

        if circlesGRY is not None:
            temp = circlesGRY[0,0]
            cxGRY = temp[0]
            cyGRY = temp[1]
            crGRY = temp[2]
            detect_GRY = True
            cv2.circle(frame,(cxGRY,cyGRY),crGRY,(0,0,255),5)
        else:
            detect_GRY = False

        # Compute superwhite centroids
        M255h = cv2.moments(mask255h)

        if M255h['m00'] > THRESH:
            cx255h = int(M255h['m10']/M255h['m00'])
            cy255h = int(M255h['m01']/M255h['m00'])
            cv2.circle(frame,(cx255h,cy255h),10,(0,255,0),-1)
            detect_255h = True
        else:
            detect_255h = False
            cv2.circle(frame,(DIMX/2,DIMY/2),10,(0,0,0),-1)


        # compute corners from grayscale
        corners = cv2.goodFeaturesToTrack(frameGRY,10,0.5,20)
        if corners is not None:
            corners = np.int0(corners)
            for i in corners:
                x,y = i.ravel()
                cv2.circle(frame,(x,y),5,(0,255,255),-1)

            temp = cv2.mean(corners)
            temp = np.int0(temp)
            cv2.circle(frame,(temp[0],temp[1]),10,(0,255,255),-1)

            cxCRN = temp[0]
            cyCRN = temp[1]

            detect_CRN = True
        else:
            detect_CRN = False


        # detection acceptance logic
        Detect = False
        Skip = False
        PXMASKON = False
        CX = -1
        CY = -1

        if detect_GRY and detect_255h: # Greyscale circle + Superwhite centroid
            error = (cxGRY - cx255h)**2 + (cyGRY - cy255h)**2
            if sqrt(error) < TOL*crGRY:
                Detect = True
                CX = cxGRY
                CY = cyGRY
                Skip = True
        
        if detect_GRY and detect_CRN and not Skip: # Greyscale circle + Corners
            error = (cxGRY - cxCRN)**2 + (cyGRY - cyCRN)**2
            if sqrt(error) < TOL*crGRY:
                Detect = True
                CX = cxGRY
                CY = cyGRY
                Skip = True                

        if detect_255h and detect_CRN and not Skip: # Superwhite centroid + Corners
            error = (cx255h - cxCRN)**2 + (cy255h - cyCRN)**2
            if sqrt(error) < PXRAD/2:
                Detect = True
                CX = cx255h
                CY = cy255h
                Skip = True

        if LIBERAL:
            if detect_255h and not detect_GRY and not detect_CRN and not Skip:
                Detect = True
                CX = cx255h
                CY = cy255h

        if HOVERLOW:
            if detect_CRN:
                Detect = True
                CX = cxCRN
                CY = cyCRN

        # Create proximity mask for next image
        if Detect and DetectHold: # create a proximity mask of PXRAD radius circle
                PXmask = np.zeros((DIMY,DIMX,1), np.uint8)
                cv2.circle(PXmask,(CX,CY),PXRAD,(255,255,255),-1)
                PXMASKON = True

        # save for next iteration
        cxHold = CX
        cyHold = CY
        DetectHold = Detect

        # publish location with reduction correction
        msgPixel.x = CX*RED
        msgPixel.y = CY*RED
        msgPixel.z = 0.0 # Not used
        (msgSp.x, msgSp.y, msgSp.z) = spGen.targetFishEye(msgPixel)

        rate.sleep()
        targetPixel.publish(msgPixel)
        targetSp.publish(msgSp)

        # show/save/stream images
        if IMGSHOW:
            cv2.imshow('color',frame)
            cv2.imshow('gray',frameGRY)
            cv2.imshow('high',mask255h)
            key = cv2.waitKey(1) & 0xFF

        if IMGPUB: # publish raw image
            if (kc*SAVE_RATE)%LOOP_RATE < SAVE_RATE:
                raw_img_pub.publish(bridge.cv2_to_imgmsg(raw_frame, encoding="bgr8"))
                img_k = img_k+1

        if IMGSTREAM: # stream processed image
            if (kc*STREAM_RATE)%LOOP_RATE < STREAM_RATE:
                gray_frame=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray_frame=imutils.resize(gray_frame, width=200)
                img_pub.publish(bridge.cv2_to_imgmsg(gray_frame, encoding="passthrough"))

        kc = kc + 1

if __name__ == '__main__':
    try:
        getLaunchPadCircles()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
