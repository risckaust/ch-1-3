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

def getCorners():

    # Initialize node

    rospy.init_node('cornerTracker', anonymous=True)
    
    # Create publishers
    targetPixels = rospy.Publisher('/getLaunchpad/corners/xyPixels', Point32, queue_size=10)
    msgPixels = Point32()
    img_pub	 = 	   rospy.Publisher('/getLaunchpad/corners/processedImage', Image, queue_size=10)
    bridge = CvBridge()
    
    # Create setpoint generator
    spGen = cvisionLib.pix2m() # setpoint generator

    # establish publish rate
    rate = rospy.Rate(rospy.get_param('/cvision/loopRate'))

    # initializations
    kc = 0              # iteration counter for downsample image streaming
    Detect = False      # for detection bookkeeping (not used)
    DetectHold = False
    
    # Create fisheye mask
    LX = rospy.get_param('/cvision/LX')
    LY = rospy.get_param('/cvision/LY')
    feMask = np.zeros((LY,LX,1), np.uint8)
    cv2.circle(feMask,(LX/2,LY/2),LX/2,(255,255,255),-1)
    
    # Parameters for Shi Tomasi corner detection
    feature_params = dict( maxCorners = 10,
                       qualityLevel = 0.5,
                       minDistance = 25,
                       blockSize = 7 ) 
                       
    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))  

    # start video stream: Replaces
    #   cap = cv2.VideoCapture(0) or cap = cv2.VideoCapture('file.mp4')
    #   _, frame = cap.read()
    quadCam = cvisionLib.getFrame()
    
    # Code for testing from video file
    if rospy.get_param('/getLaunchpad/testFileOn'):
        cap = cv2.VideoCapture(rospy.get_param('/getLaunchpad/fileName'))
        
    # Grab an initial frame & find features
    if rospy.get_param('/getLaunchpad/testFileOn'):
        _, frame = cap.read()
        frame = cv2.resize(frame,(LX,LY))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        old_frame = frame
    else:
        old_frame = quadCam.Gry
    
    if rospy.get_param('/cvision/feCamera'):
        p0 = cv2.goodFeaturesToTrack(old_frame, mask = feMask, **feature_params)
    else:
        p0 = cv2.goodFeaturesToTrack(old_frame, **feature_params)

    # Create an image for drawing purposes
    traces = np.zeros_like(old_frame)  

    while not rospy.is_shutdown():

        # grab a frame
        if rospy.get_param('/getLaunchpad/testFileOn'):
            _, frame = cap.read()
            frame = cv2.resize(frame,(LX,LY))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mask = frame
        else:
            frame = quadCam.Gry
            mask = frame
    
        # prep messages
        msgPixels.x = -1.0
        msgPixels.y = -1.0
        msgPixels.z = -1.0
        Detect = False
        
        # calculate optical flow
        if p0 is not None:
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_frame, mask, p0, None, **lk_params)
        else:
            p1 = None
            
        # Select good points
        Restart = True
        if (p1 is not None):
            if (p1.shape[0] > rospy.get_param('/getLaunchpad/minPoints')):
                mean1 = cv2.mean(p1)
                mean0 = cv2.mean(p0)
                
                mean1 = np.int0(mean1)
                
                good_new = p1[st==1]
                good_old = p0[st==1]
                
                # draw the corners
                for i,(new,old) in enumerate(zip(good_new,good_old)):
                    a,b = new.ravel()
                    c,d = old.ravel()
                    cv2.circle(frame,(a,b),3,(0,0,0),-1)
                    
                # Now update the previous frame and previous points
                cv2.circle(frame,(mean1[0],mean1[1]),5,(0,0,0),-1)
                    
                msgPixels.x = mean1[0]
                msgPixels.y = mean1[1]
                msgPixels.z = p1.shape[0] # report number or corners
                Detect = True

                old_frame = mask.copy()
                p0 = good_new.reshape(-1,1,2)
                
                Restart = False
        
        if kc%rospy.get_param('/cvision/loopRate') == 0: # Restart every second
            Restart = True
        
        if Restart:
            if rospy.get_param('/getLaunchpad/testFileOn'):
                _, frame = cap.read()
                frame = cv2.resize(frame,(LX,LY))
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                old_frame = frame
            else:
                old_frame = quadCam.Gry
            if rospy.get_param('/cvision/feCamera'):
                p0 = cv2.goodFeaturesToTrack(old_frame, mask = feMask, **feature_params)
            else:
                p0 = cv2.goodFeaturesToTrack(old_frame, **feature_params)
                
        DetectHold = Detect # hold for next iteration

        # publish target pixels only
        rate.sleep()
        targetPixels.publish(msgPixels)

        # show processed images to screen
        if rospy.get_param('/getLaunchpad/imgShow'):
            cv2.imshow('corners',frame)
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
        getCorners()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
