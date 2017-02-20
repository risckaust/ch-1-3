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

def getValve():

    # Initialize node

    rospy.init_node('whiteTracker', anonymous=True)
    
    # Create publishers
    targetPixels = rospy.Publisher('/getValve/xyPixels', Point32, queue_size=10)
    msgPixels = Point32()
    img_pub	 = 	   rospy.Publisher('/getValve/processedImage', Image, queue_size=10)
    bridge = CvBridge()

    # establish publish rate
    
    rate = rospy.Rate(rospy.get_param('/cvision/loopRate'))
    kernel = np.ones((3,3),np.uint8)
    quadCam = cvisionLib.getFrame()

    while not rospy.is_shutdown():

        # grab a frame
        frame = quadCam.Gry
        mask = frame

        mask = cv2.adaptiveThreshold(mask,205,cv2.ADAPTIVE_THRESH_MEAN_C,\
           cv2.THRESH_BINARY,21,3)

        edges = cv2.Canny(mask,0,20,apertureSize = 5)    
        mask = cv2.bitwise_not(mask)
        
        mask = cv2.erode(mask, kernel, iterations=2)

        pxMask = np.zeros((rospy.get_param('/cvision/LY'),rospy.get_param('/cvision/LX'),1), np.uint8)
        cv2.rectangle(pxMask, pt1 = (225,300), pt2 = (400,100), color = (255, 255, 255), thickness = -1)
        mask = cv2.bitwise_and(mask,pxMask)
                         
        if True:
            lines = cv2.HoughLines(mask,1,np.pi/180,250)
            if lines is not None:
                print lines.shape[0]
                for kc in range(0,lines.shape[0]):
                    for rho,theta in lines[kc]:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))

                        cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
                            
        if False:
            corners = cv2.goodFeaturesToTrack(mask,50,0.5,50)
            if corners is not None:
                corners = np.int0(corners)
                for i in corners:
                    x,y = i.ravel()
                    cv2.circle(frame,(x,y),5,(0,255,255),-1)

                temp = cv2.mean(corners)
                temp = np.int0(temp)
                cv2.circle(frame,(temp[0],temp[1]),10,(0,255,255),-1)
                
        # find contours in the masked image
        if OLDCV:
            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        else:
            _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        if False:
            if len(cnts) > 0:
                print len(cnts)
                cv2.drawContours(frame, cnts, -1, (0,0,0), 4)

        # show processed images to screen
        if rospy.get_param('/getLaunchpad/imgShow'):
            cv2.imshow('white',frame)
            cv2.imshow('mask',mask)
            cv2.imshow('edges',edges)
            # cv2.imshow('pxMask',pxMask)
            key = cv2.waitKey(1) & 0xFF


if __name__ == '__main__':
    try:
        getValve()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
