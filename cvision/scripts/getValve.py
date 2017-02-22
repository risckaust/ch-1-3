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
        gray = quadCam.Gry
        mask = quadCam.Gry
        
        ### SAMPLES ####
        # cv2.blur(mask,(5,5))
        # cv2.GaussianBlur(mask,(5,5),0)
        # cv2.medianBlur(mask,5)
        # cv2.bilateralFilter(mask,9,75,75)
        # cv2.adaptiveThreshold(mask,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,5)
        # cv2.adaptiveThreshold(mask,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
        # cv2.Canny(mask,0,20,apertureSize = 3)
        # cv2.erode(mask, kernel, iterations=2)
        # cv2.bitwise_not(mask)
        # cv2.HoughLines(mask,1,np.pi/180,50)
        # cv2.goodFeaturesToTrack(mask,50,0.5,50)
        # cv2.drawContours(mask, cnts, -1, (0,0,0), 4)
        # if OLDCV:
        #     cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        # else:
        #     _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        #############
        
        # Mask all but center
        
        pxMask = np.zeros((480,640,1), np.uint8)
        radius = 150
        cv2.rectangle(pxMask, pt1 = (320-radius,240+radius), pt2 = (320+radius,240-radius), color = 255, thickness = -1)
        mask = cv2.bitwise_and(mask,pxMask)
        
        # Find center of valve

        _, mask = cv2.threshold(mask,205,255,cv2.THRESH_BINARY)
        
        if OLDCV:
            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        else:
            _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            
        c0 = 320,240
        
        if False:
            if len(cnts) > 0:
                # keep largest contour
                c = max(cnts, key=cv2.contourArea)
                # construct & draw bounding circle
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                cv2.circle(gray, (int(x), int(y)), int(radius),(0, 0, 0), 2)
                c0 = int(x),int(y)
        
        # Create proximity mask around center
        
        radius = 100
        pxMask = np.zeros((480,640,1), np.uint8)
        cv2.rectangle(pxMask, \
            pt1 = (c0[0]-radius,c0[1]+radius), pt2 = (c0[0]+radius,c0[1]-radius), color = 255, thickness = -1)
        mask = cv2.bitwise_and(mask,pxMask)
            
        # threshold
        
        mask = cv2.adaptiveThreshold(mask,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
           cv2.THRESH_BINARY,11,1) # 11, 5
        
        mask = cv2.bitwise_not(mask)
        
        # Shrink proximity mask to remove outline
        
        pxMask = np.zeros((480,640,1), np.uint8)
        cv2.rectangle(pxMask, pt1 = (c0[0]-90,c0[1]+90), pt2 = (c0[0]+90,c0[1]-90), color = 255, thickness = -1)
        mask = cv2.bitwise_and(mask,pxMask) 
        
        # exaggerate white
        
        mask = cv2.dilate(mask, kernel, iterations=2) #iter = 1
        
        # Template matching

        best = 0.0, 0.0, 0.0, 0.0 # score, r, q (degrees)
        score = -np.inf
            
        for dx in range(-10,11,5):
            for dy in range(-10,11,5):
                for r in range(8,44,2):
                    for q in range(0,90,5):
                    
                        c = c0[0]+dx, c0[1]+dy
                        
                        square = np.zeros((480,640,1), np.uint8)
                        cv2.rectangle(square, pt1 = (c[0]-r,c[1]+r), pt2 = (c[0]+r,c[1]-r), color = 255, thickness = 2)

                        M = cv2.getRotationMatrix2D(c,q,1.0)
                        testMask = cv2.warpAffine(square,M,(640,480))
                        
                        overlap = cv2.bitwise_and(mask,testMask)

                        temp = 0.0
                        for kc in range (0,10):
                            temp = temp + cv2.sumElems(overlap)[0]
                        
                        if temp > score:
                            score = temp
                            best = c[0], c[1], r, q
                            # draw current champion
                            cB = best[0],best[1]
                            rB = best[2]
                            qB = best[3]
                            square = np.zeros((480,640,1), np.uint8)
                            cv2.rectangle(square, \
                                pt1 = (cB[0]-rB,cB[1]+rB), pt2 = (cB[0]+rB,cB[1]-rB), color = 255, thickness = 2)
                            M = cv2.getRotationMatrix2D(cB,qB,1.0)
                            bestMask = cv2.warpAffine(square,M,(640,480))
                            bestMask = cv2.bitwise_not(bestMask)
        
                            gray2 = cv2.bitwise_and(gray,bestMask)
                            mask2 = cv2.bitwise_and(mask,bestMask)
                                        
                            cv2.imshow('temp',gray2)
                            cv2.imshow('mask',mask2)
                            key = cv2.waitKey(1) & 0xFF
                                                
        
        print best
                            
        if False:
            cB = best[0],best[1]
            rB = best[2]
            qB = best[3]
            square = np.zeros((480,640,1), np.uint8)
            cv2.rectangle(square, pt1 = (cB[0]-rB,cB[1]+rB), pt2 = (cB[0]+rB,cB[1]-rB), color = 255, thickness = 2)
            M = cv2.getRotationMatrix2D(cB,qB,1.0)
            bestMask = cv2.warpAffine(square,M,(640,480))
            bestMask = cv2.bitwise_not(bestMask)
            
            gray3 = cv2.bitwise_and(gray,bestMask)
        


if __name__ == '__main__':
    try:
        getValve()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
