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

def getLaunchpad():

    # Initialize node

    rospy.init_node('launchpadTracker', anonymous=True)
    
    # Create publishers
    targetPixels = rospy.Publisher('/getLaunchpad/launchpad/xyPixels', Point32, queue_size=10)
    msgPixels = Point32()
    targetMeters = rospy.Publisher('/getLaunchpad/launchpad/xyMeters', Point32, queue_size=10)
    msgMeters = Point32()
    img_pub	 = 	   rospy.Publisher('/getLaunchpad/launchpad/processedImage', Image, queue_size=10)
    bridge = CvBridge()
    
    # Create fisheye mask
    LX = rospy.get_param('/cvision/LX')
    LY = rospy.get_param('/cvision/LY')
    feMask = np.zeros((LY,LX,1), np.uint8)
    cv2.circle(feMask,(LX/2,LY/2),LX/2,(255,255,255),-1)
    
    # Create setpoint generator
    spGen = cvisionLib.pix2m() # setpoint generator

    # establish publish rate
    rate = rospy.Rate(rospy.get_param('/cvision/loopRate'))

    # initializations
    
    kc = 0              # iteration counter for downsample image streaming
    morph_width=2       # for erode & dilate
    morph_height=2
    Detect = False      # for detection logic
    DetectHold = False
    TOL = rospy.get_param('/getLaunchpad/agreeTol')           # tolerance for detection agreement
    MaskItNow = False
    Restart = True      # for corner tracking
    # Parameters for Shi Tomasi corner detection
    feature_params = dict( maxCorners = 20, # 10
                       qualityLevel = 0.5,  # 0.5
                       minDistance = 20)    # 20
#                       minDistance = 25,
#                       blockSize = 7 ) 
                       
    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)) 
                  
                   
    getWhite = cvisionLib.xyzVar()
    getCircle = cvisionLib.xyzVar()
    getCorners = cvisionLib.xyzVar()
    
    # start video stream: Replaces
    #   cap = cv2.VideoCapture(0) or cap = cv2.VideoCapture('file.mp4')
    #   _, frame = cap.read()
    quadCam = cvisionLib.getFrame()
    
    while not rospy.is_shutdown():
        
        # grab a frame
        frame = quadCam.BGR
        mask = quadCam.Gry
        
        # apply fisheye mask
        if rospy.get_param('/cvision/feCamera'):
            mask = cv2.bitwise_and(mask,feMask)

        # apply proximity mask
        if MaskItNow:
            mask = cv2.bitwise_and(mask,pxMask)
        
        maskHold = mask.copy()

        ####
        ####
        # mimic getWhiteBlob
        ####
        ####
        
        getWhite.x = -1.0
        getWhite.y = -1.0
        getWhite.z = -1.0
        Detect = False

        # mask = maskHold (already)
        
        # extract superwhite
        # or black? _, mask = cv2.threshold(mask,0,30,cv2.THRESH_BINARY)        
        _, mask = cv2.threshold(mask,215,255,cv2.THRESH_BINARY)

        # smooth image
        if rospy.get_param('/getLaunchpad/erodeOn'):
            # opening
            mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)

        	# closing
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
            mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
        else:
            # blurring
            mask = cv2.blur(mask, (3,3))
            # cleaning
            # or black? _, mask =  cv2.threshold(mask,0,10,cv2.THRESH_BINARY)
            _, mask =  cv2.threshold(mask,245,255,cv2.THRESH_BINARY)
        
        # find contours in the masked image
        if OLDCV:
            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        else:
            _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
                        
        if len(cnts) > 0:

            # keep largest contour
            c = max(cnts, key=cv2.contourArea)
            
            # construct & draw bounding circle
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # Draw whiteBlob circles and center
            # cv2.circle(frame, (int(x), int(y)), int(radius),(0, 0, 0), 5)
            # cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 0), -1)
	    	
            if rospy.get_param('/getLaunchpad/useMass'):
                
                # M = cv2.moments(c)
                M = cv2.moments(mask)  # Find moments of mask or contour?
                
                if M["m00"]>rospy.get_param('/getLaunchpad/minMass'):
                
                    # flag positive detection
                    Detect = True
                    
	        	    # compute center of contour
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                if radius > rospy.get_param('/getLaunchpad/minRadius'):
                    Detect = True
                    center = x, y
                    
            if Detect:
                getWhite.x=center[0]
                getWhite.y=center[1]
                getWhite.z = radius # report radius of enclosing circle        

        ####                    
        ####
        # mimic getCircle
        ####
        ####
                
        getCircle.x = -1.0
        getCircle.y = -1.0
        getCircle.z = -1.0
        Detect = False
        

        mask = maskHold
        # blur image
        #### mask = cv2.blur(mask, (5,5))

        # extract circles from grayscale
        # param2: higher is harder (fewer circles)

        if OLDCV:
            circles = cv2.HoughCircles(mask,cv.CV_HOUGH_GRADIENT,1,LY,
                param1=50,param2=50,minRadius=LY/50,maxRadius=LY/2)
        else:
            circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,1,LY,
                param1=50,param2=50,minRadius=LY/50,maxRadius=LY/2)

        # assess circles 

        if circles is not None:
            center = circles[0,0]
            Detect = True
            # Draw hough circle and center
            #cv2.circle(frame,(center[0],center[1]),center[2],(0,0,0),5)
            #cv2.circle(frame, (int(center[0]), int(center[1])), 3, (0, 0, 0), -1)
            getCircle.x = center[0] # x
            getCircle.y = center[1] # y
            getCircle.z = center[2] # radius
        else:
            Detect = False
        
        
        ####                    
        ####
        # mimic getCorners
        ####
        ####
                
        getCorners.x = -1.0
        getCorners.y = -1.0
        getCorners.z = -1.0
        Detect = False

        mask = maskHold
        
        if Restart:
            old_frame = mask
            
            if rospy.get_param('/cvision/feCamera'):
                p0 = cv2.goodFeaturesToTrack(old_frame, mask = feMask, **feature_params)
            else:
                p0 = cv2.goodFeaturesToTrack(old_frame, **feature_params)
            
            if p0 is not None:
                for i in p0:
                    x,y = i.ravel()
                    cv2.circle(frame,(x,y),5,(0,0,0),-1)
                temp = cv2.mean(p0)
                temp = np.int0(temp)
                getCorners.x = temp[0]
                getCorners.y = temp[1]
                getCorners.z = p0.shape[0]
                Detect = True
                Restart = False
        else:
            Restart = True
            # calculate optical flow
            if p0 is not None:
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_frame, mask, p0, None, **lk_params)
            else:
                p1 = None
                
            # Select good points

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
                        cv2.circle(frame,(a,b),5,(0,0,0),-1)
                        
                    # Now update the previous frame and previous points
                    cv2.circle(frame,(mean1[0],mean1[1]),10,(0,0,0),-1)
                        
                    getCorners.x = mean1[0]
                    getCorners.y = mean1[1]
                    getCorners.z = p1.shape[0] # report number or corners
                    Detect = True

                    old_frame = mask.copy()
                    p0 = good_new.reshape(-1,1,2)
                    
                    Restart = False          
        
        
        N = rospy.get_param('/getLaunchpad/cornerRestart') # restart every N seconds
        if kc%(N*rospy.get_param('/cvision/loopRate')) == 0:
            Restart = True
             
        ####
        ####
        # transition to parallel getLaunchpad
        ####
        ####
        
        dataWhite = getWhite.x, getWhite.y, getWhite.z
        dataCircle = getCircle.x, getCircle.y, getCircle.z
        dataCorners = getCorners.x, getCorners.y, getCorners.z

        # detection acceptance logic
        if dataWhite[2] > 0:
            detectWhite = True
        else:
            detectWhite = False
        
        if dataCorners[2] > 0:
            detectCorners = True
        else:
            detectCorners = False
            
        if dataCircle[2] > 0:
            detectCircle = True
        else:
            detectCircle = False
        
        Detect = False
        Skip = False
        CX = -1
        CY = -1

        if detectCircle and detectWhite and not Skip: # Circle + Superwhite centroid
            error = (dataCircle[0] - dataWhite[0])**2 + (dataCircle[1] - dataWhite[1])**2
            if sqrt(error) < TOL*dataCircle[2]:
                Detect = True
                CX = dataCircle[0]
                CY = dataCircle[1]
                pxRad = max(dataCircle[2],dataWhite[2])
                Skip = True 

        if detectCircle and detectCorners and not Skip: # Circle + Corners
            error = (dataCircle[0] - dataCorners[0])**2 + (dataCircle[1] - dataCorners[1])**2
            if sqrt(error) < TOL*dataCircle[2]:
                Detect = True
                CX = dataCircle[0]
                CY = dataCircle[1]
                pxRad = dataCircle[2]
                Skip = True 

        if detectWhite and detectCorners and not Skip: # Superwhite centroid + Corners
            error = (dataWhite[0] - dataCorners[0])**2 + (dataWhite[1] - dataCorners[1])**2
            if sqrt(error) < TOL*dataWhite[2]:
                Detect = True
                CX = dataWhite[0] ### Corners or White?
                CY = dataWhite[1]
                pxRad = dataWhite[2]
                Skip = True
     
        if Detect:
            msgPixels.x = CX
            msgPixels.y = CY
            msgPixels.z = 1.0
        else:
            msgPixels.x = -1.0
            msgPixels.y = -1.0
            msgPixels.z = -1.0
            
        DetectHold = Detect # hold for next iteration
        
        # publish (unrotated) pixels and (rotated) meters
        
        rate.sleep()
        
        targetPixels.publish(msgPixels)

        if rospy.get_param('/cvision/camRotate') and msgPixels.z > 0:        # rotate camera if needed
            msgPixels.x, msgPixels.y = cvisionLib.camRotate(msgPixels.x, msgPixels.y)

        if rospy.get_param('/cvision/feCamera'):                             # convert pixels to to meters
            (msgMeters.x, msgMeters.y, msgMeters.z) = spGen.targetFishEye(msgPixels)
        else:
            (msgMeters.x, msgMeters.y, msgMeters.z) = spGen.target(msgPixels)

        targetMeters.publish(msgMeters)
        
        # prep proximity mask
        MaskItNow = False
        pxMask = np.zeros((rospy.get_param('/cvision/LY'),rospy.get_param('/cvision/LX'),1), np.uint8)
        if Detect and DetectHold:
            radius = pxRad
            cv2.circle(pxMask,(int(msgPixels.x),int(msgPixels.y)),int(radius*rospy.get_param('/getLaunchpad/pxRadius')),(255,255,255),-1)
            MaskItNow = True
       
        
        # prep processed images

        if dataWhite[2] > 0:
            cv2.circle(frame, (int(dataWhite[0]), int(dataWhite[1])), int(dataWhite[2]),(0, 0, 0), 5)
        if dataCircle[2] > 0:
            cv2.circle(frame, (int(dataCircle[0]), int(dataCircle[1])), int(dataCircle[2]),(255, 0, 0), 5)
        if dataCorners[2] > 0:
            cv2.circle(frame, (int(dataCorners[0]), int(dataCorners[1])), 5,(0, 255, 255), -1)
        if Detect:
            cv2.circle(frame, (int(CX), int(CY)), 10,(0, 0, 255), -1)

        # show processed images to screen
        if rospy.get_param('/getLaunchpad/imgShow'):
            cv2.imshow('launchpad',frame)
            cv2.imshow('pxMask',maskHold)
            key = cv2.waitKey(1) & 0xFF

        # published downsized/grayscale processed image
        STREAM_RATE = rospy.get_param('/getLaunchpad/imgStreamRate')
        if rospy.get_param('/getLaunchpad/imgStream'): # stream processed image
            if (kc*STREAM_RATE)%rospy.get_param('/cvision/loopRate') < STREAM_RATE:
                gray_frame=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray_frame=imutils.resize(gray_frame, width=rospy.get_param('/cvision/LX')/2)
                img_pub.publish(bridge.cv2_to_imgmsg(gray_frame, encoding="passthrough"))
                
        kc = kc + 1

if __name__ == '__main__':
    try:
        getLaunchpad()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
