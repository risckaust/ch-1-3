#!/usr/bin/env python

# COMMAND LINE example: rosrun cvision getColorBlob.py local_color:='/getColors/red'
# Colors: red, blue, green, yellow

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

def getColor():

    # Initialize node

    rospy.init_node('colorTracker', anonymous=True)

    # COMMAND LINE example: rosrun cvision getColorBlob.py local_color:='/getColors/red'
    color = rospy.get_param('local_color')
    
    # Create publishers
    targetPixels = rospy.Publisher('/getColors/' + color + '/xyPixels', Point32, queue_size=10)
    msgPixels = Point32()
    targetMeters = rospy.Publisher('/getColors/' + color + '/xyMeters', Point32, queue_size=10)
    msgMeters = Point32()
    img_pub	 = 	   rospy.Publisher('/getColors/' + color + '/processedImage', Image, queue_size=10)
    bridge = CvBridge()
    
    # Create setpoint generator
    spGen = cvisionLib.pix2m() # setpoint generator

    # establish publish rate
    
    rate = rospy.Rate(rospy.get_param('/cvision/loopRate'))

    # initializations
    
    kc = 0              # iteration counter for downsample image streaming
    morph_width=2       # for erode & dilate
    morph_height=2
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
    #quadCam = cvisionLib.getCvFrame()
    
    
    # Code for testing from video file
    if rospy.get_param('/getColors/testFileOn'):
        cap = cv2.VideoCapture(rospy.get_param('/getColors/fileName'))

    while not rospy.is_shutdown():

        # grab a frame
        if rospy.get_param('/getColors/testFileOn'):
            _, frame = cap.read()
        else:
            frame = quadCam.BGR
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert to HSV
            
    	# find the color in the image. red is a special cases
    	
    	if color == 'blue':
            lower = np.array([100,100,100],np.uint8)
            upper = np.array([120,255,255],np.uint8)
        elif color == 'green':
            lower = np.array([80,100,100],np.uint8)
            upper = np.array([100,255,255],np.uint8) 
        elif color == 'yellow':
            lower = np.array([20,100,100],np.uint8)
            upper = np.array([40,255,255],np.uint8)      
        elif color == 'red':
            # find the red in the image with low "H"
            lower = np.array([0,100,100],np.uint8)
            upper = np.array([10,255,255],np.uint8)
            maskLow = cv2.inRange(hsv, lower, upper)
        
            # find the red in the image with high "H"
            lower = np.array([170,100,100],np.uint8)
            upper = np.array([180,255,255],np.uint8)
            maskHigh = cv2.inRange(hsv, lower, upper)
            
        if  color == 'red':
            mask = cv2.bitwise_or(maskLow,maskHigh)
        else:
            mask = cv2.inRange(hsv,lower,upper)

        # apply fisheye mask
        if rospy.get_param('/cvision/feCamera'):
            mask = cv2.bitwise_and(mask,feMask)

        # apply proximity mask
        if rospy.get_param('/getColors/proximityOn') and MaskItNow:
            mask = cv2.bitwise_and(mask,pxMask)
            
        if rospy.get_param('/getColors/erodeOn'):
            # opening
            mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)

        	# closing
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
            mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
        else:
            # blurring
            mask = cv2.blur(mask, (3,3))
            # clearing
            _, mask =  cv2.threshold(mask,100,255,cv2.THRESH_BINARY)
            
        # find contours in the masked image
        if OLDCV:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        else:
            _, cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
        # prep messages
        msgPixels.x = -1.0
        msgPixels.y = -1.0
        msgPixels.z = -1.0
        Detect = False
        
        if len(cnts) > 0:

            # keep largest contour
            c = max(cnts, key=cv2.contourArea)
            
            # construct & draw bounding circle
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            # cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
            
            # compute centroid of max contour vs center of circle
            
            if rospy.get_param('/getColors/useMass'):
                
                M = cv2.moments(c)
                
                if M["m00"]>rospy.get_param('/getColors/minMass'):
                
                    # flag positive detection
                    Detect = True
                    
	        	    # compute center of contour
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                if radius > rospy.get_param('/getColors/minRadius'):
                    Detect = True
                    center = x, y
    	   
    	    if Detect:
                msgPixels.x=center[0]
                msgPixels.y=center[1]
                msgPixels.z = radius # report radius of enclosing circle
                
        # create proximity mask

        pxMask = np.zeros((rospy.get_param('/cvision/LY'),rospy.get_param('/cvision/LX'),1), np.uint8)
        if Detect and DetectHold and rospy.get_param('/getColors/proximityOn'): # create a proximity mask of radius multiple
                cv2.circle(pxMask,(int(center[0]),int(center[1])),np.uint8(radius*rospy.get_param('/getColors/pxRadius')),(255,255,255),-1)
                MaskItNow = True
        else:
            MaskItNow = False
                
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

        # show processed images to screen
        if rospy.get_param('/getColors/imgShow'):
            cv2.imshow(color,frame)
            cv2.imshow('pxMask',pxMask)
            key = cv2.waitKey(1) & 0xFF

        # published downsized/grayscale processed image
        STREAM_RATE = rospy.get_param('/getColors/imgStreamRate')
        if rospy.get_param('/getColors/imgStream'): # stream processed image
            if (kc*STREAM_RATE)%rospy.get_param('/cvision/loopRate') < STREAM_RATE:
                gray_frame=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray_frame=imutils.resize(gray_frame, width=rospy.get_param('/cvision/LX')/2)
                img_pub.publish(bridge.cv2_to_imgmsg(gray_frame, encoding="passthrough"))

        kc = kc + 1

if __name__ == '__main__':
    try:
        getColor()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
