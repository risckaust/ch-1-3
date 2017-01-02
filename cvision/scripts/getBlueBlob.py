#!/usr/bin/env python

# Publishing demo that publishes multiple scalar values

import rospy
import numpy as np
import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math
import time

# from goprohero import GoProHero
from geometry_msgs.msg import Point32

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

FECAMERA = rospy.get_param('/getLaunchPad/feCamera')
CAMFLIP = rospy.get_param('/getLaunchPad/camFlip')
IMGSHOW = rospy.get_param('/getLaunchPad/imgShow')

###################################

# Create publishers
targetPixel = rospy.Publisher('target_xyPixel', Point32, queue_size=10)
targetSp = rospy.Publisher('target_xySp', Point32, queue_size=10)
img_pub = rospy.Publisher('processed_image', Image, queue_size=10)
img_stream_pub= rospy.Publisher('img_stream', Image, queue_size=10)

msgPixel = Point32()
msgSp = Point32()
bridge = CvBridge()  
spGen = cvisionLib.pix2m() # setpoint generator

def getBlueBlob():

    # flag to whether to write images to disk
    write_images=True
    saving_rate=3.0	# image saving rate, Hz
    saving_k=0.0	# saving rate counter

    # flag to stream images to ground station
    STREAM_IMG=True
    stream_rate=2.0	# Hz
    stream_k=0.0	# counter

#    camera = GoProHero(password='odroid')
#    camera.command('record','on')
#    status = camera.status()

    # initialize node & set rate in Hz
    rospy.init_node('tracker', anonymous=True)
    loop_rate=20
    rate = rospy.Rate(loop_rate)

    morph_width=2
    morph_height=2

    # start video stream
    cap = cv2.VideoCapture(0)

    img_k=0

    start_stream_t=time.time()
    start_saving_t=time.time()

    while not rospy.is_shutdown():

        if STREAM_IMG :
            stream_k=stream_k+1

        if write_images :
            saving_k=saving_k+1

        # grab a frame
        _, frame = cap.read()

        # convert to HSV        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    	# find the blue in the image
        lower = np.array([60,121,180],np.uint8)
        upper = np.array([130,255,255],np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

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
        msgPixel.x = -1
        msgPixel.y = -1
        center= None
        radius = 0
        if len(cnts) > 0:
            # keep largest contour
            c = max(cnts, key=cv2.contourArea)
            # construct & draw bounding circle
            ((x, y), radius) = cv2.minEnclosingCircle(c)
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

        if STREAM_IMG:
            if (time.time()-start_stream_t)>(1.0/stream_rate):
			    start_stream_t=time.time()
			    # get gray scale image
			    gray_frame=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			    gray_frame=imutils.resize(gray_frame, width=200)
			    img_stream_pub.publish(bridge.cv2_to_imgmsg(gray_frame, encoding="mono8"))

    	if write_images:
    		if (time.time()-start_saving_t)>(1.0/saving_rate):
			    start_saving_t=time.time()
			    resized_frame=imutils.resize(frame, width=300)
			    img_pub.publish(bridge.cv2_to_imgmsg(resized_frame, encoding="bgr8"))

        if center is None:
		    center=(-1.0,-1.0)
		
        msgPixel.x=center[0]
        msgPixel.y=center[1]
        msgPixel.z=0.0        
        
        if CAMFLIP and msgPixel.x > 0 and msgPixel.y > 0:
            hold = msgPixel.x
            msgPixel.x = msgPixel.y
            msgPixel.y = rospy.get_param('/pix2m/LY') - hold
            
        if FECAMERA:
            (msgSp.x, msgSp.y, msgSp.z) = spGen.targetFishEye(msgPixel)
        else:
            (msgSp.x, msgSp.y, msgSp.z) = spGen.target(msgPixel)

        rate.sleep()
        targetPixel.publish(msgPixel)
        targetSp.publish(msgSp)

        # draw frame and mask
        if IMGSHOW:
            cv2.imshow('frame',frame)
            cv2.imshow('mask',mask)

        key = cv2.waitKey(1) & 0xFF


if __name__ == '__main__':
    try:
        getBlueBlob()
    except rospy.ROSInterruptException:
        # cleanup the camera and close any open windows [doesn't work]
        cap.release()
        cv2.destroyAllWindows()
        pass


