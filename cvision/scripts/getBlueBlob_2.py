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

class ImageSubscriber:
	def __init__(self):
		self.img= Image()
		self.height=0
		self.width=0
	def cb(self, msg):
		self.img	= msg
		self.height	=msg.height
		self.width	=msg.width

def getBlueBlob():
    # centroid of detected object
    centroid = rospy.Publisher('blue_xy', Point32, queue_size=10)
    # processed image for logging
    img_pub = rospy.Publisher('processed_image', Image, queue_size=10)
    # processed gray image for downstream
    img_stream_pub= rospy.Publisher('img_stream', Image, queue_size=10)
    # subscriber for VREP image
    img_obj= ImageSubscriber()
    vrep_img_sub = rospy.Subscriber('/VREP_Image', Image, img_obj.cb)

    
	
    # flage to whether to write images to disk
    write_images=False
    saving_rate=3.0	# image saving rate, Hz
    saving_k=0.0	# saving rate counter

    # flag to stream images to ground station
    STREAM_IMG=True
    stream_rate=2.0	# Hz
    stream_k=0.0	# counter

    # prep to publish two topics: x & y of ellipse center
    msg = Point32()

    # initialize node & set rate in Hz
    rospy.init_node('tracker', anonymous=True)
    loop_rate=20
    rate = rospy.Rate(loop_rate)

    morph_width=2
    morph_height=2

    # start video stream
    # Not needed as we are reading an image from VREP
    #cap = cv2.VideoCapture(0)

    img_k=0

    start_stream_t=time.time()
    start_saving_t=time.time()

    # create cv bridge object
    bridge = CvBridge()

    while not rospy.is_shutdown():

	if STREAM_IMG :
		stream_k=stream_k+1

	if write_images :
		saving_k=saving_k+1

        # grab a frame: get it from VREP image topic
	if img_obj.height > 0:
		frame = bridge.imgmsg_to_cv2(img_obj.img, desired_encoding="passthrough")
		#frame=cv2.flip(frame,0)
		img_obj.height=0
		img_obj.width=0
	else:
		print "could not get image"
		continue
        #_, frame = cap.read()
        # frame = imutils.resize(frame, width=640) # resize to std

        # convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
	frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    	# find the blue in the image
        lower = np.array([60,121,180],np.uint8)
	#lower = np.array([100,150,0],dtype = "uint8")
	#upper = np.array([140,255,255],dtype = "uint8")
        upper = np.array([130,255,255],np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

       # opening
	mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
	mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)

	# closing
	mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)
	mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morph_width,morph_height)), iterations=1)

        # find contours in the masked image
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    	    cv2.CHAIN_APPROX_NONE)
    
        # prep messages
        msg.x = -1
        msg.y = -1
	center= None
        if len(cnts) > 0:
            # keep largest contour
            c = max(cnts, key=cv2.contourArea)
            # construct & draw bounding ellipse
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
		msg.x=center[0]
		msg.y=center[1]
		msg.z=0.0        
	msg.x=center[0]
	msg.y=center[1]
	# publish centroid
        centroid.publish(msg)
	
	# print image size
	#print frame.shape[:2]

        # clock sync to rate          
        rate.sleep()

        # draw frame and mask
        #cv2.imshow('frame',frame)
        #cv2.imshow('mask',mask)

        key = cv2.waitKey(1) & 0xFF


if __name__ == '__main__':
    try:
        getBlueBlob()
    except rospy.ROSInterruptException:
        # cleanup the camera and close any open windows [doesn't work]
        cap.release()
        cv2.destroyAllWindows()
        pass


