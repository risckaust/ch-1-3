#####
#
# cvisionParams.py
#
#####

import rospy

def setParams():

    # ROS parameters for general vision tasks
    rospy.set_param('/cvision/loopRate', 20.0)                   # loop rate for vision algorithms
    rospy.set_param('/cvision/LX', 640)                          # full size screen width
    rospy.set_param('/cvision/LY', 480)                          # full size screen height
    rospy.set_param('/cvision/camRotate', False)                  # camera rotated 90 degrees CCW facing down
    rospy.set_param('/cvision/feCamera', True)                   # use fisheye mask and pixel2meters

    # ROS parameters getLaunchPadx3
    rospy.set_param('/getLaunchPad/pxMasking', False)            # use proximity masking
    rospy.set_param('/getLaunchPad/centroidThresh', 10000.0)     # threshold for white centroid detection
    rospy.set_param('/getLaunchPad/circleTol', 1.5)              # radius multiplier for circles
    rospy.set_param('/getLaunchPad/erodeOn', False)              # use erode/dilate vs blur
    rospy.set_param('/getLaunchPad/liberal', False)              # allow lone bright white detection
    rospy.set_param('/getLaunchPad/reduction', 2)                # image dimension reduction for realtime processing speed
    rospy.set_param('/getLaunchPad/hoverLow', False)             # corner override (temp)
    rospy.set_param('/getLaunchPad/imgShow', False)               # show processed images to screen
    rospy.set_param('/getLaunchPad/imgStream', True)             # stream reduced processed images
    rospy.set_param('/getLaunchPad/imgStreamRate', 3)            # streaming rate
    
    # ROS parameters getColors
    rospy.set_param('/getColors/imgShow', False)                  # show processed images to screen
    rospy.set_param('/getColors/imgStream', True)                # stream reduced processed images
    rospy.set_param('/getColors/imgStreamRate', 3)               # streaming rate
    rospy.set_param('/getColors/red','red')
    rospy.set_param('/getColors/blue','blue')
    rospy.set_param('/getColors/green','green')
    rospy.set_param('/getColors/white','white')
    
    # ROS parameters for pix2m
    rospy.set_param('/pix2m/m2pix', 0.00104167)                  # 0.5m = 480pixels
    rospy.set_param('/pix2m/altCal',1.2)                         # calibration altitude (used in main loop)




 
