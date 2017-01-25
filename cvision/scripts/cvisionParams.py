#####
#
# cvisionParams.py
#
#####

import rospy

def setParams(ns):

    # ROS parameters for general vision tasks
    rospy.set_param(ns+'/cvision/loopRate', 20.0)                   # loop rate for vision algorithms
    rospy.set_param(ns+'/cvision/LX', 640)                          # full size screen width
    rospy.set_param(ns+'/cvision/LY', 480)                          # full size screen height
    rospy.set_param(ns+'/cvision/camRotate', False)                  # camera rotated 90 degrees CCW facing down
    rospy.set_param(ns+'/cvision/feCamera', False)                   # use fisheye mask and pixel2meters
    rospy.set_param(ns+'/cvision/gripperOffset', 0.0)               # gripper location from screen center in NED x-axis 
                                                                 # measured in pixels when landed
    # ROS parameters getColors
    rospy.set_param(ns+'/getColors/imgShow', False)                  # show processed images to screen
    rospy.set_param(ns+'/getColors/imgStream', True)                # stream reduced processed images
    rospy.set_param(ns+'/getColors/imgStreamRate', 3)               # streaming rate
    rospy.set_param(ns+'/getColors/red','red')
    rospy.set_param(ns+'/getColors/blue','blue')
    rospy.set_param(ns+'/getColors/green','green')
    rospy.set_param(ns+'/getColors/yellow','yellow') 
    rospy.set_param(ns+'/getColors/minMass',50.0)                   # minimum mass to detect a color blob
    rospy.set_param(ns+'/getColors/erodeOn',False)                  # use erode/dilate vs blurring 
    rospy.set_param(ns+'/getColors/proximityOn',True)               # use proximity filter on most recent detection
    rospy.set_param(ns+'/getColors/pxRadius', 5.0)                  # radius multiplier for proximity mask
    rospy.set_param(ns+'/getColors/imgShow', True)                  # show processed images to screen
    rospy.set_param(ns+'/getColors/imgStream', True)                # stream reduced processed images
    rospy.set_param(ns+'/getColors/imgStreamRate', 3)               # streaming rate
    
    # ROS parameters for pix2m
    rospy.set_param(ns+'/pix2m/m2pix', 0.00104167)                  # 0.5m = 480pixels
    rospy.set_param(ns+'/pix2m/altCal',1.2)                         # calibration altitude (used in main loop)
    
    # ROS parameters getLaunchPadx3 ***TO BE ELIMINATED***
    rospy.set_param(ns+'/getLaunchpad/minMass',50.0)                # minimum mass to detect a white blob
    rospy.set_param(ns+'/getLaunchPad/pxMasking', False)            # use proximity masking
    rospy.set_param(ns+'/getLaunchpad/pxRadius', 1.2)               # radius multiplier for proximity mask (both white & circle)
    rospy.set_param(ns+'/getLaunchpad/minPoints',4)                 # minimum number of corners for positive detection
    rospy.set_param(ns+'/getLaunchPad/centroidThresh', 10000.0)     # threshold for white centroid detection
    rospy.set_param(ns+'/getLaunchPad/circleTol', 1.5)              # radius multiplier for circles
    rospy.set_param(ns+'/getLaunchPad/erodeOn', False)              # use erode/dilate vs blur
    rospy.set_param(ns+'/getLaunchPad/liberal', False)              # allow lone bright white detection
    rospy.set_param(ns+'/getLaunchPad/reduction', 2)                # image dimension reduction for realtime processing speed
    rospy.set_param(ns+'/getLaunchPad/hoverLow', False)             # corner override (temp)
    rospy.set_param(ns+'/getLaunchPad/imgShow', True)               # show processed images to screen
    rospy.set_param(ns+'/getLaunchPad/imgStream', True)             # stream reduced processed images
    rospy.set_param(ns+'/getLaunchPad/imgStreamRate', 3)            # streaming rate




 
