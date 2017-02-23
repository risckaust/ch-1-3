#####
#
# cvisionParams.py
#
#####

import rospy

def setParams(ns):

    # ROS parameters for general vision tasks
    rospy.set_param(ns+'/cvision/loopRate', 30.0)                   # loop rate for vision algorithms
    rospy.set_param(ns+'/cvision/reduce', False)                     # reduce frame size
    if rospy.get_param(ns+'/cvision/reduce'):
        rospy.set_param(ns+'/cvision/LX', 320)                          # half size screen width
        rospy.set_param(ns+'/cvision/LY', 240)                          # half size screen height
    else:
        rospy.set_param(ns+'/cvision/LX', 640)                          # full size screen width
        rospy.set_param(ns+'/cvision/LY', 480)                          # full size screen height

    rospy.set_param(ns+'/cvision/camRotate', False)                 # camera rotated 90 degrees CCW facing down
    rospy.set_param(ns+'/cvision/feCamera', True)                   # use fisheye mask and meter conversions
    rospy.set_param(ns+'/cvision/gripperOffset', 0.0)               # gripper location from screen center in NED x-axis 
                                                                 # measured in pixels when landed at full size screen
                                                                 
    rospy.set_param(ns+'/cv_camera/rate',rospy.get_param(ns+'/cvision/loopRate'))
    rospy.set_param(ns+'/cv_camera/image_width',rospy.get_param(ns+'/cvision/LX'))
    rospy.set_param(ns+'/cv_camers/image_height',rospy.get_param(ns+'/cvision/LY'))
        
    # ROS parameters getLaunchpad
    rospy.set_param(ns+'/getLaunchpad/useMass', False)
    rospy.set_param(ns+'/getLaunchpad/minMass',50.0)                # minimum mass to detect a color blob
    rospy.set_param(ns+'/getLaunchpad/minRadius',10.0)              # minimum circle to detect a white blob
    rospy.set_param(ns+'/getLaunchpad/erodeOn',True)                # use erode/dilate vs blurring in white detection 
    rospy.set_param(ns+'/getLaunchpad/pxRadius', 1.2)               # radius multiplier for proximity mask (both white & circle) 
    rospy.set_param(ns+'/getLaunchpad/minPoints',4)                 # minimum number of corners for positive detection
    
    rospy.set_param(ns+'/getLaunchpad/testFileOn',False)            # binary for testing on file
    rospy.set_param(ns+'/getLaunchpad/fileName','/home/shamma/Documents/jeff_ws/variable.m4v')
    #rospy.set_param('/getLaunchpad/fileName','/home/shamma/Documents/MultiObjectImages/imgset4/output.mp4')
    
    rospy.set_param(ns+'/getLaunchpad/imgShow', False)               # show processed images to screen
    rospy.set_param(ns+'/getLaunchpad/imgStream', True)             # stream reduced processed images
    rospy.set_param(ns+'/getLaunchpad/imgStreamRate', 3)            # streaming rate
    
    # ROS parameters getColors
    rospy.set_param(ns+'/getColors/red','red')
    rospy.set_param(ns+'/getColors/blue','blue')
    rospy.set_param(ns+'/getColors/green','green')
    rospy.set_param(ns+'/getColors/yellow','yellow')
    
    rospy.set_param(ns+'/getColors/useMass', True)                 # use mass for color blob (otherwise, only circle)
    rospy.set_param(ns+'/getColors/minMass',50.0)                   # minimum mass to detect a color blob
    rospy.set_param(ns+'/getColors/minRadius',10.0)                 # minimum cirlce radius to detect a color blob
    rospy.set_param(ns+'/getColors/erodeOn',True)                   # use erode/dilate vs blurring 
    rospy.set_param(ns+'/getColors/proximityOn',True)               # use proximity filter on most recent detection
    rospy.set_param(ns+'/getColors/pxRadius', 1.2)                  # radius multiplier for proximity mask
    
    rospy.set_param(ns+'/getColors/testFileOn',False)               # binary for testing on file
    rospy.set_param(ns+'/getColors/fileName','/home/shamma/Documents/MultiObjectImages/imgset4/output.mp4')
    
    rospy.set_param(ns+'/getColors/imgShow', False)                  # show processed images to screen
    rospy.set_param(ns+'/getColors/imgStream', True)                # stream reduced processed images
    rospy.set_param(ns+'/getColors/imgStreamRate', 3)               # streaming rate
    
    # ROS parameters for pix2m
    rospy.set_param(ns+'/pix2m/m2pix', 0.00104167)                  # 0.5m = 480pixels
    rospy.set_param(ns+'/pix2m/altCal',1.3)                         # calibration altitude (used in main loop)
    
    # ROS parameters getLaunchPadx3 ***TO BE ELIMINATED***
    rospy.set_param(ns+'/getLaunchPadx3/pxMasking', False)            # use proximity masking
    rospy.set_param(ns+'/getLaunchPadx3/centroidThresh', 10000.0)     # threshold for white centroid detection
    rospy.set_param(ns+'/getLaunchPadx3/circleTol', 1.5)              # radius multiplier for circles
    rospy.set_param(ns+'/getLaunchPadx3/erodeOn', False)              # use erode/dilate vs blur
    rospy.set_param(ns+'/getLaunchPadx3/liberal', False)              # allow lone bright white detection
    rospy.set_param(ns+'/getLaunchPadx3/reduction', 2)                # image dimension reduction for realtime processing speed
    rospy.set_param(ns+'/getLaunchPadx3/hoverLow', False)             # corner override (temp)
 
    rospy.set_param(ns+'/getLaunchPadx3/imgShow', True)               # show processed images to screen
    rospy.set_param(ns+'/getLaunchPadx3/imgStream', True)             # stream reduced processed images
    rospy.set_param(ns+'/getLaunchPadx3/imgStreamRate', 3)            # streaming rate
