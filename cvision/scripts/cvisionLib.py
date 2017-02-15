import rospy
import numpy as np
import cv2

from math import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Check version of OpenCV

if cv2.__version__.startswith('2'):
    OLDCV = True
else:
    OLDCV = False
    
if OLDCV:
    import cv2.cv as cv

###################################
#
# class pix2m 
#   Convert pixel center to distance setpoint in body NED (m)
#
# Modules:
#   self.target(center) = position setpoint in body NED coordinates given center.x, center.y, center.z = detection info
#   self.targetFE(center) = same but for FE lens
#   (xSp,ySp,info) = position setpoint (m) body NED coordinates with detection info (positive value <=> detection)
#
# NOTE: Altitude correction done in main loop using /cvision/altCal = altitude of camera calibration (m)
#
# ROS parameters:
#   /pix2m/m2pix = meters-per-pixel ratio
#   /pix2m/altCal = calibration altitude (used in main loop)
#
# Fields:
#   LX, LY, m2pix, gripperOffset
#   target(), targetFE()
#####

class pix2m():
    def __init__(self):
        self.LX = rospy.get_param('/cvision/LX')
        self.LY = rospy.get_param('/cvision/LY')
        self.m2pix = rospy.get_param('/pix2m/m2pix')
        self.gripperOffset = rospy.get_param('/cvision/gripperOffset')
        if rospy.get_param('/cvision/reduce'):
            self.scale = 2.0
        else:
            self.scale = 1.0
        
    def target(self,center):
        xSp = 0.0
        ySp = 0.0
             
        if center.z > 0:
            xSp = self.scale*(center.x - self.LX/2) 
            ySp = self.scale*(self.LY/2 - center.y) - self.gripperOffset # will switch x/y
            xSp = xSp*self.m2pix
            ySp = ySp*self.m2pix
            hold = xSp                                  # switch for NED
            xSp = ySp
            ySp = hold
                
        return [xSp,ySp,center.z]                       # pass information through z-channel
                
    def targetFishEye(self,center):
        xSp = 0.0
        ySp = 0.0
        
        if center.z > 0:
            xSp = self.scale*(center.x - self.LX/2)
            ySp = self.scale*(self.LY/2 - center.y) - self.gripperOffset # will switch x/y
            radius = sqrt(xSp**2 + ySp**2)
            scale = 0.0019*radius + 0.1756              # empirical data fit
            xSp = xSp*scale                             # convert to centimeters
            ySp = ySp*scale
            xSp = xSp/100.0                             # convert to meters
            ySp = ySp/100.0
            hold = xSp                                  # switch for NED
            xSp = ySp
            ySp = hold

        return [xSp,ySp,center.z]                       # pass information through z-channel
        

###################################
#
# class getFrame
#   Use CvBridge to subscribe to image messages
#
# Subscriptions:
#   rospy.Subscriber('frameBGR', Image, self.cbBGR)
#   rospy.Subscriber('frameBGR', Image, self.cbGry)
#
# Fields:
#   callback functions
#   bridge = CvBridge() class
#   LX = image width
#   LY = image height
#   BGR = BGR image
#   Gry = Gray image
#   subBGR = subscriber
#   subGry = subscriber
#####

class getFrame():
    def __init__(self):
        self.bridge = CvBridge()
        self.LX = rospy.get_param('/cvision/LX')
        self.LY = rospy.get_param('/cvision/LY')
        self.BGR = np.zeros((self.LY,self.LX,3), np.uint8)
        self.Gry = np.zeros((self.LY,self.LX,1), np.uint8)
        self.subFrame = rospy.Subscriber('/cvision/frame', Image, self.cbFrame)

    def cbFrame(self,msg):
        if not msg == None:
            self.BGR = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.Gry = cv2.cvtColor(self.BGR, cv2.COLOR_BGR2GRAY)
            
###################################
#
# function camRotate
#   Recompute pixel measurements for a 90deg (ccw looking downward) rotated camera 
#
# Syntax:
#   (new_x,new_y) = camRotate(old_x,old_y)
#
#####

def camRotate(old_x,old_y):
    LX = rospy.get_param('/cvision/LX')
    LY = rospy.get_param('/cvision/LY')
    new_x = LX/2.0 + (old_y - LY/2.0)
    new_y = LY/2.0 - (old_x - LX/2.0)
    return new_x, new_y

###################################
#
# class xyzVar
#   Generic class to subscribe to setpoints
#
# Subscriptions:
#   
#   rospy.Subscriber('xyzTopic', Point32, self.cbXYZ)
#
# Fields:
#   x,y = target position
#   z = detection information
#
#####

class xyzVar:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = -1.0
        
    def cbXYZ(self,msg):
        if not msg == None:
            self.x = msg.x
            self.y = msg.y
            self.z = msg.z

