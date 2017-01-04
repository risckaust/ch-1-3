import rospy
import numpy as np
from math import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

###################################
#
# class pix2m 
#   Convert pixel center to distance setpoint in body NED (m)
#
# Modules:
#   self.target(center) = position setpoint in body NED coordinates given center.x, center.y, center.z = -1/1 flag
#   self.targetFE(center) = same but for FE lens
#   (xSp,ySp,flag) = position setpoint (m) body NED coordinates with -1/+1 flag
#
# NOTE: Altitude correction done in main loop using /cvision/altCal = altitude of camera calibration (m)
#
# ROS parameters:
#   /pix2m/m2pix = meters-per-pixel ratio
#   /pix2m/altCal = calibration altitude (used in main loop)
#
# Fields:
#   LX, LY, m2pix
#   target(), targetFE()
#####

class pix2m():
    def __init__(self):
        self.LX = rospy.get_param('/cvision/LX')
        self.LY = rospy.get_param('/cvision/LY')
        self.m2pix = rospy.get_param('/pix2m/m2pix')
        
    def target(self,center):
        xSp = 0.0
        ySp = 0.0
        flag = -1
        
        if center.z > 0:
            xSp = (center.x - self.LX/2)
            ySp = (self.LY/2 - center.y)
            xSp = xSp*self.m2pix
            ySp = ySp*self.m2pix
            hold = xSp                                  # switch for NED
            xSp = ySp
            ySp = hold
            flag = 1
                
        return [xSp,ySp,flag]
                
    def targetFishEye(self,center):
        xSp = 0.0
        ySp = 0.0
        flag = -1
        
        if center.z > 0:
            xSp = (center.x - self.LX/2)
            ySp = (self.LY/2 - center.y)
            radius = sqrt(xSp**2 + ySp**2)
            scale = 0.0019*radius + 0.1756              # empirical data fit
            xSp = xSp*scale                             # convert to centimeters
            ySp = ySp*scale
            xSp = xSp/100.0                             # convert to meters
            ySp = ySp/100.0
            hold = xSp                                  # switch for NED
            xSp = ySp
            ySp = hold
            flag = 1

        return [xSp,ySp,flag]
        

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
        self.subBGR = rospy.Subscriber('frameBGR', Image, self.cbBGR)
        self.subGry = rospy.Subscriber('frameGry', Image, self.cbGry)
    
    def cbBGR(self,msg):
        if not msg == None:
            self.BGR = self.bridge.imgmsg_to_cv2(msg, "passthrough")
    
    def cbGry(self,msg):
        if not msg == None:
            self.Gry = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            
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

    

