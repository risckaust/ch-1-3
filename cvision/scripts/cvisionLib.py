import rospy
from math import *

###################################
#
# class pix2m 
#   Convert pixel center to distance setpoint in body NED (m)
#
# Return:
#   (xSp,ySp,flag) = position setpoint in body NED coordinates with -1/+1 flag
#
# ROS parameters:
#   /pix2m/LX = image width (pixels)
#   /pix2m/LY = image height (pixels)
#   /pix2m/m2pix = meters-per-pixel ratio
#   /pix2m/altCal = altitude of camera calibration (m) used in main loop
#
# Fields:
#   LX, LY, m2pix
#####

class pix2m():
    def __init__(self):
        self.LX = rospy.get_param('/pix2m/LX')
        self.LY = rospy.get_param('/pix2m/LY')
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
