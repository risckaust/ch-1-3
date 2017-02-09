#!/usr/bin/env python

import rospy
import numpy as np
import tf
import time

from math import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import myLib
import autopilotClass
autopilotClass.setParams()

###################################

# Main loop

def autopilot():
    rospy.init_node('autopilot', anonymous=True)

    ####
    # Attributes:
    # sm.bodK, sm.altK, sm.modes
    # sm.setp, sm.rate.
    ####    
    sm = autopilotClass.autopilotClass()
    
    # Instantiate a tracker
    target = sm.xyzVar()
    rospy.Subscriber('/getLaunchpad/launchpad/xyMeters', Point32, target.cbXYZ)
    #rospy.Subscriber('/getColors/blue/xyMeters', Point32, target.cbXYZ)
    
    # Instantiate a generic xyz positions
    home = sm.xyzVar()
    takeoff = sm.xyzVar()
    base = sm.xyzVar()
            
    # Cycle to initialize local positions
    kc = 0.0
    while kc < 10: # cycle for subscribers to read local position
        sm.rate.sleep()
        kc = kc + 1
    zGround = sm.altK.z                                # define ground level
    zHover = rospy.get_param('/autopilot/altStep')   # base hover altitude
    takeoff.x = sm.bodK.x
    takeoff.y = sm.bodK.y
    base.x = takeoff.x + 3.0
    base.y = takeoff.y + 3.0
    
    # Initializations
    camOffset = rospy.get_param('/autopilot/camOffset')

    #################################
    # MODES
    # Takeoff: Initial takeoff
    # GoToBase: Go to base location while scanning
    # Scanning: Looking for target in place
    # TrackUp: Track target while maintaining altitude
    # TrackDown: Track target while descending
    # LostEKF: Lost target but track EKF
    # LostUp: Lost target. Ascend in place.
    # Land: Landing maneuver
    #################################
    
    Takeoff = True
    GoToBase = False
    TrackUp = False
    TrackDown= False
    Landing = False
    
    cRate = 0.98
    
    while not rospy.is_shutdown():

        #####
        # Take off to one meter and then to altStep
        #####
            
        if Takeoff:
        
            print "Takeoff..."
        
            home.x = takeoff.x
            home.y = takeoff.y   
            
            for phase in range(0,2):
                if phase == 0:
                    print "Phase 1..."
                    home.z = zGround + 1.0
                else:
                    print "Phase 2..."
                    home.z = zGround + zHover
                    
                sm.altK.zSp = home.z
                
                print "z/zSp: ", sm.altK.z, sm.altK.zSp
                
                while not abs(sm.altK.zSp - sm.altK.z) < 0.2 and not rospy.is_shutdown():
                    
                    # Issue velocity commands
                    sm.setp.header.stamp = rospy.Time.now()
                    sm.setp.velocity.z = sm.altK.controller()
                    (sm.bodK.xSp,sm.bodK.ySp) = sm.wayHome(sm.bodK,home)
                    (sm.setp.velocity.x,sm.setp.velocity.y,sm.setp.yaw_rate) = sm.bodK.controller()
                    sm.rate.sleep()
                    sm.command.publish(sm.setp)
                
            Takeoff = False
            GoToBase = True
            
        #####
        # Go to base while scanning
        #####
            
        if GoToBase:
        
            print "Go to base while scanning..."

            home.x = base.x
            home.y = base.y
            home.z = zGround + zHover
            
            # Zero out EKF
            sm.bodK.ekf.xhat = np.matrix(np.zeros( (5,1) ))
            sm.bodK.ekf.P = np.matrix(np.identity(5)) # TODO: parameter
            
            confidence = 0.0
            
            while confidence < 0.7: # TODO: parameter
            
                error = sqrt((sm.bodK.x - home.x)**2 + (sm.bodK.y - home.y)**2)

                print "GoToBase:d2h/conf: ", error, confidence
            
                if target.z > 0:
                    seeIt = True
                else:
                    seeIt = False
                    
                sm.setp.header.stamp = rospy.Time.now()
                
                sm.altK.zSp = home.z
                sm.setp.velocity.z = sm.altK.controller()
                
                if seeIt: # increase confidence and hold position
                    confidence = cRate*confidence + (1-cRate)*1.0 # TODO: Parameter
                    sm.setp.velocity.x = 0.0
                    sm.setp.velocity.y = 0.0
                    sm.setp.velocity.z = 0.0
                    sm.setp.yaw_rate = 0.0
                else: # decrease confidence and go home
                    confidence = cRate*confidence
                    (sm.bodK.xSp,sm.bodK.ySp) = sm.wayHome(sm.bodK,home)
                    (sm.setp.velocity.x,sm.setp.velocity.y,sm.setp.yaw_rate) = sm.bodK.controller()

                sm.rate.sleep()
                sm.command.publish(sm.setp)
                
            GoToBase = False
            TrackUp = True

        #####
        # Track while holding altitude
        #####
         
        if TrackUp:
        
            print "Tracking..."
        
            # Initialize EKF
            sm.bodK.ekf.xhat[0] = sm.bodK.x
            sm.bodK.ekf.xhat[1] = sm.bodK.y
            sm.bodK.ekf.xhat[3] = sqrt(sm.bodK.vx**2 + sm.bodK.vy**2)

            # Start time count
            tStart = rospy.Time.now()
            dT = 0.0
            
            while confidence > 0.5 and dT < 10.0: # TODO: parameter
            
                if target.z > 0:
                    seeIt = True
                else:
                    seeIt = False
                    
                # Update EKF
                sm.bodK.ekfUpdate(seeIt)
                
                if seeIt: # Track target
                    confidence = cRate*confidence + (1-cRate)*1.0
                    altCorrect = (sm.altK.z - zGround + camOffset)/rospy.get_param('/pix2m/altCal')
                    sm.bodK.xSp = target.x*altCorrect
                    sm.bodK.ySp = target.y*altCorrect
                else: # Track EKF
                    confidence = cRate*confidence
                    home.x = sm.bodK.ekf.xhat[0]
                    home.y = sm.bodK.ekf.xhat[1]
                    (sm.bodK.xSp,sm.bodK.ySp) = sm.wayHome(sm.bodK,home)
                    
                sm.setp.header.stamp = rospy.Time.now()
                                
                sm.altK.zSp = zGround + zHover

                # Issue velocity commands
                sm.setp.velocity.z = sm.altK.controller()
                (sm.setp.velocity.x,sm.setp.velocity.y,sm.setp.yaw_rate) = sm.bodK.controller()
                sm.rate.sleep()
                sm.command.publish(sm.setp)

                
                dTee = rospy.Time.now() - tStart
                dT = dTee.to_sec()
                print "Tracking:dT/seeIt/conf: ", dT, seeIt, confidence
                print "x/xHat/y/yHat: ", sm.bodK.x, sm.bodK.ekf.xhat[0], sm.bodK.y, sm.bodK.ekf.xhat[1]

            TrackUp = False
            if confidence < 0.51:
                GoToBase = True
            else:
                TrackDown = True

        #####
        # Track while descending
        #####
        
        if TrackDown:
        
            print "Descending..."
        
            tStart = rospy.Time.now()
            dT = 0.0
            
            while confidence > 0.5 and sm.altK.z > 0.5: # TODO: parameter
            
                if target.z > 0:
                    seeIt = True
                else:
                    seeIt = False
                    
                # Update EKF
                sm.bodK.ekfUpdate(seeIt)
                
                if seeIt: # Track target
                    confidence = cRate*confidence + (1-cRate)*1.0
                    altCorrect = (sm.altK.z - zGround + camOffset)/rospy.get_param('/pix2m/altCal')
                    sm.bodK.xSp = target.x*altCorrect
                    sm.bodK.ySp = target.y*altCorrect
                else: # Track EKF
                    confidence = cRate*confidence
                    home.x = sm.bodK.ekf.xhat[0]
                    home.y = sm.bodK.ekf.xhat[1]
                    (sm.bodK.xSp,sm.bodK.ySp) = sm.wayHome(sm.bodK,home)
                    
                if seeIt: # TODO: landing logic
                    dXYZ = sqrt(sm.bodK.xSp**2 + sm.bodK.ySp**2)
                    dV = abs(sqrt(sm.bodK.vx**2 + sm.bodK.vy**2) - sm.bodK.ekf.xhat[3])
                    if dXYZ < 0.5*(1.0 + sm.altK.z) and dV < 0.5: # TODO: parameters
                        sm.setp.velocity.z = -0.5
                    else:
                        sm.setp.velocity.z = 0.0
                else:
                    sm.altK.zSp = zGround + zHover
                    sm.setp.velocity.z = sm.altK.controller()
                    dXYZ = -1.0
                    dV = -1.0
                           
                # Issue velocity commands
                sm.setp.header.stamp = rospy.Time.now()
                (sm.setp.velocity.x,sm.setp.velocity.y,sm.setp.yaw_rate) = sm.bodK.controller()
                sm.rate.sleep()
                sm.command.publish(sm.setp)
                
                print "Descending:seeIt/dXYZ/dV/conf: ", seeIt, dXYZ, dV, confidence
                print "vHat: ", sm.bodK.ekf.xhat[3]

            TrackDown = False
            if confidence < 0.51:
                GoToBase = True
            else:
                Landing = True
                
        if Landing:
            print "Landing..."
            break
                
        
if __name__ == '__main__':
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass





 
