#!/usr/bin/env python

import rospy
import numpy as np
import tf
import time
import os

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
    rospy.init_node('tuning', anonymous=True)

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
    switch = 1    

    # Initializations
    camOffset = rospy.get_param('/autopilot/camOffset')
    
    Takeoff = True
    WaypointS = False
    
    # os.system("rosrun mavros mavsys mode -c  \"OFFBOARD\" ")
    # os.system("rosrun mavros mavcmd takeoffcur 0 0 2")
        
    while not rospy.is_shutdown():

        #####
        # Take off to one meter and then to altStep
        #####
            
        if Takeoff:
        
            print "Takeoff..."
            
            rospy.set_param('/kBodVel/feedForward', False)
        
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
            Waypoints = True
            
        #####
        # Go to Waypoints
        #####
            
        if Waypoints:
        
            print "Go back and forth between waypoints..."
            
            rospy.set_param('/kBodVel/feedForward', False)

            switch = -switch
            home.x = base.x + switch*2.0
            home.y = base.y
            home.z = zGround + zHover

            # Start time count
            tStart = rospy.Time.now()
            dT = 0.0
            
            while dT < 10.0: # TODO: parameter
                
                # Issue setpoints
                sm.altK.zSp = zGround + zHover
                (sm.bodK.xSp,sm.bodK.ySp) = sm.wayHome(sm.bodK,home)    

                # Issue velocity commands
                sm.setp.header.stamp = rospy.Time.now()
                sm.setp.velocity.z = sm.altK.controller()
                (sm.setp.velocity.x,sm.setp.velocity.y,sm.setp.yaw_rate) = sm.bodK.controller()
                sm.rate.sleep()
                sm.command.publish(sm.setp)

                
                dTee = rospy.Time.now() - tStart
                dT = dTee.to_sec()
                print "Ws: ", home.x, sm.bodK.x, home.y, sm.bodK.y
        
if __name__ == '__main__':
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass





 
