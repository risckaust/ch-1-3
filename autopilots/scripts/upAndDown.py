#!/usr/bin/env python

import rospy
import numpy as np
import tf

from math import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import autopilotLib
import myLib
import autopilotParams
autopilotParams.setParams()

###################################

# Publishers

command = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

###################################

# Main loop

def autopilot():
    rospy.init_node('autopilot', anonymous=True)

    # Instantiate a setpoint
    setp = PositionTarget()
    setp.type_mask = int('010111000111', 2)

    # Instantiate altitude controller
    altK = autopilotLib.kAltVel()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, altK.cbPos)
    rospy.Subscriber('/mavros/state', State, altK.cbFCUstate)
    rospy.Subscriber('/mavros/extended_state', ExtendedState, altK.cbFCUexState)

    # Instantiate body controller
    bodK = autopilotLib.kBodVel()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, bodK.cbPos)
    rospy.Subscriber('/mavros/state', State, bodK.cbFCUstate)
    
    # Instantiate a tracker
    target = autopilotLib.xyzVar()
    rospy.Subscriber('target_xySp', Point32, target.cbTracker)

    # Establish a rate
    fbRate = rospy.get_param('/autopilot/fbRate')
    rate = rospy.Rate(fbRate)

    # Cycle to register local position
    kc = 0.0
    while kc < 10: # cycle for subscribers to read local position
        rate.sleep()
        kc = kc + 1

    zGround = altK.z                                # define ground level
    camOffset = rospy.get_param('/autopilot/camOffset')

    #####
    # Execute gradual altitude step response while holding current position
    #####

    altK.zSp = zGround + rospy.get_param('/autopilot/altStep')
    home = autopilotLib.xyzVar()
    home.x = bodK.x
    home.y = bodK.y

    while not abs(altK.zSp - altK.z) < 0.2 and not rospy.is_shutdown():
        
        setp.header.stamp = rospy.Time.now()

        setp.velocity.z = altK.controller()
        
        takeOffVel = 2.0*(altK.z - zGround) + 0.3

        setp.velocity.z = min(setp.velocity.z, takeOffVel)
        
        (bodK.xSp,bodK.ySp) = autopilotLib.wayHome(bodK,home)
        (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()

        rate.sleep()
        command.publish(setp)
        
        print 'Set/Alt/Gnd/takeOffVel:',altK.zSp, altK.z, zGround, takeOffVel
        
        
    #####
    # Track camera detection and land
    #####
    
    home.x = bodK.x                 # define home position
    home.y = bodK.y
    home.z = altK.z
    
    airborne = True
    
    while airborne and not rospy.is_shutdown():
    
        setp.header.stamp = rospy.Time.now()
        
        if target.z > 0:
            seeIt = True
        else:
            seeIt = False
        
        if seeIt:                   # positive detection
            altCorrect = (altK.z - zGround + camOffset)/rospy.get_param('/pix2m/altCal')
            bodK.xSp = target.x*altCorrect
            bodK.ySp = target.y*altCorrect
            home.x = bodK.x         # store most recent successful target
            home.y = bodK.y
            home.z = altK.z
            (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()
            
            error = sqrt(bodK.xSp**2 + bodK.ySp**2)
            if error < 0.25:
                setp.velocity.z = -0.1
            else:
                setp.velocity.z = 0.0
                
        else:
            (bodK.xSp,bodK.ySp) = autopilotLib.wayHome(bodK,home)
            (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()
            
            altK.zSp = home.z
            setp.velocity.z = altK.controller()

        rate.sleep()
        command.publish(setp)

        print 'bodK.xSp, bodK.ySp, seeIt: ', bodK.xSp, bodK.ySp, seeIt
        print 'airborne: ', altK.airborne
        
        if not altK.airborne:
            airborne = False
            autopilotLib.fcuModes.setDisarm()
   
        
if __name__ == '__main__':
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass





 
