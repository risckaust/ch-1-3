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

# Main loop

def autopilot():
    rospy.init_node('autopilot', anonymous=True)

    # Publishers
    command = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    # Instantiate a setpoint
    setp = PositionTarget()
    setp.type_mask = int('010111000111', 2)

    # Instantiate altitude controller
    altK = autopilotLib.kAltVel()

    # Instantiate body controller
    bodK = autopilotLib.kBodVel()
    
    # Instantiate a tracker
    target = autopilotLib.xyzVar()
    rospy.Subscriber('/getLaunchpad/launchpad/xyMeters', Point32, target.cbXYZ)
    #rospy.Subscriber('/getColors/blue/xyMeters', Point32, target.cbXYZ)
    
    # Instantiate a mode switcher
    modes = autopilotLib.fcuModes()

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
        
        (bodK.xSp,bodK.ySp) = autopilotLib.wayHome(bodK,home)
        (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()

        rate.sleep()
        command.publish(setp)
        
        print 'Set/Alt/zDot/Gnd:',altK.zSp, altK.z, altK.vz, zGround
        
        
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
            
            bodK.ekfUpdate(True)
            
            # Construct velocity estimates
            thHat = bodK.ekf.xhat[2]
            vHat =  bodK.ekf.xhat[3]
            wHat = bodK.ekf.xhat[4]
            VX = vHat*cos(thHat)      # ENU coordinates                  
            VY = vHat*sin(thHat)
            
            home.x = bodK.x         # store most recent successful target
            home.y = bodK.y
            home.z = altK.z
            (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()
            
            error = sqrt(bodK.xSp**2 + bodK.ySp**2)
            errorVelocity = sqrt((bodK.vx - VX)**2 + (bodK.vy - VY)**2)
            
            if error < 0.25 and errorVelocity < 0.5:
                setp.velocity.z = -0.3
            else:
                altK.zSp = altK.z
                setp.velocity.z = 0*altK.controller()

            if False: # altK.z < zGround + 0.1:
                altK.zSp = zGround + 0.1
                setp.velocity.z = altK.controller()
                    
        else:
            bodK.ekfUpdate(False)
            
            # Here, enter to follow EKF estimates
            
            (bodK.xSp,bodK.ySp) = autopilotLib.wayHome(bodK,home)
            (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()
            
            altK.zSp = home.z
            setp.velocity.z = altK.controller()

            error = -1.0

        rate.sleep()
        command.publish(setp)

        print 'xSp/ySp/error/seeIt: ', bodK.xSp, bodK.ySp, error, seeIt
        print 'z, airborne: ', altK.z, altK.airborne
        
        if not altK.airborne:
            airborne = False
            # modes.setDisarm()
        
if __name__ == '__main__':
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass





 
