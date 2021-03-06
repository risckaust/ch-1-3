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

###################################

# Publishers and parameters

command = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

# ROS parameters for main loop
rospy.set_param('/main/fbRate',20.0)
rospy.set_param('/main/altStep',5.0)

# ROS parameters for kAltVel
rospy.set_param('/kAltVel/gP',1.5)
rospy.set_param('/kAltVel/gI',0.1)
rospy.set_param('/kAltVel/vMaxU',1.0)
rospy.set_param('/kAltVel/vMaxD',0.5)

# ROS parameters for kBodVel
rospy.set_param('/kBodVel/gP',1.5)
rospy.set_param('/kBodVel/gI',0.1)
rospy.set_param('/kBodVel/vMax',5.0)
rospy.set_param('/kBodVel/gPyaw',0.5)
rospy.set_param('/kBodVel/yawOff',1.0)
rospy.set_param('/kBodVel/yawCone',45.0)
rospy.set_param('/kBodVel/yawTurnRate',15.0)

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

    # Instantiate body controller
    bodK = autopilotLib.kBodVel()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, bodK.cbPos)
    rospy.Subscriber('/mavros/state', State, bodK.cbFCUstate)
    
    # Instantiate a tracker
    target = autopilotLib.spTracker()
    rospy.Subscriber('target_xySp', Point32, target.cbTracker)

    # Establish a rate
    fbRate = rospy.get_param('/main/fbRate')
    rate = rospy.Rate(fbRate)

    # Cycle to register local position
    kc = 0.0
    while kc < 10: # cycle for subscribers to read local position
        rate.sleep()
        kc = kc + 1

    zGround = altK.z        # define ground level

    #####
    # Execute altitude step response while holding current position
    #####

    altK.zSp = zGround + rospy.get_param('/main/altStep')
    home = myLib.xyVar()
    home.x = bodK.x
    home.y = bodK.y

    while not abs(altK.zSp - altK.z) < 0.2 and not rospy.is_shutdown():
        
        setp.header.stamp = rospy.Time.now()

        setp.velocity.z = altK.controller()
        (bodK.xSp,bodK.ySp) = autopilotLib.wayHome(bodK,home)
        (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()

        rate.sleep()
        command.publish(setp)
        
        print 'Set/Alt/Gnd:',altK.zSp, altK.z, zGround
        
        
    #####
    # Track camera detection
    #####
    
    home.x = bodK.x                 # define home position
    home.y = bodK.y
    
    while not rospy.is_shutdown():
    
        setp.header.stamp = rospy.Time.now()
        
        if target.z > 0:           # positive detection
            bodK.xSp = target.x*(altK.z - zGround)/rospy.get_param('/pix2m/altCal')
            bodK.ySp = target.y*(altK.z - zGround)/rospy.get_param('/pix2m/altCal')
            home.x = bodK.x         # store most recent successful target
            home.y = bodK.y
        else:
            (bodK.xSp,bodK.ySp) = autopilotLib.wayHome(bodK,home)
            
        setp.velocity.z = altK.controller()
        (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()

        rate.sleep()
        command.publish(setp)
        
        print bodK.xSp, bodK.ySp, target.z
        
if __name__ == '__main__':
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass





 
