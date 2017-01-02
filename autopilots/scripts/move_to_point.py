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

# Publishers and parameters

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

    # Instantiate body controller
    bodK = autopilotLib.kBodVel()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, bodK.cbPos)
    rospy.Subscriber('/mavros/state', State, bodK.cbFCUstate)

    # Establish a rate
    fbRate = rospy.get_param('/autopilot/fbRate')
    rate = rospy.Rate(fbRate)

    # Cycle to register local position
    kc = 0.0
    while kc < 10: # cycle for subscribers to read local position
        rate.sleep()
        kc = kc + 1

    zGround = altK.z

    #####
    # Execute altitude step response while holding current position
    #####

    altK.zSp = zGround + rospy.get_param('/autopilot/altStep')
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
    # Go to point
    #####
    
    V = 4.0
    omega = 0.1
    theta = pi/2.0
    home.x = 0.0
    home.y = 0.0
    
    while not rospy.is_shutdown():
    
        setp.header.stamp = rospy.Time.now()
            
        home.x = 1.0
        home.y = 1.0 
        
        setp.velocity.z = altK.controller()
        (bodK.xSp,bodK.ySp) = autopilotLib.wayHome(bodK,home)
        (setp.velocity.x,setp.velocity.y,setp.yaw_rate) = bodK.controller()

        rate.sleep()
        command.publish(setp)
        
        error = sqrt((home.x - bodK.x)**2 + (home.y - bodK.y)**2)
        print error, setp.velocity.x, setp.velocity.y, setp.yaw_rate
        
if __name__ == '__main__':
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass





 
