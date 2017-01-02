#!/usr/bin/env python

#####
# OUTER VELOCITY LOOPS BASED ON LOCAL VELOCITY COMMANDS
#####

import rospy
import numpy as np
import os
import tf

from math import *

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

###################################
###################################
###################################

# Publishers and globals

command = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
rospy.set_param('/hertz',20.0) # cannot be faster than pose publish rate

###################################
###################################
###################################

# FCU mode selection

def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arming call failed: %s"%e

def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arming call failed: %s"%e

def setOffboardMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='OFFBOARD')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Override Mode could not be set."%e

def setAltitudeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='ALTCTL')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Altitude Mode could not be set."%e

def setAutoLandMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='AUTO.LAND')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Autoland Mode could not be set."%e

###################################
###################################
###################################

# preflight tasks

def preflight():
    # Arm FCU
    setArm()
    rospy.sleep(1.0)

    # Instantiate a setpoint
    setp = PositionTarget()
    setp.type_mask = int('100111000111', 2)

    # Preload setpoints
    rate = rospy.get_param('/hertz')
    rate = rospy.Rate(rate)
    kc = 0
    while kc < 10:
        setp.header.seq = kc
        setp.header.stamp = rospy.Time.now()
        kc = kc+1
        rate.sleep()
        command.publish(setp)

    # Set Offboard mode
    setOffboardMode()
    rospy.sleep(0.1)


###################################
###################################
###################################

# Custom math functions

def sat(x,X):
    z = x
    if x > X:
        z = X
    elif x < -X:
        z = -X
    return z

def dead(x,X):
    z = x - sat(x,X)
    return z

###################################
###################################
###################################

# Altitude controller

class K_alt:
    def __init__(self):
        self.Kp = 1.2           # controller parameters & states
        self.Ki = 0.0
        self.vMax = 2.0
        self.ezInt = 0          # integrated altitude error

        self.RATE = rospy.get_param('/hertz')       # rate in hz

        self.zSp = 0            # commanded altitude setpoint
        self.vzSp = 0            # commanded altitude setpoint velocity
        self.vzRef = 0           # reference altitude velocity

        self.z = 0              # current altitude
        self.v = 0              # current velocity

    def cbPos(self,msg):
        if not msg == None:
            self.z = msg.pose.position.z

    def cbVel(self,msg):
        if not msg == None:
            self.v = msg.twist.linear.z

    def controller(self):

        # Outer loop

        ez = self.zSp - self.z # altitude error

        self.vzRef = self.Kp*ez + self.Ki*self.ezInt # PI outer loop
        self.vzRef = self.vzRef + self.vzSp

        if self.vzRef > self.vMax: # Saturation with anti-windup
            self.vzRef = self.vMax
        elif self.vzRef < -self.vMax:
            self.vzRef = -self.vMax
        else:
            self.ezInt = self.ezInt + ez/self.RATE

        return self.vzRef

###################################
###################################
###################################

# Body controller

class K_bod:
    def __init__(self):
        self.Kp = 0.8          # controller parameters and states
        self.Ki = 0.1
        self.vMax = 2.0

        self.RATE = rospy.get_param('/hertz')       # rate in hz

        self.exInt = 0.0        # ENU *body* coordinates
        self.eyInt = 0.0        #          

        self.xSp = 0.0                # commanded x setpoint
        self.vxSp = 0.0               # commanded x setpoint velocity
        self.vxRef = 0.0              # reference x velocity
        self.ySp = 0.0                # commanded y setpoint
        self.vySp = 0.0               # commanded y setpoint velocity
        self.vyRef = 0.0              # reference y velocity
 
        self.x = 0.0                  # (x,y) of body frame wrt local frame ENU
        self.y = 0.0
        self.yaw = 0.0                # yaw of body frame wrt local frame

        self.vxCom = 0.0              # reference x velocity in local frame
        self.vyCom = 0.0              # reference y velocity in local frame
        self.yawCom = 0.0             # yaw command in local frame
        self.vBod = [0, 0, 0]

        tc = 0.5
        self.gYaw = 1/tc               # Controller gains for yaw

    def cbPos(self,msg):
        if not msg == None:
            q = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(q, 'rzyx')
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            self.yaw = euler[0] - 3.141592654/2 #####CORRECTION#####

    def cbVel(self,msg):
        if not msg == None:
            self.xVel = msg.twist.linear.x
            self.yVel = msg.twist.linear.y

    def controller(self):

        ######
        # x-axis (lateral)
        ######
        ex = self.xSp # x-axis error

        self.vxRef = self.Kp*ex + self.Ki*self.exInt # PI outer loop desired relative velocity
        self.vxRef = self.vxRef + self.vxSp

        if self.vxRef > self.vMax: # Saturation with anti-windup
            self.vxRef = self.vMax
        elif self.vxRef < -self.vMax:
            self.vxRef = -self.vMax
        else:
            self.exInt = self.exInt + ex/self.RATE

        ######
        # y-axis (longitudinal)
        ######

        ey = self.ySp # y-axis error

        self.vyRef = self.Kp*ey + self.Ki*self.eyInt # PI outer loop desired relative velocity
        self.vyRef = self.vyRef + self.vySp

        if self.vyRef > self.vMax: # Saturation with anti-windup
            self.vyRef = self.vMax
        elif self.vyRef < -self.vMax:
            self.vyRef = -self.vMax
        else:
            self.eyInt = self.eyInt + ey/self.RATE

        ######
        # Convert body commands to local ENU coordinates
        ######

        self.vxCom = self.vxRef*cos(self.yaw) - self.vyRef*sin(self.yaw)
        self.vyCom = self.vxRef*sin(self.yaw) + self.vyRef*cos(self.yaw)

        ######
        # Yaw control
        ######

        yawLoc = -atan2(self.xSp,self.ySp) # express local yaw as error to zero

        if abs(ey) < 5 and abs(ex) < 5:
            yaw_r = 0.0
        else:
            if abs(yawLoc) > 45*3.14/180:
                yaw_r = copysign(15.0,yawLoc)
                self.vxCom = self.vxCom/10
                self.vyCom = self.vyCom/10
            else:
                yaw_r = self.gYaw*yawLoc
            
        self.yawCom = self.yaw + yaw_r/self.RATE

        self.vBod = [self.vxCom, self.vyCom, self.yawCom]

        return self.vBod


###################################
###################################
###################################

# Vector to home in body coordinates NED (not ENU)

class wayHome:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.xHome = 0.0
        self.yHome = 0.0
        self.yaw = 0.0

    def callback(self,msg):
        if not msg == None:
            tmp_x = msg.pose.position.x - self.xHome
            tmp_y = msg.pose.position.y - self.yHome
            self.z = msg.pose.position.z
            q = (
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z)
            yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]),1 - 2*(q[2]*q[2] + q[3]*q[3]))

            tmp_x = -tmp_x # reverse sign for way home
            tmp_y = -tmp_y
            yaw = yaw - 3.141592654/2   # correct that NED flips ENU
            tmp_yaw = -yaw              # body yaw rotation equals negative local rotation

            x = cos(tmp_yaw)*tmp_x - sin(tmp_yaw)*tmp_y
            y = sin(tmp_yaw)*tmp_x + cos(tmp_yaw)*tmp_y

            self.x = y
            self.y = x
            self.yaw = -yaw # yaw in local coordinates

###################################
###################################
###################################

# Vector target in body ENU

class trackerSTD():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.Flag = False
        self.LX = 640.0
        self.LY = 480.0
        self.M2PIX = 0.5/480.0 # @1m: 0.5m = 480pix
    def callback(self,msg):
        if not msg == None:
            if msg.x > 0 and msg.y > 0:
                self.x = (msg.x - self.LX/2)*self.M2PIX
                self.y = (self.LY/2 - msg.y)*self.M2PIX
                self.Flag = True
            else:
                self.x = 0.0
                self.y = 0.0
                self.Flag = False

###################################
###################################
###################################

# Vector target in body ENU

class tracker():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.Flag = False
        self.LX = 640.0
        self.LY = 480.0
        self.ALT = 1.2
    def callback(self,msg):
        if not msg == None:
            if msg.x > 0 and msg.y > 0:
                self.x = (msg.x - self.LX/2) # center pixel measurements
                self.y = (self.LY/2 - msg.y)
                self.x = 0.0019*self.x*abs(self.x) + 0.1756*self.x # convert to centimeters based on self.ALT data
                self.y = 0.0019*self.y*abs(self.y) + 0.1756*self.y
                self.x = self.x/100.0 # convert to meters
                self.y = self.y/100.0
                self.Flag = True
            else:
                self.x = 0.0
                self.y = 0.0
                self.Flag = False
        else:
            self.Flag = False

###################################
###################################
###################################

# Main loop

def autopilot():
    rospy.init_node('autopilot', anonymous=True)

    # Instantiate a setpoint
    setp = PositionTarget()
    setp.type_mask = int('100111000111', 2)

    # Preflight tasks
    # preflight()

    # Instantiate altitude controller
    altK = K_alt()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, altK.cbPos)
    rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, altK.cbVel)

    # Instantiate body controller
    bodK = K_bod()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, bodK.cbPos)
    rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, bodK.cbVel)

    # Instantiate way home
    myWay = wayHome()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, myWay.callback)

    # Instantiate blue tracker
    target = tracker()
    rospy.Subscriber('blue_xy', Point32, target.callback)

    # Establish a rate
    rate = rospy.Rate(rospy.get_param('/hertz'))

    # Cycle to register local position
    kc = 0.0
    while kc < 10: # cycle to read local position
        print altK.z, bodK.x, bodK.y
        rate.sleep()
        kc = kc + 1

    # Execute altitude step response while holding current position
    altK.zSp = altK.z + 5.0 # increase 1.5m
    myWay.xHome = bodK.x # set new home position to be current position
    myWay.yHome = bodK.y

    while altK.z < altK.zSp  and not rospy.is_shutdown():
        setp.header.stamp = rospy.Time.now()

        bodK.xSp = myWay.y # SWITCHED for ENU velocity controller: Hold original position
        bodK.ySp = myWay.x

        setp.velocity.z = altK.controller()
        vCom = bodK.controller()
        setp.velocity.x = vCom[0]
        setp.velocity.y = vCom[1]
        setp.yaw = vCom[2]

        print altK.zSp, altK.z

        rate.sleep()
        command.publish(setp)
    
    myWay.xHome = bodK.x # Reset home position
    myWay.yHome = bodK.y

    while not rospy.is_shutdown():

        setp.header.stamp = rospy.Time.now()

        if target.Flag == True:
            bodK.xSp = target.x*altK.z/target.ALT # scale for altitude
            bodK.ySp = target.y*altK.z/target.ALT
            myWay.xHome = bodK.x # set new home position to be current position of last target aquisition
            myWay.yHome = bodK.y
        else:
            bodK.xSp = myWay.y # SWITCHED for ENU velocity controller
            bodK.ySp = myWay.x

        setp.velocity.z = altK.controller()
        vCom = bodK.controller()
        setp.velocity.x = vCom[0]
        setp.velocity.y = vCom[1]
        setp.yaw = vCom[2]

        print 'Targ:', target.x, target.y, target.Flag
        print 'Errs:', bodK.xSp, bodK.ySp
	print "Altitude Err: ", abs(altK.z-altK.zSp)
        rate.sleep()
        command.publish(setp)

    #setAltitudeMode()

if __name__ == '__main__':
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass





 
