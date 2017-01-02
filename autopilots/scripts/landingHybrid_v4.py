#!/usr/bin/env python

#####
# OUTER VELOCITY LOOPS BASED ON LOCAL VELOCITY COMMANDS
#####

import rospy
import numpy as np
import tf

#import FCUmodes

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
rospy.set_param('/hertz',25.0)

DROP_ALT = 0.0          # altitude to switch to drop mode
ALT_STEP=2.0
LANDING_TOL = 0.05       # radial tolerance to lower altitude
VEL_TOL=0.2		# x/y velocity tolerance to lower altitude, m/s
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
        self.vMaxP = 2.0
        self.vMaxN = 0.50
        self.ezInt = 0.0        # integrated altitude error

        self.RATE = rospy.get_param('/hertz')       # rate in hz

        self.zSp = 0.0             # commanded altitude setpoint
        self.vzSp = 0.0            # commanded altitude setpoint velocity
        self.vzRef = 0.0           # reference altitude velocity

        self.z = 0.0              # current altitude
        self.zRel=0.0
        self.zLocal=0.0
        self.zRanger=0.0
        self.zGround = 0.0        # ground position
        self.v = 0.0              # current velocity


        self.zRelativeFlag = True

        self.rangerIsOK=False
        self.zRangerFlag = True

        self.Engaged = False          # Boolean if Armed and Offboard

    def cbPos(self,msg):
        if not msg == None:
            self.zLocal = msg.pose.position.z

    def cbRelAlt(self, msg):
	if not msg == None :
		self.zRel = msg.data

    def cbAltRanger(self, msg):
        if not msg == None:
                self.zRanger=msg.range
                self.rangerIsOK = True
        else:
                self.rangerIsOK=False

    def cbVel(self,msg):
        if not msg == None:
            self.v = msg.twist.linear.z

    def getFCUstate(self,msg):
        if not msg == None:
            if msg.armed and (msg.mode == 'OFFBOARD'):
                self.Engaged = True
            else:
                self.Engaged = False

    def controller(self):
	
	# choose altitude measurment
        if self.zRangerFlag:
                if self.rangerIsOK:
                        self.z = self.zRanger
                else:
                        self.z=self.zRel
        else:
                self.z=self.zRel

        # Outer loop

        ez = self.zSp - self.z # altitude error

        self.vzRef = self.Kp*ez + self.Ki*self.ezInt # PI outer loop
        self.vzRef = self.vzRef + self.vzSp

        if self.vzRef > self.vMaxP: # Saturation with anti-windup
            self.vzRef = self.vMaxP
        elif self.vzRef < -self.vMaxN:
            self.vzRef = -self.vMaxN
        else:
            if self.Engaged: # only integrate if armed & offboard
                self.ezInt = self.ezInt + ez/self.RATE

        return self.vzRef

###################################
###################################
###################################

# Body controller

class K_bod:
    def __init__(self):
        self.Kp = 0.80         # controller parameters and states
        self.Ki = 0.05
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

        self.Engaged = False          # Boolean if Armed and Offboard

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
            self.yaw = euler[0] - 3.141592654/2 # correct that NED flips ENU

    def cbVel(self,msg):
        if not msg == None:
            self.xVel = msg.twist.linear.x
            self.yVel = msg.twist.linear.y

    def getFCUstate(self,msg):
        if not msg == None:
            if msg.armed and (msg.mode == 'OFFBOARD'):
                self.Engaged = True
            else:
                self.Engaged = False

    def controller(self):

        ######
        # x-axis (lateral)
        ######

        ex = self.xSp # x-axis error

        self.vxRef = self.Kp*ex + self.Ki*self.exInt # PI outer loop desired relative velocity
        self.vxRef = self.vxRef + self.vxSp


        ######
        # y-axis (longitudinal)
        ######

        ey = self.ySp # y-axis error

        self.vyRef = self.Kp*ey + self.Ki*self.eyInt # PI outer loop desired relative velocity
        self.vyRef = self.vyRef + self.vySp

        ######
        # apply saturations
        ######

        vRefMag = sqrt(self.vxRef**2 + self.vyRef**2)
        if vRefMag > self.vMax:
            self.vxRef = self.vxRef/vRefMag*self.vMax
            self.vyRef = self.vyRef/vRefMag*self.vMax
        else:
            if self.Engaged: # only integrate if armed & offboard
                self.exInt = self.exInt + ex/self.RATE
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

        if sqrt(ex**2 + ey**2) < 5:
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
        self.xHome = 0.0
        self.yHome = 0.0

    def callback(self,msg):
        if not msg == None:
            tmp_x = msg.pose.position.x - self.xHome
            tmp_y = msg.pose.position.y - self.yHome

            q = (
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z)
            yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]),1 - 2*(q[2]*q[2] + q[3]*q[3]))

            tmp_x = -tmp_x # reverse sign for way home
            tmp_y = -tmp_y
            yaw = yaw - 3.141592654/2   # correct that NED flips ENU
            tmp_yaw = -yaw              # body yaw rotation equals negative local axis rotation

            x = cos(tmp_yaw)*tmp_x - sin(tmp_yaw)*tmp_y
            y = sin(tmp_yaw)*tmp_x + cos(tmp_yaw)*tmp_y

            self.x = y
            self.y = x

###################################
###################################
###################################

# Vector target in body ENU with fisheye correction at fixed altitude
# Assumes 640 x 480 vision frame

class tracker():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.Flag = False # no measurement flag
        self.LX = 640.0
        self.LY = 480.0
        self.ALT = 1.2
        self.M2PIX = 0.5/480.0 #@1m: 0.5m = 480pix
        self.FE = True # fish eye correction
    def callback(self,msg):
        self.x = 0.0
        self.y = 0.0
        self.Flag = False
        if not msg == None:
            if msg.x > 0 and msg.y > 0:
                self.x = (msg.x - self.LX/2) # center pixel measurements
                self.y = (self.LY/2 - msg.y)
                if self.FE:
                    radius = sqrt(self.x**2 + self.y**2)
                    scale = 0.0019*radius + 0.1756 # empirical data fit
                    self.x = self.x*scale # convert to centimeters based on self.ALT data
                    self.y = self.y*scale
                    #self.x = 0.0019*self.x*abs(self.x) + 0.1756*self.x 
                    #self.y = 0.0019*self.y*abs(self.y) + 0.1756*self.y
                    self.x = self.x/100.0 # convert to meters
                    self.y = self.y/100.0
                else:
                    self.x = self.x*self.M2PIX
                    self.y = self.y*self.M2PIX
                self.Flag = True


###################################
###################################
###################################

# Main loop

def autopilot():
    rospy.init_node('autopilot', anonymous=True)

    # Instantiate a setpoint
    setp = PositionTarget()
    setp.type_mask = int('100111000111', 2)

    # Instantiate altitude controller
    altK = K_alt()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, altK.cbPos)
    rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, altK.cbVel)
    rospy.Subscriber('/mavros/state', State, altK.getFCUstate)
    rospy.Subscriber('/mavros/global_position/rel_alt', Float64, altK.cbRelAlt)
    rospy.Subscriber('/terarangerone', Range, altK.cbAltRanger)

    # Instantiate body controller
    bodK = K_bod()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, bodK.cbPos)
    rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, bodK.cbVel)
    rospy.Subscriber('/mavros/state', State, bodK.getFCUstate)


    # Instantiate way home
    myWay = wayHome()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, myWay.callback)

    # Instantiate launchpad tracker
    target = tracker()
    rospy.Subscriber('blue_xy', Point32, target.callback)

    # Establish a rate
    rate = rospy.Rate(rospy.get_param('/hertz'))

    # Cycle to register local position
    kc = 0.0
    while kc < 10: # cycle to read local position
        rate.sleep()
        kc = kc + 1

    altK.controller()	# dummy call to get altitude
    altK.zGround = altK.z

    #####
    # Execute altitude step response while holding current position
    #####

    altK.zSp = altK.zGround + ALT_STEP
    myWay.xHome = bodK.x # set new home position to be current position
    myWay.yHome = bodK.y

    while not abs(altK.zSp - altK.z) < 0.2 and not rospy.is_shutdown():
        setp.header.stamp = rospy.Time.now()

        bodK.xSp = myWay.y # SWITCHED for ENU velocity controller: Hold original position
        bodK.ySp = myWay.x

        setp.velocity.z = altK.controller()
        vCom = bodK.controller()
        setp.velocity.x = vCom[0]
        setp.velocity.y = vCom[1]
        setp.yaw = vCom[2]

        print 'Set/Alt/Gnd:',altK.zSp, altK.z, altK.zGround

        rate.sleep()
        command.publish(setp)
    
    myWay.xHome = bodK.x # Reset home position
    myWay.yHome = bodK.y
    zHome = altK.z

    #####
    # Follow vision based tracking & landing
    #####

    Dropmode = False    # Switch to Dropmode

    # Prepare to average good (x,y) measurements
    kc = 0
    goodxAvg = 0.0
    goodyAvg = 0.0

    while not rospy.is_shutdown():

        setp.header.stamp = rospy.Time.now()
        minRad = float('inf')
        
        if not Dropmode:
            # if target detected then compute body setpoint and store location. Otherwise, return to fallback

            if target.Flag == True:
                altScale = altK.z-altK.zGround
                if altScale < 0.1:
                    altScale = 0.1
                bodK.xSp = target.x*(altScale)/target.ALT # scale for altitude
                bodK.ySp = target.y*(altScale)/target.ALT # assumes zGround is ground level!!!

                radius = sqrt(bodK.xSp**2 + bodK.ySp**2)
                vMag = sqrt(bodK.xVel**2 + bodK.yVel**2)
                if altK.z < altK.zSp + 0.1 and radius < LANDING_TOL and vMag < VEL_TOL:
                    # register good (x,y) and lower altitude setpoint
                    goodxAvg = goodxAvg + (1/(kc+1))*(bodK.x - goodxAvg)
                    goodyAvg = goodyAvg + (1/(kc+1))*(bodK.y - goodyAvg)
                    kc = kc + 1
                    altK.zSp = altK.zGround + 0.97*(altK.zSp - altK.zGround)
                    # test for new fallback position    
                    rad = sqrt(bodK.xSp**2 + bodK.ySp**2)
                    if rad < minRad:
                        myWay.xHome = bodK.x
                        myWay.yHome = bodK.y
                        zHome = altK.z
                        minRad = rad
            else:                  # go to fallback position (x,y,z) with target acquisition
                bodK.xSp = myWay.y # SWITCHED for ENU velocity controller
                bodK.ySp = myWay.x
                altK.zSp = zHome

            # publish velocity setpoint or switch to position mode
            
            if altK.zSp > DROP_ALT + altK.zGround:                     # issue velocity command

                setp.velocity.z = altK.controller()
                vCom = bodK.controller()
                setp.velocity.x = vCom[0]
                setp.velocity.y = vCom[1]
                setp.yaw = vCom[2]
            else:
                Dropmode= True
                zDrop = altK.zSp
                myWay.xHome = goodxAvg
                myWay.yHome = goodyAvg

            # print status to screen
            print 'Targ:', target.x, target.y, target.Flag
            print 'Errs:', bodK.xSp, bodK.ySp
            print 'Good:', goodxAvg, bodK.x, goodyAvg, bodK.y, kc
            print 'Alti:', altK.zSp, altK.z, altK.zGround

            #Dropmode=False

        else: # Dropmode
            print 'Drop mode'

            setp.velocity.z = altK.controller()

            bodK.xSp = myWay.y # SWITCHED for ENU velocity controller: Hold original position
            bodK.ySp = myWay.x
            vCom = bodK.controller()
            setp.velocity.x = vCom[0]
            setp.velocity.y = vCom[1]
            setp.yaw = vCom[2]

            radius = sqrt((bodK.x - goodxAvg)**2 + (bodK.y - goodyAvg)**2)
            print 'radius/zDrop:', radius, zDrop
            if altK.z < zDrop + 0.03 and radius < LANDING_TOL:
                print 'computing new height..'
                zDrop = max(altK.zGround, zDrop - 0.06)
                #setp.position.z = zDrop
                altK.zSp=zDrop

            print 'Good:', goodxAvg, bodK.x, goodyAvg, bodK.y
            print 'Alti:', zDrop, altK.z, altK.zGround

        rate.sleep()
        command.publish(setp)

if __name__ == '__main__':
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass

 
