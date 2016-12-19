import rospy
import numpy as np
import tf
import myLib

from math import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


###################################
#
# class kAltVel
#   Altitude controller based on outer loop velocity commands to FCU
#
# Return:
#   vzRef = reference velocity to FCU (m/s, positive upward)
#
# Subscriptions:
#   rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cbPos)
#   rospy.Subscriber('/mavros/state', State, self.cbFCUstate)
#   
# ROS parameters:
#   /main/fbRate = feedback sampling rate (Hz)
#   /kAltVel/gP = proportional gain
#   /kAltVel/gI = integral gain
#   /kAltVel/vMaxU = maximum upward reference velocity (positive m/s)
#   /kAltVel/vMaxD = maximum downward reference velocity (positive m/s)
#
# Fields:
#   callback functions
#   ezInt = integrated altitude error
#   zSp = commanded altitude setpoint (m)
#   z = current altitude from /mavros/local_position/pose (m)
#   engaged = Boolean if armed and offboard
#
#####

class kAltVel:
    def __init__(self):

        self.ezInt = 0.0
        self.zSp = 0.0
        self.z = 0.0
        self.engaged = False

    def cbPos(self,msg):
        if not msg == None:
            self.z = msg.pose.position.z

    def cbFCUstate(self,msg):
        if not msg == None:
            if msg.armed and (msg.mode == 'OFFBOARD'):
                self.engaged = True
            else:
                self.engaged = False

    def controller(self):
    
        fbRate = rospy.get_param('/main/fbRate')
        gP = rospy.get_param('/kAltVel/gP')
        gI = rospy.get_param('/kAltVel/gI')
        vMaxU = rospy.get_param('/kAltVel/vMaxU')
        vMaxD = rospy.get_param('/kAltVel/vMaxD')

        ez = self.zSp - self.z                              # altitude erro

        vzRef = gP*ez + gI*self.ezInt                       # to be published

        if vzRef > vMaxU or vzRef < -vMaxD:                 # anti-windup
            vzRef = myLib.sat(vzRef,-vMaxD,vMaxU)
        else:
            if self.engaged:                                # if armed & offboard
                self.ezInt = self.ezInt + ez/fbRate         # integrate

        return vzRef

###################################
#
# class kBodVel
#   Body velocity controller based on outer loop velocity commands to FCU
#
# Return:
#   vxCom,vyCom,yawRateCom = reference commands to FCU (m/s, local ENU coordinates)
#
# Subscriptions:
#   rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cbPos)
#   rospy.Subscriber('/mavros/state', State, self.cbFCUstate)
#   
# ROS parameters:
#   /main.fbRate = feedback sampling rate (Hz)
#   /kBodVel/gP = proportional gain for velocity control
#   /kBodVel/gI = integral gain for velocity control
#   /kBodVel/vMax = maximum reference velocity (positive m/s)
#   /kBodVel/gPyaw = proportional gain for yaw control
#   /kBodVel/yawOff = radius to disable yaw control (m)
#   /kBodVel/yawCone = cone angle to disable (x,y) velocity commands (deg)
#   /kBodVel/yawTurnRate = constant yaw turn rate (deg/s)
#
# Fields:
#   callback functions
#   exInt = integrated error
#   eyInt = integrated error
#   xSp = commanded x setpoint (NED-h, m) NOTE: NED-h = NED projected to horizontal
#   ySp = commanded y setpoint (NED-h, m)
#   x = x of body frame origin in local ENU coordinates
#   y = y of body frame origin in local ENU coordinates
#   yaw = yaw angle of relative (yaw,pitch,roll) in Local ENU -> Body NED
#   engaged = Boolean if armed and offboard
#
#####

class kBodVel:
    def __init__(self):

        self.exInt = 0.0
        self.eyInt = 0.0
        self.xSp = 0.0
        self.ySp = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.engaged = False

    def cbPos(self,msg):
        if not msg == None:
            q = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(q, 'rzyx') #yaw/pitch/roll
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            self.yaw = euler[0]

    def cbFCUstate(self,msg):
        if not msg == None:
            if msg.armed and (msg.mode == 'OFFBOARD'):
                self.engaged = True
            else:
                self.engaged = False

    def controller(self):
    
        fbRate = rospy.get_param('/main/fbRate')
        gP = rospy.get_param('/kBodVel/gP')
        gI = rospy.get_param('/kBodVel/gI')
        vMax = rospy.get_param('/kBodVel/vMax')
        gPyaw = rospy.get_param('/kBodVel/gPyaw')
        yawOff = rospy.get_param('/kBodVel/yawOff')
        yawCone = rospy.get_param('/kBodVel/yawCone')
        yawTurnRate = rospy.get_param('/kBodVel/yawTurnRate')

        ######
        # longitudinal/lateral control
        ######
        
        ex = self.xSp                               # longitudinal error 
        vxRef = gP*ex + gI*self.exInt               # to be published
        
        ey = self.ySp                               # lateral error 
        vyRef = gP*ey + gI*self.eyInt               # to be published

        vel = sqrt(vxRef**2 + vyRef**2)
        if vel > vMax:                            # anti-windup        
            scale = vMax/vel
            vxRef = vxRef*scale
            vyRef = vyRef*scale
        else:
            if self.engaged:                            # if armed & offboard
                self.exInt = self.exInt + ex/fbRate     # integrate
                self.eyInt = self.eyInt + ey/fbRate

        ######
        # Convert body commands to local ENU coordinates
        ######

        bodyRot = self.yaw - pi/2.0                        # rotation of NED y-axis
        vxCom = vyRef*cos(bodyRot) - vxRef*sin(bodyRot)    # local ENU velocity commands
        vyCom = vyRef*sin(bodyRot) + vxRef*cos(bodyRot)

        ######
        # Yaw control
        ######

        dYawSp = -atan2(vyRef,vxRef)              # desired rotational change

        radius = sqrt(ex**2 + ey**2)
        if radius < yawOff:                             # no yaw control if too close
            yaw_r = 0.0
        else:
            if abs(dYawSp) > radians(yawCone):          # target out of view
                yaw_r = copysign(radians(yawTurnRate),dYawSp)   # constant rate
            else:
                yaw_r = gPyaw*dYawSp                            # proportional rate

        yawRateCom = yaw_r                              # yaw rate command

        return vxCom, vyCom, yawRateCom


###################################
#
# function wayHome
#   Vector to ENU home position expressed in NED body coordinates
#
# Syntax:
#   vec2home_x, vec2home_y = wayHome(pos,home)
#
#   vec2home.x, vec2home.y = vector to home position expressed in body NED-h coordinates
#   pos.x, pos.y, pos.yaw = origin & y-axis heading of NED body in local ENU coordinates
#   home.x, home.y = location of home in local ENU coordinates
#
#####

def wayHome(pos,home):
    dx = -(pos.x - home.x)
    dy = -(pos.y - home.y)
    
    bodyRot = pos.yaw - pi/2.0           # rotation of NED frame (ccw = positive)
    
    vec2home_x = dy*cos(bodyRot) - dx*sin(bodyRot)
    vec2home_y = dy*sin(bodyRot) + dx*cos(bodyRot)
    
    return vec2home_x, vec2home_y


###################################
#
# class spTracker
#   Class to subscribe to setpoints
#
# Subscriptions:
#   
#   rospy.Subscriber('target_xySp', Point32, self.cbTracker)
#
# Fields:
#   x,y = target position
#   z = detection flag -1/+1
#
#####

class spTracker:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
    def cbTracker(self,msg):
        if not msg == None:
            self.x = msg.x
            self.y = msg.y
            self.z = msg.z



 
