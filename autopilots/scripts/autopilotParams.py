#####
#
# autopilotParams.py
#
#####

import rospy

def setParams():

    # ROS parameters for autopilot
    rospy.set_param('/autopilot/fbRate',20.0)        # feedback rate (hz)
    rospy.set_param('/autopilot/altStep',1.0)        # initial altitude step command
    rospy.set_param('/autopilot/camOffset',0.0)      # camera offset from ground

    # ROS parameters for kAltVel
    rospy.set_param('/kAltVel/gP',2.0)
    rospy.set_param('/kAltVel/gI',0.1)
    rospy.set_param('/kAltVel/vMaxU',2.0)
    rospy.set_param('/kAltVel/vMaxD',0.5)

    # ROS parameters for kBodVel
    rospy.set_param('/kBodVel/gP',1.5)
    rospy.set_param('/kBodVel/gI',0.1)
    rospy.set_param('/kBodVel/vMax',5.0)
    rospy.set_param('/kBodVel/gPyaw',0.5)           # yaw proportional gain
    rospy.set_param('/kBodVel/yawOff',1.0)         # error to turn off yaw control (m)
    rospy.set_param('/kBodVel/yawCone',45.0)        # cone to use proportional control (deg)
    rospy.set_param('/kBodVel/yawTurnRate',15.0)    # constant turn rate outside cone (deg/s)
    rospy.set_param('/kBodVel/feedForward', False)   # use EKF and feedforward estimates



 
