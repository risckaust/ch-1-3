#####
#
# autopilotParams.py
#
#####

import rospy

def setParams(ns):

    # ROS parameters for autopilot
    rospy.set_param(ns+'/autopilot/fbRate',20.0)        # feedback rate (hz)
    rospy.set_param(ns+'/autopilot/altStep',3.0)        # initial altitude step command
    rospy.set_param(ns+'/autopilot/camOffset',0.5)      # camera offset from ground

    # ROS parameters for kAltVel
    rospy.set_param(ns+'/kAltVel/gP',1.5)
    rospy.set_param(ns+'/kAltVel/gI',0.1)
    rospy.set_param(ns+'/kAltVel/vMaxU',2.0)
    rospy.set_param(ns+'/kAltVel/vMaxD',0.2)

    # ROS parameters for kBodVel
    rospy.set_param(ns+'/kBodVel/gP',1.0)
    rospy.set_param(ns+'/kBodVel/gI',0.05)
    rospy.set_param(ns+'/kBodVel/vMax',2.0)
    rospy.set_param(ns+'/kBodVel/gPyaw',0.5)           # yaw proportional gain
    rospy.set_param(ns+'/kBodVel/yawOff',1.0)         # error to turn off yaw control (m)
    rospy.set_param(ns+'/kBodVel/yawCone',45.0)        # cone to use proportional control (deg)
    rospy.set_param(ns+'/kBodVel/yawTurnRate',15.0)    # constant turn rate outside cone (deg/s)
    rospy.set_param(ns+'/kBodVel/feedForward', False)   # use EKF and feedforward estimates
