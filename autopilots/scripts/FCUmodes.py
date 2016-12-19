import rospy
from mavros_msgs.srv import *

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
        print "Service disarming call failed: %s"%e

def setStabilizedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='STABILIZED')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

def setOffboardMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='OFFBOARD')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Offboard Mode could not be set."%e

def setAltitudeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='ALTCTL')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Altitude Mode could not be set."%e

def setPositionMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='POSCTL')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Position Mode could not be set."%e

def setAutoLandMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='AUTO.LAND')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Autoland Mode could not be set."%e


