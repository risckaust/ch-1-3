#!/usr/bin/env python


# Modules import 
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


###### State Machine Class ######
# States: {Idle, Takeoff, ObjectSearch, Picking, GoToDrop, Dropping, Land}
class StateMachine( object ):
	def __init__(self, ns):
		self.namespace		= ns				# ns: namespace [REQUIRED]
		self.current_state	= 'Idle'			# Initially/finally, do nothing.
		self.current_input	= None				# used to decide on transitions to other states
		self.erro_signal	= False				# to indicate error staus in state machine (for debug)
		self.error_str		= 'No error'			# Error description (for debug)

		# internal state-related fields
		self.TKOFFALT		= 2.0				# takeoff altitude [m]
		self.PRE_DROP_COORDS	= np.array([23.1, 12.1])	# Lat/Lon of pre-drop location: different for each vehicle
		self.DROP_COORDS	= np.array([23.3, 12.5])	# Lat/Lon of Drop Zone

		# Instantiate a setpoint
		self.setp		= PositionTarget()
		self.setp.type_mask	= int('010111000111', 2)

		# Instantiate altitude controller
		self.altK = autopilotLib.kAltVel(ns)

		# Instantiate body controller
		self.bodK = autopilotLib.kBodVel(ns)
    
		# Instantiate a tracker
		self.target = autopilotLib.xyzVar()
		rospy.Subscriber('blue_xySp', Point32, self.target.cbXYZ)

		# Establish a rate
		fbRate = rospy.get_param('/autopilot/fbRate')
		rate = rospy.Rate(fbRate)

	##### States implementation #######
	
	# State: Takeoff
	def execute_takeoff():
		
#############################################
def mission():
	rospy.init_node('mission1', anonymous=True)

	# get namespace
	ns=rospy.get_namespace()
	

######### Main ##################
if __name__ == '__main__':
    try:
        mission()
    except rospy.ROSInterruptException:
        pass
