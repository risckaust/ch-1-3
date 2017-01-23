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

from autopilots.msg import StateMachine, GripperFeedback, GripperAction

import autopilotLib
import myLib
import autopilotParams

# TODO: include the namespace in the following function
autopilotParams.setParams()

#!!!!!!!!!!!!! Need to define a message type for the state machine !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
######################################################################################################
# structure of the StateMachine.msg
#	Headaer header
#	string	state
#	string	signal

#!!!!!!!!!!!!! Need to define a message type for the Gripper Feedback !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
########################################################################################################
# structure of the GripperFeedback.msg
#	Headaer header
#	bool[]	pushbutton_state	# holds the state of each pushbutton
#	bool	picked			# indicates an object is picked or not

#!!!!!!!!!!!!! Need to define a message type for the Gripper actuation !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#########################################################################################################
# structure of the GripperAction.msg
#	Headaer header
#	bool	command	# true: activate. false: deactivate


###### State Machine Class ######
# States: {Idle, Takeoff, ObjectSearch, Picking, GoToDrop, WaitToDrop, Dropping, GoHome, Land}
# Possible Signals for each state:
#	Takeoff:	{'Done', 'Running'}
#	ObjectSearch:	{'Done', 'Running'}
#	Picking:	{'Done', 'Running', 'Failed'}
#	GoToDrop:	{'Done', 'Running'}
#	Dropping:	{'Done', 'Running'}
#	GoHome:		{'Done', 'Running'}
#	Land:		{'Done', 'Running'}
class StateMachineC( object ):
	def __init__(self, ns):
		self.namespace		= ns				# ns: namespace [REQUIRED]. Always append it to subscribed/published topics
		self.current_state	= 'Idle'			# Initially/finally, do nothing.
		self.current_signal	= None				# used to decide on transitions to other states
		self.erro_signal	= False				# to indicate error staus in state machine (for debug)
		self.error_str		= 'No error'			# Error description (for debug)
		self.DEBUG		= False				# Turn debug mode on/off

		# internal state-related fields
		self.TKOFFALT		= 2.0				# takeoff altitude [m] to be set by external controller
		self.PRE_DROP_COORDS	= np.array([23.1, 12.1])	# Lat/Lon of pre-drop location: different for each vehicle
		self.DROP_COORDS	= np.array([23.3, 12.5])	

		self.ZGROUND		= 0.0				# Altitude at ground level
		self.home		= autopilotLib.xyzVar()
		self.PICK_ALT		= 0.2				# Altitude at which we pick object [m]
		self.CAMOFFSET		= 0.1				# [m]
		self.ENVELOPE_XY_POS	= 0.2				# relative pos error where descend is allowed [m]
		self.ENVELOPE_XY_V	= 0.1				# relative vel error where descend is allowed [m/s]

		# Instantiate a setpoint topic structure
		self.setp		= PositionTarget()
		self.setp.type_mask	= int('010111000111', 2)

		# Instantiate altitude controller object (from autopilot library)
		self.altK 		= autopilotLib.kAltVel(ns)

		# Instantiate body controller object (from autopilot library)
		self.bodK 		= autopilotLib.kBodVel(ns)
    
		# Instantiate a tracker (blue)
		self.blue_target 		= autopilotLib.xyzVar()		# xyz location of object w.r.t quad [m]. z only used to indicate if object is tracked or not
		rospy.Subscriber(ns+'/blue_xySp', Point32, self.blue_target.cbXYZ)

		# Instantiate a tracker (green)
		self.green_target 		= autopilotLib.xyzVar()
		rospy.Subscriber(ns+'/green_xySp', Point32, self.green_target.cbXYZ)

		# Establish a rate
		self.fbRate 		= rospy.get_param(ns+'/autopilot/fbRate')
		self.rate 		= rospy.Rate(self.fbRate)

		# setpoint publisher (velocity to Pixhawk)
		self.command 		= rospy.Publisher(ns+'/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

		# State Machine topic & its publisher 
		self.state_topic	= StateMachine()
		self.state_pub		= rospy.Publisher(ns+'/state_machine/state', StateMachine, queue_size=10)

		# Gripper feedback topic
		self.gripper_feedback	= GripperFeedback()
		rospy.Subscriber(ns+'/gripper_feedback', GripperFeedback, self.gripper_cb)

		# Gripper command topic
		self.gripper_action	= GripperAction()
		self.gripper_pub	= rospy.Publisher(ns+'/gripper_action', GripperAction, queue_size=10)

	#----------------------------------------------------------------------------------------------------------------------------------------#
	#                                                   (States implementation)                                                              #
	
	# State: Takeoff
	def execute_takeoff(self):
		self.current_state = 'Takeoff'
		self.current_signal = 'Running'
		
		self.debug()
		
		# set the controllers setpoints
		self.altK.zSp = self.ZGROUND + rospy.get_param(self.namespace+'/autopilot/altStep')
		print self.altK.zSp
		self.home.x = self.bodK.x
		self.home.y = self.bodK.y
		# takeoff
		print abs(self.altK.zSp - self.altK.z) < 0.2
		while self.current_state == 'Takeoff' and not abs(self.altK.zSp - self.altK.z) < 0.2 and not rospy.is_shutdown():
			self.setp.header.stamp = rospy.Time.now()

			self.setp.velocity.z = self.altK.controller()
			(self.bodK.xSp, self.bodK.ySp) = autopilotLib.wayHome(self.bodK, self.home)
			(self.setp.velocity.x, self.setp.velocity.y, self.setp.yaw_rate) = self.bodK.controller()

			self.rate.sleep()
			self.command.publish(self.setp)
			# publish state topic
			self.state_topic.state = self.current_state
			self.state_topic.signal = self.current_signal
			self.state_pub.publish(self.state_topic)

			print 'State/Set/Alt/Gnd:', self.current_state, self.altK.zSp, self.altK.z, self.ZGROUND
			

		# Done with takoeff, send signal
		self.current_signal = 'Done'
		# publish state topic
		self.state_topic.state = self.current_state
		self.state_topic.signal = self.current_signal
		self.state_pub.publish(self.state_topic)
		
		self.debug()
	
		return

	######### Done with Takeof State #########

	# State: ObjectSearch
	def excecute_objectSearch(self):
		self.current_state = 'ObjectSearch'
		self.current_signal= 'Running'

		self.debug()

		objectFound = False

		# execute some trajectory, e.g. circle
		# keep checking vision feedback
		# once an object is found, exit current state
		while  not objectFound and not rospy.is_shutdown():
			# TODO executing circle trajectory for now

			# publish state topic
			self.state_topic.state = self.current_state
			self.state_topic.signal = self.current_signal
			self.state_pub.publish(self.state_topic)

			# check for objects
			objectFound, _ = self.monitorObjects()
	
		# Done with searchobject state, send signal
		self.current_signal = 'Done'
		# publish state topic
		self.state_topic.state = self.current_state
		self.state_topic.signal = self.current_signal
		self.state_pub.publish(self.state_topic)

		self.debug()

		return
	####### Done with ObjectSearch State #####

	# State: Picking
	def execute_picking(self):
		self.current_state = 'Picking'
		self.current_signal = 'Running'

		self.debug()

		# 1) track closest object, keep constant altitude
		# 2) once inside a pick envelope, descend, while tracking
		# 3) once below some LOW Alt, check pick state, keep tracking
		# 4) if low and picked, fly up, exit Picking state
		# 5) if low, and not picked, and timeout(maybe not?), fly up to search/track again goto(1)
		# !!!!!!!!!!!!!! Always make sure that you are in your assigned area!!!!!!!!!

		# temp variables
		objectFound = False
		xy=[0,0]
		picked = False

		# set altitude
		self.altK.zSp= self.ZGROUND + rospy.get_param(self.namespace+'/autopilot/altStep')
		while  self.current_signal != 'Failed' and not picked and not rospy.is_shutdown():
			objectFound, xy = self.monitorObjects()							# monitor objects
			if objectFound:										# found an object
				altCorrect = (self.altK.z - self.ZGROUND + self.CAMOFFSET)/rospy.get_param(self.namspace+'/pix2m/altCal')
				self.bodK.xSp = xy[0]*altCorrect
				self.bodK.ySp = xy[1]*altCorrect
				self.home.x = self.bodK.x       						# store most recent successful target
				self.home.y = self.bodK.y
				if self.inside_envelope(xy):
					self.altK.zSp = max(self.altK.z - 0.1*abs(self.altK.z), self.ZGROUND+self.PICK_ALT)	# descend if inside envelope
			else:											# object not found
				# TODO if at max ALT, exit state with signal='Failed'
				if (self.altK.z >= self.ZGROUND+rospy.get_param(self.namespace+'/autopilot/altStep')):	# I am at max & no objects, return with 'Failed'
					self.current_signal = 'Failed'

				(self.bodK.xSp, self.bodK.ySp) = autopilotLib.wayHome(self.bodK,self.home)	# go to last location where object was seen
				self.altK.zSp = min(self.altK.z + 0.1*abs(self.altK.z), self.ZGROUND+rospy.get_param(self.namespace+'/autopilot/altStep'))# increase altitude gradually, until an object is seen again

			# TODO !!!!!!!! constrain your location inside your zone!!!!!!

			# check if object is picked, fly up
			if (self.gripper_feedback.picked):
				picked = True	# set True to exit the while loop
				self.altK.zSp = self.ZGROUND + rospy.get_param(self.namespace+'/autopilot/altStep')
			
			self.setp.velocity.z = self.altK.controller()
			(self.setp.velocity.x, self.setp.velocity.y, self.setp.yaw_rate) = self.bodK.controller()
	
			self.rate.sleep()
			# publish setpoint to pixhawk
			self.command.publish(self.setp)
			# publish state topic
			self.state_topic.state = self.current_state
			self.state_topic.signal = self.current_signal
			self.state_pub.publish(self.state_topic)


		# Done with Picking state, send signal
		# TODO manage different signals
		self.current_signal = 'Done'
		# publish state topic
		self.state_topic.state = self.current_state
		self.state_topic.signal = self.current_signal
		self.state_pub.publish(self.state_topic)
		
		self.debug()

		return
				
					
	

	# State: GoToDrop
	def execute_gotodrop(self):
		self.current_state = 'Picking'
		self.current_signal = 'Running'

		self.debug()

		# temp variables
		arrived = False

		while not arrived and not rospy.is_shutdown():
			# TODO: convert the pre-drop location to local ENU (MAVROS expects ENU not NED)
			# (x_enu, y_enu) = self.get_enu_from_gps(self.DROP_COORDS)
			# self.home.x = x_enu
			# self.home.y = y_enu

			# TODO: implement trajectory to go to PRE_FROP
			# (self.bodK.xSp, self.bodK.ySp) = autopilotLib.wayHome(self.bodK, self.home)
			self.altK.zSp = self.ZGROUND + rospy.get_param(self.namespace+'/autopilot/altStep')

			# check if arrived
			if np.sqrt(self.bodK.xSp**2 + self.bodK.ySp*2) < 0.2:
				arrived = True	# set to True to leave the while loop

			# compute setpoints
			self.setp.velocity.z = self.altK.controller()
			(self.setp.velocity.x, self.setp.velocity.y, self.setp.yaw_rate) = self.bodK.controller()

			self.rate.sleep()
			# publish setpoints
			self.command.publish(setp)
			# publish state topic
			self.state_topic.state = self.current_state
			self.state_topic.signal = self.current_signal
			self.state_pub.publish(self.state_topic)

		# Done with GoToDrop state, send signal
		self.current_signal = 'Done'
		# publish state topic
		self.state_topic.state = self.current_state
		self.state_topic.signal = self.current_signal
		self.state_pub.publish(self.state_topic)

		self.debug()

		return

	# State: WaitToDrop
	def execute_waittodrop(self):
		self.current_state='WaitToDrop'
		self.current_sginal='Running'

		self.debug()

		# TODO: Implement coordinated dropping below

		# Done with GoToDrop state, send signal
		self.current_signal = 'Done'
		# publish state topic
		self.state_topic.state = self.current_state
		self.state_topic.signal = self.current_signal
		self.state_pub.publish(self.state_topic)

		self.debug()
	
		return

	# State: Drop
	def execute_drop(self):
		self.current_state='Dropping'
		self.current_sginal='Running'

		self.debug()

		# TODO: look for dropbox (vision-based)
		# once centerd, go to drop alt
		# deactivate magnets, and keep checking gripper feedback!


		# Done with Drop state, send signal
		self.current_signal = 'Done'
		# publish state topic
		self.state_topic.state = self.current_state
		self.state_topic.signal = self.current_signal
		self.state_pub.publish(self.state_topic)

		self.debug()

		return

	# State: GoHome
	def execute_gohome(self):
		self.current_state='GoHome'
		self.current_signal='Running'

		self.debug()

		# go to the Home XY coordinates. Landing is handled by landing state, see below.

		# Done with GoHome state, send signal
		self.current_signal = 'Done'
		# publish state topic
		self.state_topic.state = self.current_state
		self.state_topic.signal = self.current_signal
		self.state_pub.publish(self.state_topic)

		self.debug()

		return

	# State: Land
	def execute_land(self):
		self.current_state='Land'
		self.current_signal='Running'

		self.debug()

		# send land command to Pixhawk, or execute landing routine using the velocity controller

		# Done with GoHome state, send signal
		self.current_signal = 'Done'
		# publish state topic
		self.state_topic.state = self.current_state
		self.state_topic.signal = self.current_signal
		self.state_pub.publish(self.state_topic)

		self.debug()

		return

	#                                          (End of States Implementation)                                                                #
	#----------------------------------------------------------------------------------------------------------------------------------------#


	#----------------------------------------------------------------------------------------------------------------------------------------#
	#                                               (helper functions)                                                                       #

	# check if a colored object is found
	# returns a tuple: (bool objectFound, xy_coord of closest object)
	def monitorObjects(self):
		# define distance to each color object
		d_to_blue=np.inf
		d_to_green=np.inf

		# radius list: of detected objects
		r_list=[]
		# list of x/y coords of found objects
		xy_list=[]
	
		# flag if object is found
		objectFound = False

		# update the distance if (blue) is found
		if self.blue_target.z > 0 :
			d_to_blue = np.sqrt(self.blue_target.x**2 + self.blue_target.y**2)
			r_list.append(d_to_blue)
			xy_list.append([self.blue_target.x, self.blue_target.y])
			objectFound = True

		# update the distance if (green) is found
		if self.green_target.z > 0 :
			d_to_green = np.sqrt(self.green_target.x**2 + self.green_target.y**2)
			r_list.append(d_to_green)
			xy_list.append([self.green_target.x, self.green_target.y])
			objectFound = True			

		# other colors..........?

		# find the closest object
		min_d_index = r_list.index(min(r_list))
	
		# Finally return
		return (objectFound, xy_list[min_d_index])
	########## End of Monitoring Objects #######################

	# determins if an object is inside an allowable descend envelope
	def inside_envelope(self,xy):
		# currently, only based on relative pos
		if np.sqrt(xy[0]**2 + xy[1]**2) <= self.ENVELOPE_XY_POS:
			return True
		else:
			return False
	############## End of envelope function #################

	######## for debug: prints the state/signal
	def debug(self):
		if self.DEBUG:
			print '#--------------------------------------#'
			print 'State/Signal: ', self.current_state, self.current_signal
			print '#--------------------------------------#'
	############### End of debug function ##################

	#                                           (End of helper functions)                                                                    #
	#----------------------------------------------------------------------------------------------------------------------------------------#


	#----------------------------------------------------------------------------------------------------------------------------------------#
	#                                                 (Callbacks)                                                                            #

	############ Gripper callback function #####################
	def gripper_cb(self,msg):
		if msg is not None:
			self.gripper_feedback.picked = msg.picked
	########### End of Gripper callback function ##############

	#                                              (End of Callbacks)                                                                        #
	#----------------------------------------------------------------------------------------------------------------------------------------#
			
		
		
#############################################
def mission():
	rospy.init_node('mission1', anonymous=True)

	# get namespace
	ns=rospy.get_namespace()
	ns=''
	sm = StateMachineC(ns)
	sm.DEBUG=True
	sm.execute_takeoff()
	

######### Main ##################
if __name__ == '__main__':
    try:
        mission()
    except rospy.ROSInterruptException:
        pass
