#!/usr/bin/env python


# Modules import
import rospy
import numpy as np
import tf
import datetime

from math import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import autopilotLib
import myLib
import autopilotParams

class Tracker():
	def __init__(self, ns):
		autopilotParams.setParams(ns)
		self.ns			= ns
		# [meters]
		self.TRACK_ALT		= 3.0
		# camera offset [m]
		self.CAMOFFSET		= 0.1

		# ground altitude
		self.ZGROUND		= 0.0

		# Instantiate a setpoint topic structure
		self.setp		= PositionTarget()
		# use velocity setpoints
		self.setp.type_mask	= int('010111000111', 2)

		# Instantiate altitude controller object (from autopilot library)
		self.altK 		= autopilotLib.kAltVel(ns)

		# Instantiate body controller object (from autopilot library)
		self.bodK 		= autopilotLib.kBodVel(ns)

		self.home		= autopilotLib.xyzVar()

		# subscribe to BGR object
		self.bgr_target 		= autopilotLib.xyzVar()
		rospy.Subscriber(ns+'/getColors/bgr/xyMeters', Point32, self.bgr_target.cbXYZ)

		# Establish a rate
		self.fbRate 		= rospy.get_param(ns+'/autopilot/fbRate')
		self.rate 		= rospy.Rate(self.fbRate)

		# setpoint publisher (velocity to Pixhawk)
		self.command 		= rospy.Publisher(ns+'/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

	def main(self):
		objectSeen = False

		# loop for a while to get current ground altitude
		c=0
		while c < 10:
			self.rate.sleep()
			c=c+1
		self.ZGROUND = self.altK.z
		# goodX and goodY
		good_x = self.bodK.x
		good_y = self.bodK.y

		while not rospy.is_shutdown():

			# se altitude
			self.altK.zSp = self.TRACK_ALT

			# check if onject is seen
			if self.bgr_target.z > 0: # object is seen
				objectSeen = True
				good_x = self.bodK.x
				good_y = self.bodK.y

				obj_x = self.bgr_target.x
				obj_y = self.bgr_target.y

				altCorrect = (self.altK.z - self.ZGROUND + self.CAMOFFSET)/rospy.get_param(self.ns+'/pix2m/altCal')
				self.bodK.xSp = obj_x*altCorrect
				self.bodK.ySp = obj_y*altCorrect

				print '#----------- An object is seen --------------#'
				print 'Altitude correction (meters): ', altCorrect
				print 'X2Object/Y2Object (meters): ', self.bodK.xSp, '/', self.bodK.ySp
				print '      '
			else: # object not seen
				objectSeen = False
				# set the last good position
				self.home.x = good_x
				self.home.y = good_y
				(self.bodK.xSp, self.bodK.ySp) = autopilotLib.wayHome(self.bodK,self.home)

				print 'X-------------- Object is NOT seen ----------------X'
				print 'XY towards last good position (meters): ', self.bodK.xSp, '/', self.bodK.ySp
				print '   '
				
			# update setpoint topic
			self.setp.velocity.z = self.altK.controller()
			(self.setp.velocity.x, self.setp.velocity.y, self.setp.yaw_rate) = self.bodK.controller()
			self.rate.sleep()
			# publish setpoints
			self.setp.header.stamp = rospy.Time.now()
			self.command.publish(self.setp)


#############################################
#          Mission function             #
def mission():
	rospy.init_node('tracking_node', anonymous=True)

	# get namespace
	ns=rospy.get_namespace()
	ns = ns[0:len(ns)-1]

	tr = Tracker(ns)
	tr.TRACK_ALT = 3.0

	# run the main function
	tr.main()


######### Main ##################
if __name__ == '__main__':
    try:
        mission()
    except rospy.ROSInterruptException:
        pass

		
