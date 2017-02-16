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
		# pick altitude [m]
		self.PICK_ALT		= 0.3
		# camera offset [m]
		self.CAMOFFSET		= 0.1

		# distance from object threshold
		self.envelope		= 0.2

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

		# Gripper feedback topic
		# False: not picked, True: picked
		self.gripperIsPicked	= False
		rospy.Subscriber(ns+'/gripper_status', Bool, self.gripper_cb)

		# gripper trigger Altitiude, [m]
		self.grip_trig_ALT	= 0.3

		# picking counter
		# how many tics to pass before claiming picked
		pick_counter 		= 0

		# Gripper command topic
		# .data=True: activate magnets, .data=False: deactivate
		self.gripper_action	= Bool()
		self.gripper_pub	= rospy.Publisher(ns+'/gripper_command', Bool, queue_size=10)

		self.confidence 	= 0.0
		self.cTh 		= 0.5
		self.cRate		= 0.95

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
		good_z = self.ZGROUND + self.TRACK_ALT

		# set altitude
		self.altK.zSp = good_z

		# next descend altitude
		descend_alt = good_z

		# gripper counter, to activate only once more after picking
		gripper_counter = 0

		# make the magnets ready
		self.gripper_action = True
		self.gripper_pub.publish(self.gripper_action)
		self.rate.sleep()

		while not rospy.is_shutdown():



			if not self.gripperIsPicked:
				pick_counter = max(pick_counter-1, 0)

				# reset gripper counter
				gripper_counter = 0

				# check if onject is seen
				if self.bgr_target.z > 0: # object is seen
					self.confidence = min(self.cRate*self.confidence + (1-self.cRate)*1.0, 1)
				else:
					self.confidence = min(self.cRate*self.confidence + (1-self.cRate)*0.0, 1)
				
				# behaviour based on confidence
				if self.confidence > self.cTh:

					objectSeen = True


					obj_x = self.bgr_target.x
					obj_y = self.bgr_target.y

					altCorrect = (self.altK.z - self.ZGROUND + self.CAMOFFSET)/rospy.get_param(self.ns+'/pix2m/altCal')
					self.bodK.xSp = obj_x*altCorrect
					self.bodK.ySp = obj_y*altCorrect

					print '#----------- An object is considered seen --------------#'
					print 'Confidence: ', self.confidence
					print 'Altitude correction (meters): ', altCorrect
					print 'X2Object/Y2Object (meters): ', self.bodK.xSp, '/', self.bodK.ySp
					print '      '
			
					dxy = np.sqrt(self.bodK.xSp**2 + self.bodK.ySp**2)
					if dxy <= self.envelope :
						good_x = self.bodK.x
						good_y = self.bodK.y
						good_z = self.altK.z
						# update descend altitude only if the previous one was reached
						if abs(self.altK.z - descend_alt) < 0.1:
							descend_alt = descend_alt - 0.1*descend_alt
							self.altK.zSp = max(descend_alt, self.ZGROUND + self.PICK_ALT)
			
						print 'Object seen and Descending.....'
						print '   '

					else:
						self.altK.zSp = good_z	#TODO: or descend_alt ????
						print 'Object seen but not inside envelope. NOT descending..'
						print '    '


				else: # low confidence
					objectSeen = False
					# set the last good position
					self.home.x = good_x
					self.home.y = good_y
					#self.altK.zSp = good_z
					(self.bodK.xSp, self.bodK.ySp) = autopilotLib.wayHome(self.bodK,self.home)

					print 'X-------------- Object is considered NOT seen ----------------X'
					print 'Confidence: ', self.confidence
					print 'XY towards last good position (meters): ', self.bodK.xSp, '/', self.bodK.ySp
					print '   '

					# go up gradually
					self.altK.zSp = min(self.altK.z + 0.2*(self.altK.z), self.ZGROUND + self.TRACK_ALT)
					print 'object not seen, going up gradually.....'
					print '    '
			
					if self.altK.z >= (self.ZGROUND + self.TRACK_ALT):
						print 'Reached Max allowed altitude..... still not seeing an object'
			else: # picked
				print 'Object is considered PICKED. Climbing up..'
				print ' '

				# make sure to stay for some time to sink
				if pick_counter >= 20:
					self.altK.zSp = self.ZGROUND + self.TRACK_ALT
				pick_counter = min(pick_counter+1, 20)

				# activate magnets once more to ensure gripping
				if gripper_counter <1:
					gripper_counter = gripper_counter+1
					self.gripper_action = True
					self.gripper_pub.publish(self.gripper_action)
				
			# update setpoint topic
			self.setp.velocity.z = self.altK.controller()
			(self.setp.velocity.x, self.setp.velocity.y, self.setp.yaw_rate) = self.bodK.controller()
			self.rate.sleep()
			# publish setpoints
			self.setp.header.stamp = rospy.Time.now()
			self.command.publish(self.setp)
	############ Gripper callback function #####################
	def gripper_cb(self,msg):
		if msg is not None:
			self.gripperIsPicked = msg.data
	########### End of Gripper callback function ##############


#############################################
#          Mission function             #
def mission():
	rospy.init_node('tracking_and_land_node', anonymous=True)

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

		
