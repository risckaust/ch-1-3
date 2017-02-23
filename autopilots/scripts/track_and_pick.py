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
		self.envelope_pos_max	= 0.6 # [m]
		self.envelope_pos_min	= 0.2 # [m]
		self.envelope_vel_min	= 0.15 # [m/s]
		self.envelope_vel_max	= 0.5 # [m/s]

		# descend_rate: fraction of the previous setpoint
		self.descend_factor_low = 0.05 #[ % ]
		self.descend_factor_high = 0.1 #[ % ]
		# altitude where descend rate is reduced, [m]
		self.LOW_ALT		= 1.0

		# ground altitude
		self.ZGROUND		= 0.0

		# Instantiate a setpoint topic structure
		self.setp		= PositionTarget()
		# use velocity setpoints
		self.setp.type_mask	= int('010111000111', 2)

		# Instantiate altitude controller object (from autopilot library)
		self.altK 		= autopilotLib.kAltVel(ns)

		# how much velocity to hold if confidenc is high+ object not seen in the current frame
		self.vHold_factor = 0.1

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

		# gripper trigger Altitude, [m]
		self.grip_trig_ALT	= 0.3

		# Gripper command topic
		# .data=True: activate magnets, .data=False: deactivate
		self.gripper_action	= Bool()
		self.gripper_pub	= rospy.Publisher(ns+'/gripper_command', Bool, queue_size=10)

		self.confidence 	= 0.0
		self.cTh 		= 0.5
		self.cRate		= 0.95

		# lidar measurements
		self.USE_LIDAR		= False
		self.OBJ_OFFSET		= 0.2
		self.lidar_z 		= 0.0
		self.lidar_active 	= False
		# lidar msg sequence
		self.lidar_seq		= 0
		# altitude at which the lidar measurement is used [m]
		self.LIDAR_USE_ALT	= 1.0
		rospy.Subscriber(ns+'/mavros/distance_sensor/hrlv_ez4_pub', Range, self.lidar_cb)

	def main(self):
		objectSeen = False
		vHold = False

		# loop for a while to get current ground altitude
		c=0
		while c < 10:
			self.rate.sleep()
			c=c+1
		self.ZGROUND = self.altK.z
		print 'Current ground altitude = ', self.ZGROUND
		print '    '
		# goodX and goodY, where object was last seen inside envelope
		good_x = self.bodK.x
		good_y = self.bodK.y
		good_z = self.ZGROUND + self.TRACK_ALT

		# initialize target/object location
		obj_x = good_x
		obj_y = good_y

		# set altitude
		self.altK.zSp = good_z

		# initialize: next descend altitude
		descend_alt = good_z

		# gripper counter, to activate only once more after picking
		gripper_counter = 0

		# picking counter
		# how many tics to pass before claiming picked
		pick_counter = 0

		# make the magnets ready
		self.gripper_action.data = True
		self.gripper_pub.publish(self.gripper_action)
		self.rate.sleep()

		bodyVmax = rospy.get_param(self.ns + '/kBodVel/vMax')

		while not rospy.is_shutdown():

			# If object is not picked
			if not self.gripperIsPicked:
				# reduce pick_counter
				pick_counter = max(pick_counter-1, 0)

				# reset gripper counter since it is not picked
				gripper_counter = 0

				# Update confidence of object detection
				if self.bgr_target.z > 0: # object is most likely seen
					self.confidence = min(self.cRate*self.confidence + (1-self.cRate)*1.0, 1)
					objectSeen = True
					# update direction towards object
					obj_x = self.bgr_target.x
					obj_y = self.bgr_target.y
				else: # object is most likely NOT seen
					self.confidence = min(self.cRate*self.confidence + (1-self.cRate)*0.0, 1)
					objectSeen = False
				
				# confidence is High
				if self.confidence > self.cTh:

					# two possibilites: 1) see it in the frame, 2) not
					if objectSeen: # confidence high + detection

						# track in xy
						if self.USE_LIDAR and self.lidar_active and (self.lidar_z <= self.LIDAR_USE_ALT):
							print '#------------Using Lidar correction-----------#'
							dz = max(self.lidar_z - self.OBJ_OFFSET, 0.0)
							altCorrect = dz/rospy.get_param(self.ns+'/pix2m/altCal')
						else:
							dz = max(self.altK.z - self.ZGROUND, 0.0)
							altCorrect = (dz + self.CAMOFFSET)/rospy.get_param(self.ns+'/pix2m/altCal')
						self.bodK.xSp = obj_x*altCorrect
						self.bodK.ySp = obj_y*altCorrect

						# update home
						self.home.x = self.bodK.x
						self.home.y = self.bodK.y

						print '#----------- Confidence = High  && Object is in frame --------------#'
						print 'Confidence: ', self.confidence
						print 'Altitude correction (meters): ', altCorrect
						print 'X2Object/Y2Object (meters): ', self.bodK.xSp, '/', self.bodK.ySp
						print '      '

						# inside envelope: track+descend
						# adjust envelope size based on height
						# current envelope is convex combination of envelope end points (defined in initialization)
						s = 0.0
						s = abs(self.altK.z/self.TRACK_ALT)
						s = min(s,1.0)
						env_pos = s*self.envelope_pos_max + (1-s)*self.envelope_pos_min
						env_vel = s*self.envelope_vel_max + (1-s)*self.envelope_vel_min
						dxy = np.sqrt(self.bodK.xSp**2 + self.bodK.ySp**2)
						dvxy = np.sqrt(self.bodK.vx**2 + self.bodK.vy**2)
						if dxy <= env_pos and dvxy <= env_vel:
							# record good position
							good_x = self.bodK.x
							good_y = self.bodK.y
							good_z = self.altK.z

							# lower descent rate if at low altitude
							if (self.altK.z - self.ZGROUND) < self.LOW_ALT:
								descend_rate = self.descend_factor_low
							else:
								descend_rate = self.descend_factor_high
							# update descend altitude only if the previous one was reached
							if abs(self.altK.z - descend_alt) < 0.1:
								descend_alt = descend_alt - descend_rate*descend_alt
								self.altK.zSp = max(descend_alt, self.ZGROUND + self.PICK_ALT)
								# TODO: should update good_z here ??
			
							print 'Object seen and Descending.....'
							print '   '

						else: # not inside envelope; keep at last good z
							self.altK.zSp = descend_alt	#TODO: good_z, or descend_alt ????
							print 'Object seen but not inside envelope.'
							print 'Keeping current altitude, tracking in xy.'
							print '    '
					else: # confidence high + miss-detection
						# TODO: implement follow startegy
						# hold last velocity
						vHold = True
						print 'Confidence high  +  miss-detection ==> holding last velocity'
						print '   '
			
					


				else: # low detection confidence: not seen
					# set the last good position
					self.home.x = good_x
					self.home.y = good_y
					#self.altK.zSp = good_z
					(self.bodK.xSp, self.bodK.ySp) = autopilotLib.wayHome(self.bodK,self.home)

					print 'X-------------- Confidence low => Not seen ----------------X'
					print 'Confidence: ', self.confidence
					print 'XY towards last good position (meters): ', self.bodK.xSp, '/', self.bodK.ySp
					print 'Going up gradually....'
					print '   '
					# go up gradually
					self.altK.zSp = min(self.altK.z + 0.1*(self.altK.z), self.ZGROUND + self.TRACK_ALT)

			
					if self.altK.z >= (self.ZGROUND + self.TRACK_ALT):
						print 'Reached Max allowed altitude..... object still considered not seen'

			else: # object is picked

				# make sure to stay for some time to confirm
				if pick_counter >= 20:
					print 'Object is considered PICKED.'
					print ' '
					self.altK.zSp = self.ZGROUND + self.TRACK_ALT
					print 'Climbing to Altitude: ', self.altK.zSp
				else:
					print 'Pick signal is received. Waiting for confirmation....'
					print ' '

				pick_counter = min(pick_counter+1, 20)

				# activate magnets once more to ensure gripping
				if gripper_counter <1:
					gripper_counter = gripper_counter+1
					self.gripper_action.data = True
					self.gripper_pub.publish(self.gripper_action)

				(self.bodK.xSp, self.bodK.ySp) = autopilotLib.wayHome(self.bodK,self.home)
				
			# update setpoint topic
			self.setp.velocity.z = self.altK.controller()
			# save last xy velocity setpoint
			last_vx = self.setp.velocity.x
			last_vy = self.setp.velocity.y
			if vHold:
				self.setp.velocity.x = last_vx*self.vHold_factor
				self.setp.velocity.x = last_vy*self.vHold_factor
				self.setp.yaw_rate = 0.0
				# reset vHold
				vHold = False
			else:
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
	def lidar_cb(self, msg):
		if msg is not None:
			if msg.header.seq > self.lidar_seq:
				self.lidar_active = True
				self.lidar_seq = self.lidar_seq + 1
			else:
				self.lidar_active = False
			if self.lidar_active:
				self.lidar_z = msg.range


#############################################
#          Mission function             #
def mission():
	rospy.init_node('tracking_and_land_node', anonymous=True)

	# get namespace
	ns=rospy.get_namespace()
	ns = ns[0:len(ns)-1]

	tr = Tracker(ns)
	tr.TRACK_ALT = 3.0
	tr.PICK_ALT = 0.2
	tr.envelope_pos_min = 0.2
	tr.envelope_pos_max = 0.6
	tr.envelope_vel_min = 0.15
	tr.envelope_vel_max = 0.5
	tr.envelope_vel = 0.15
	tr.vHold_factor = 0.1
	tr.USE_LIDAR = True

	# run the main function
	tr.main()


######### Main ##################
if __name__ == '__main__':
    try:
        mission()
    except rospy.ROSInterruptException:
        pass

		
