#!/usr/bin/env python
# Modules import
import rospy
import serial
import time
import sys

from autopilots.msg import StateMachine
from sensor_msgs.msg import *

class Telecom():
	def __init__(self, quadN):
		self.quadN 		= quadN
		self.ns			= '/Quad'+str(quadN)
		self.port		= '/dev/ttyUSB1'
		self.baudrate		= 57600
		self.ser		= serial.Serial()
		self.out_buf		= None
		self.in_buf		= None

		# my gps
		self.my_gps_msg	= NavSatFix()
		self.my_lat		= 0.0
		self.my_lon		= 0.0	

		# my state machine state
		self.my_sm_msg		= StateMachine()

		# resolve other quads' namespaces
		if (quadN == 1):
			self.quadA_N 	= 2
			self.quadB_N 	= 3
			self.quadA_ns 	= '/Quad2'
			self.quadB_ns 	= '/Quad3'
		elif (quadN == 2):
			self.quadA_N 	= 1
			self.quadB_N 	= 3
			self.quadA_ns 	= '/Quad1'
			self.quadB_ns 	= '/Quad3'
		elif (quadN == 3):
			self.quadA_N 	= 1
			self.quadB_N 	= 2
			self.quadA_ns 	= '/Quad1'
			self.quadB_ns 	= '/Quad2'

		# other quad A
		# gps
		self.quadA_gps_msg 	= NavSatFix()
		self.quadA_lat		= 0.0
		self.quadA_lon		= 0.0
		# state machine
		self.quadA_sm_msg 	= StateMachine()

		# other quad B
		self.quadB_gps_msg 	= NavSatFix()
		self.quadB_lat		= 0.0
		self.quadB_lon		= 0.0
		# state machine
		self.quadB_sm_msg 	= StateMachine()

		# subscribers
		# to my gps
		rospy.Subscriber(self.ns+'/mavros/global_position/raw/fix', NavSatFix, self.gps_cb)
		# to my state machine state
		rospy.Subscriber(self.ns+'/state_machine/state', StateMachine, self.sm_cb)

		# publishers
		# quadA
		self.qA_gps_pub 	= rospy.Publisher(self.quadA_ns + '/mavros/global_position/raw/fix',NavSatFix, queue_size=10)
		self.qA_state_pub 	= rospy.Publisher(self.quadA_ns + '/state_machine/state', StateMachine, queue_size=10)
		# quadB
		self.qB_gps_pub 	= rospy.Publisher(self.quadB_ns + '/mavros/global_position/raw/fix',NavSatFix, queue_size=10)
		self.qB_state_pub 	= rospy.Publisher(self.quadB_ns + '/state_machine/state', StateMachine, queue_size=10)

		# counter for transmission
		self.counter 		= 0


	# callbacks
	def gps_cb(self, msg):
		if msg is not None:
			self.my_gps_msg = msg
	
	def sm_cb(self, msg):
		if msg is not None:
			self.my_sm_msg = msg

	# open serial
	def open_serial(self):
		self.ser.timeout	= 0
		self.ser.bytesize	= serial.EIGHTBITS
		self.ser.parity		= serial.PARITY_NONE
		self.ser.stopbits	= serial.STOPBITS_ONE
		self.ser.port		= self.port
		self.ser.baudrate	= self.baudrate		
		self.ser.open()

	# encoding function
	def encode(self):
		if self.ser.isOpen():
			self.out_buf 	= 'Q'+ ',' + str(self.quadN) + ',' + 'gps' + ',' + str(self.my_gps_msg.latitude) + ',' + str(self.my_gps_msg.longitude)+ ',' + str(self.my_gps_msg.altitude)+ ',' +  'sm'+ ',' + self.my_sm_msg.state+'\n'
			# send buffer
			if self.my_gps_msg.header.seq > self.counter or self.my_sm_msg.header.seq > self.counter :
				self.ser.write(bytearray(self.out_buf))
				self.counter = max(self.my_gps_msg.header.seq, self.my_sm_msg.header.seq)
			else:
				rospy.logwarn('Nothing to write to telemetry module.')
		else:
			rospy.logwarn('Telemetry serial port is not open.')

	def decode(self):
		res = False
		if self.ser.isOpen():
			self.in_buf 	= self.ser.readline()
			parser = self.in_buf.split(',')							
			L = len(parser)
			if L == 8:
				# check if first letter is Q
				if parser[0] == 'Q':
					qn = parser[1]
					# start parsing
					# other quad A
					if qn == str(self.quadA_N):
						self.quadA_gps_msg.header.stamp = rospy.Time.now()
						self.quadA_gps_msg.latitude = float(parser[3])
						self.quadA_gps_msg.longitude = float(parser[4])
						self.quadA_gps_msg.altitude = float(parser[5])
						self.quadA_sm_msg.header.stamp = rospy.Time.now()
						self.quadA_sm_msg.state = parser[7].replace('\n','')	
						res =  True
					# other quad B
					if qn == str(self.quadB_N):
						self.quadB_gps_msg.header.stamp = rospy.Time.now()
						self.quadB_gps_msg.latitude = float(parser[3])
						self.quadB_gps_msg.longitude = float(parser[4])
						self.quadB_gps_msg.altitude = float(parser[5])
						self.quadB_sm_msg.header.stamp = rospy.Time.now()
						self.quadB_sm_msg.state = parser[7].replace('\n','')
						res =  True

		else:
			rospy.logwarn('Telemetry serial port is not open.')
		
		return res

	def test(self):
		if True:
			self.out_buf 	= 'Q'+ ',' + str(self.quadN)+',1.2323435,2,3,4,5' +'\n'
			# send buffer
			self.ser.write(bytearray(self.out_buf))


def main(arg):
	rospy.init_node('telemetry_node', anonymous=True)
	rate = rospy.Rate(50)

	if len(arg)<2:
		rospy.logerr('Insufficient input arguments')
		return

	quadN = int(arg[1])
	obj = Telecom(quadN)

	obj.port = '/dev/ttyUSB0'
	obj.baudrate = 57600

	obj.open_serial()
	while not rospy.is_shutdown():

		# encode
		obj.encode()
		# decode
		res = obj.decode()

		# publish
		if res:
			self.qA_gps_pub.publish(self.quadA_gps_msg)
			self.qA_sm_pub.publish(self.quadA_sm_msg)

			self.qB_gps_pub.publish(self.quadB_gps_msg)
			self.qB_sm_pub.publish(self.quadB_sm_msg)

		rate.sleep()


if __name__ == '__main__':
    
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
