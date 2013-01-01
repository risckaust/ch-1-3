#!/usr/bin/env python

# Module import
import rospy
from std_msgs.msg import *
from mavros_msgs.msg import *

class  Main():
	# constructor
	def __init__(self):
		self.drop = Bool()
		self.rc_ch7 = 1000

	# rc callback
	def rcCb(self,msg):
		if msg is not None:
			self.rc_ch7 = msg.channels[6]
			if self.rc_ch7 > 1500:
				self.drop.data = True
			else:
				self.drop.data  = False

def main():
        # initiate node
        rospy.init_node('gripper_rc_test', anonymous=True)
	g = Main()
        # subscribe to RC input
        rospy.Subscriber('mavros/rc/in', RCIn, g.rcCb)
	
	# gripper publisher
	g_pub = rospy.Publisher('gripper_command', Bool, queue_size = 10)

	# loop rate
	rate = rospy.Rate(10)

	# loop
	while not rospy.is_shutdown():
		g_pub.publish(g.drop)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
