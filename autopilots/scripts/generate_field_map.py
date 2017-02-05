#!/usr/bin/env python

# Modules import
import rospy
from sensor_msgs.msg import NavSatFix
import numpy as np
import os.path
import sys
import os


class MapField:
	def __init__(self):
		# contains the lat/lon pairs of the mapped field
		self.NUM_OF_FIELD_POINTS = 14
		self.field_map	= [[0.0, 0.0] for x in range(self.NUM_OF_FIELD_POINTS)]
		self.field_map_str = ['# stores the lat/lon waypoints of the mapped filed\n']
		self.field_map_str.append('field_map: [\n')
		for i in range(self.NUM_OF_FIELD_POINTS):
			self.field_map_str.append('  [0.0, 0.0],\n')
		self.field_map_str.append('  ]')
		self.current_lat= 0.0
		self.current_lon= 0.0
		# sequnce number, to make sure we are getting fresh message
		self.seq = 0

		self.fname = 'field_map.yaml'

		rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_cb)

	# GPS callback
	def gps_cb(self, msg):
		if msg is not None:
			self.seq	= msg.header.seq
			self.current_lat= msg.latitude
			self.current_lon= msg.longitude

	def get_gps_pos(self):
		
		seq=0
		while not rospy.is_shutdown():
			# get user input
			inp = raw_input('Please enter the expected index of field point between 0 and '+str(self.NUM_OF_FIELD_POINTS) + ' or q to exit:\n')

			#c_inp=ord(inp)
			if not inp.isdigit() :
				if inp == 'Q' or inp == 'q' :
					print '###### Exiting ########'
					return

			# flag to check if we ot new gps msg
			new_msg = self.seq > seq
			inp = int(inp)
			if inp >= 0 and inp <  self.NUM_OF_FIELD_POINTS :
				if new_msg :
					seq = self.seq
					self.field_map[inp]=[self.current_lat,self.current_lon]
					yes, k = self.read_file()
					if yes and k > -1:
						print 'File is read'
						self.field_map_str[k + inp+1] = '  ' + str(self.field_map[inp]) + ',\n'
						self.write_file()
						print 'Way point ', inp, ' is written.'
					else:
						self.field_map_str[1 + inp+1] = '  ' + str(self.field_map[inp]) + ',\n'
						self.write_file()
						print 'Way point ', inp, ' is written.'
				else:
					print 'Did not receive new GPS position. Try again.'
					continue
			else:
				print 'Not a valid input. Try again.'
				continue

	# Reads YAML config file that contains filed_map
	# if file is availble, it checks that it's in the write format, then load it
	# otherwise, will leave field_map at zeros
	def read_file(self):
		c = 0
		k = -1
		lines = ['' for x in range(self.NUM_OF_FIELD_POINTS+3)]
		yes = os.path.isfile(self.fname)
		if yes :
			f = open(self.fname)
			print self.fname
			try:
				print 'Reading file'
				c = 0
				k = -1
				
				for line in f:
					lines[c] = line
					# check if 'field_map' keyword is found
					if 'field_map' in line:
						k=c
					c = c+1
				print 'Length of file: ', c
			finally:
				f.close()

		# check if we read the right format
		if c == (self.NUM_OF_FIELD_POINTS+3) and (c-1 - (self.NUM_OF_FIELD_POINTS+1)) == k:
			self.field_map_str = lines

		return yes, k

	# writes field_map to a yaml config file, to be loaded in ROS
	def write_file(self):
		f = open(self.fname,'w')
		try:
			f.writelines(self.field_map_str)
		finally:
			f.close()
				

########## End of Class #######

def sub_main(arg):
	rospy.init_node('map_generator', anonymous=True)
	path=''
	if len(arg) >3 and len(arg) ==5:
		if arg[1] == '--path':
			path = arg[2]
			if os.path.isfile(path):
				print 'Map file is found'
				print 'File path: ', path
			else:
				print 'File was not found. It will be created'
	else:
		print 'Number of inputs is invalid'
		print 'Usage: generate_field_map.py --path path/to/file'
	# instantiate MapField object
	obj = MapField()
	if path:
		obj.fname = path

	obj.get_gps_pos()

       
if __name__ == '__main__':
    
    try:
        sub_main(sys.argv)
    except rospy.ROSInterruptException:
        pass
