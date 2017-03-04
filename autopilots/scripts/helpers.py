#!/usr/bin/env python


# Modules import
import numpy as np

from math import *



'''
#subscribe
rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_cb)
'''

''' Example
(dy_enu, dx_enu) = self.LLA_local_deltaxy(self.current_lat, self.current_lon, target_lat, target_lon)
self.home.x = self.bodK.x + dx_enu
self.home.y = self.bodK.y + dy_enu
(self.bodK.xSp, self.bodK.ySp) = autopilotLib.wayHome(self.bodK, self.home)
'''
	
######## function for converting LLA points to local delta xy(NED) :########################
	def LLA_local_deltaxy(self, lat_0, lon_0,  lat,  lon):

		M_DEG_TO_RAD = 0.01745329251994
		CONSTANTS_RADIUS_OF_EARTH	= 6371000.0
		DBL_EPSILON = 2.2204460492503131E-16

		curr_lat_rad = lat_0 * M_DEG_TO_RAD
		curr_lon_rad = lon_0 * M_DEG_TO_RAD
		curr_sin_lat = sin(curr_lat_rad)
		curr_cos_lat = cos(curr_lat_rad)

		lat_rad = lat * M_DEG_TO_RAD
		lon_rad = lon * M_DEG_TO_RAD

		sin_lat = sin(lat_rad)
		cos_lat = cos(lat_rad)

		cos_d_lon = cos(lon_rad - curr_lon_rad)

		arg = curr_sin_lat * sin_lat + curr_cos_lat * cos_lat * cos_d_lon

		if (arg > 1.0):
			arg = 1.0
		elif (arg < -1.0):
			arg = -1.0

		c = acos(arg)

		if(abs(c) < DBL_EPSILON):
			k=1
		else:
			k=c/sin(c)


		delta_x = k * (curr_cos_lat * sin_lat - curr_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH
		delta_y = k * cos_lat * sin(lon_rad - curr_lon_rad) * CONSTANTS_RADIUS_OF_EARTH
		return (delta_x,delta_y)

############### End of LLA_local_deltaxy function ##################
