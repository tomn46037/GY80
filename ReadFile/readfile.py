#!/usr/bin/python

import sys

import pprint;

pp = pprint.PrettyPrinter(indent=4)



"""
struct AHRS_Data {
  
  unsigned long current_milis;
  float euler_Yaw;
  float euler_Pitch;
  float euler_Roll;
  
  float temperature;
  float pressure;
  float altitude;
  
} fileData;
"""

from ctypes import *

class fileData(LittleEndianStructure):
    _fields_ = [
				# This is the problem.. Arduino isn't doing c_ulong correctly
		('current_millis', c_uint),
		('euler_Yaw', c_float),
                ('euler_Pitch', c_float),
                ('euler_Roll', c_float),

                ('temperature', c_float),
                ('pressure', c_float),
                ('altitude', c_float)

]

with open('DATALOG.BIN', 'rb') as file:
    result = []
    x = fileData()

    # See to where we started logging
    #file.seek(0x0af0, 0)

    last = 0;
    while file.readinto(x) == sizeof(x):
        result.append((x.current_millis, x.temperature, x.pressure))

	print "================="
	print x.current_millis, x.current_millis - last
	print x.euler_Yaw
	print x.euler_Pitch
	print x.euler_Roll
	print x.temperature
	print x.pressure
	print x.altitude

	print

	last = x.current_millis


# print(result)


