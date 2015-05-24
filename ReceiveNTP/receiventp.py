#!/usr/bin/python

import sys

import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 55152


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



sock = socket.socket(	socket.AF_INET, 
			socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))


x = fileData()

last = 0
while True:
	data, addr = sock.recvfrom_into(x, sizeof(x)) # buffer size is 1024 bytes
	# print "received message:", data

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


