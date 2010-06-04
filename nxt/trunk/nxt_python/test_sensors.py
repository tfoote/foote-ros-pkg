#!/usr/bin/env python

import roslib; roslib.load_manifest("nxt_python")

import nxt.locator
from nxt.sensor import *

def test_sensors(b):
	print 'Touch:',
	if TouchSensor(b, PORT_4).get_sample():
		print 'yes'
	else:
		print 'no'
	#print 'Sound:', SoundSensor(b, PORT_2).get_sample()
	print 'Light:', LightSensor(b, PORT_2).get_sample()
	print 'Ultrasonic:', UltrasonicSensor(b, PORT_3).get_sample()

sock = nxt.locator.find_one_brick()
if sock:
	test_sensors(sock.connect())
	sock.close()
else:
	print 'No NXT bricks found'
