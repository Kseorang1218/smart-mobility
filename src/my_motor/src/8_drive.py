#!/usr/bin/env python

import rospy
import time
from xycar_msgs.msg import xycar_motor

motor_control = xycar_motor()

rospy.init_node('auto_driver')

pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)

def motor_pub(angle, speed):
	global pub
	global motor_control

	motor_control.angle = angle
	motor_control.speed = speed

	pub.publish(motor_control)

speed = 2

while not rospy.is_shutdown():
	angle = 45
	for i in range(10):
		motor_pub(angle, speed)
		time.sleep(2)

	angle = -45
	for i in range(10):
		motor_pub(angle, speed)
		time.sleep(4)

	#angle = 0
	#for i in range(30):
	#	motor_pub(angle, speed)
	#	time.sleep(0.1)

	#angle = 50
	#for i in range(40):
	#	motor_pub(angle, speed)
	#	time.sleep(0.1)

	#angle = 0
	#for i in range(30):
	#	motor_pub(angle, speed)
	#	time.sleep(0.1)

