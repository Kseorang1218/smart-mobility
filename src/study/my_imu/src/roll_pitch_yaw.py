#!/usr/bin/env python

ang_vel = 90#speed of rad.

import rospy
import time
import math

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

Imu_msg = None

def imu_callback(data):
	global Imu_msg
	Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z,
               data.orientation.w] 

rospy.init_node("Imu_Print")
rospy.Subscriber("imu", Imu, imu_callback)
curr_yaw = 0.0

while not rospy.is_shutdown():
	if Imu_msg == None:
            continue
    
        (roll, pitch, yaw) = euler_from_quaternion(Imu_msg)
	past_yaw = (yaw * 180/ math.pi)
	
	if past_yaw - curr_yaw < -360 + ang_vel:
		past_yaw = past_yaw + 360
	elif  past_yaw - curr_yaw > 360 - ang_vel:
		past_yaw = past_yaw - 360
	
	if past_yaw < curr_yaw:
		print("right")
	elif past_yaw > curr_yaw:
		print("left")
	else:
		print("straight")

	curr_yaw = yaw * 180/ math.pi

	time.sleep(1.0)

	
