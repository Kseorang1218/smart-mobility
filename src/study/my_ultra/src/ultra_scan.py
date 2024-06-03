#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32MultiArray

ultra_msg = None

def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

rospy.init_node("ultra_node")
rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)

while not rospy.is_shutdown():

    if ultra_msg == None:
        continue
        
    print(ultra_msg)
    time.sleep(0.5)