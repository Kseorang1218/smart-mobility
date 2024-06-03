#!/usr/bin/env python

import rospy
from msg_send.msg import my_msg


def callback(msg):
	print("Good afternoon, ")
	print(msg.last_name)
	print(msg.first_name)

rospy.init_node('msg_receiver', anonymous = True)

sub = rospy.Subscriber('msg_to_xycar', my_msg, callback)

rospy.spin()
