#!/usr/bin/env python

import rospy
from msg_send.msg import my_msg


def callback(msg):
	print "Good afternoon, ",msg.last_name, msg.first_name
	

rospy.init_node('msg_receiver')

sub = rospy.Subscriber('msg_to_xycar', my_msg, callback)
pub = rospy.Publisher('msg_from_xycar',my_msg)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
	rate.sleep()
