#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
import numpy as np

height=1080
width=1920
left_img=np.zeros((height,width,3), np.uint8)
right_img=np.zeros((height,width,3), np.uint8)

def left_callback(left_color):
    global left_img
    left_img = np.zeros((height,width,3), np.uint8)

    if left_color.data == "R":
        left_img[:]=(0,0,255)
    elif left_color.data == "G":
        left_img[:]=(0,255,0)
    elif left_color.data == "Y":
        left_img[:]=(51,250,250)


def start():
    rospy.init_node('tl_one_display')
    rospy.Subscriber("Single_color", String, left_callback)
    
    whole_img=np.zeros((height,width,3), np.uint8)
    # r= rospy.Rate(0.5)
    while not rospy.is_shutdown():
        whole_img = left_img
        cv2.imshow('Single traffic light', whole_img)
        # r.sleep()
        if cv2.waitKey(150) & 0xFF == 27:
            cv2.destroyAllWindows()
            break

if __name__ == '__main__':
    start()
