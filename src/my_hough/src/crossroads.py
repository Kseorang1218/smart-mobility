#!/usr/bin/env python

import rospy
import cv2
import math
from std_msgs.msg import String, Int64
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
import numpy as np

# 전역 변수
height = 1080
width = int(1920 / 2)
left_img = np.zeros((height, width, 3), np.uint8)
right_img = np.zeros((height, width, 3), np.uint8)
left_color = None
right_color = None
time_left = None
image = None
img_ready = False
arData = {"DX":0, "DY":0, "DZ":0, "AX":0, "AY":0, "AZ":0, "AW":0}
arID = -1
roll, pitch, yaw = 0, 0, 0

crossroads = 2

bridge = CvBridge()

# 신호등 콜백 함수
def left_callback(msg):
    global left_img, left_color
    left_img = np.zeros((height, width, 3), np.uint8)
    left_color = msg.data

    if msg.data == "R":
        left_img[:] = (0, 0, 255)
    elif msg.data == "G":
        left_img[:] = (0, 255, 0)
    elif msg.data == "Y":
        left_img[:] = (51, 250, 250)

def right_callback(msg):
    global right_img, right_color
    right_img = np.zeros((height, width, 3), np.uint8)
    right_color = msg.data

    if msg.data == "R":
        right_img[:] = (0, 0, 255)
    elif msg.data == "G":
        right_img[:] = (0, 255, 0)
    elif msg.data == "Y":
        right_img[:] = (51, 250, 250)

def time_callback(msg):
    global time_left
    time_left = msg.data

# 이미지 콜백 함수
def img_callback(data):
    global image, img_ready
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        img_ready = True
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

# AR 태그 콜백 함수
def ar_callback(msg):
    global arData, arID, roll, pitch, yaw
    for marker in msg.markers:
        arID = marker.id
        arData["DX"] = marker.pose.pose.position.x
        arData["DY"] = marker.pose.pose.position.y
        arData["DZ"] = marker.pose.pose.position.z
        arData["AX"] = marker.pose.pose.orientation.x
        arData["AY"] = marker.pose.pose.orientation.y
        arData["AZ"] = marker.pose.pose.orientation.z
        arData["AW"] = marker.pose.pose.orientation.w

        (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

def calculate_time_to_green(color, time_left):
    if color == "G":
        return 0
    elif color == "R":
        return time_left
    elif color == "Y":
        return time_left
    else:
        return float('inf')

def decide_fastest_path():
    if left_color is None or right_color is None or time_left is None:
        return None

    left_time = calculate_time_to_green(left_color, time_left)
    right_time = calculate_time_to_green(right_color, time_left)

    if left_time < right_time:
        return "left"
    else:
        return "right"

def start():
    rospy.init_node('tl_two_display')
    rospy.Subscriber("Left_color", String, left_callback)
    rospy.Subscriber("Right_color", String, right_callback)
    rospy.Subscriber("time_count", Int64, time_callback)
    rospy.Subscriber("/camera/rgb/image_raw", Image, img_callback)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_callback)

    whole_img = np.zeros((height, width * 2, 3), np.uint8)

    if arID == crossroads
        while not rospy.is_shutdown():
            if img_ready and arID != -1:
                fastest_path = decide_fastest_path()
                if fastest_path:
                    rospy.loginfo(f"The fastest path is: {fastest_path}")
                else:
                    rospy.loginfo("AR tag detected, but waiting for traffic light data...")

            whole_img = np.hstack([left_img, right_img])
            if image is not None:
                cv2.imshow('Camera View', image)
            cv2.imshow('Two traffic light', whole_img)
            if cv2.waitKey(150) & 0xFF == 27:
                break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    start()
