#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random   
import time

# Constants
CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250 
ROI_HEIGHT = HEIGHT - ROI_ROW 
L_ROW = ROI_HEIGHT - 120 # Row for position detection

# Constants for PID
i_error = 0.0
prev_error = 0.0

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
img_ready = False

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# camera image topic callback
def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True

# publish xycar_motor msg
def drive(Angle, Speed): 
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed
    motor.publish(motor_msg)

#compute PID
def PID(input_data, kp, ki, kd):
    global start_time, end_time, prev_error, i_error
    view_center = WIDTH//2

    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = view_center - input_data
    derror = error - prev_error


    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return -output

def get_edge_img(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 30, 60)

    return edge_img

def get_lines(image, rho, theta, threshold, minLineLength, maxLineGap):
    all_lines = cv2.HoughLinesP(image, rho, theta, threshold, minLineLength, maxLineGap)

    return all_lines

def get_roi(img):
    roi_img = img[ROI_ROW:HEIGHT, 0:WIDTH]

    return roi_img

def filter_lines_by_slope(all_lines):
    # Calculate slopes and filter out lines with small absolute slope values, 
    # storing the rest in a list.
    slopes = []
    filtered_lines = []
    for line in all_lines:
        x1, y1, x2, y2 = line[0]
        if (x2 == x1):
            slope = 1000.0
        else:
            slope = float(y2-y1) / float(x2-x1)
        if 0.2 < abs(slope):
            slopes.append(slope)
            filtered_lines.append(line[0])

    return slopes, filtered_lines

def separate_lines(slopes, filtered_lines):
    left_lines = []
    right_lines = []
    
    for j in range(len(slopes)):
        Line = filtered_lines[j]
        slope = slopes[j]
        
        x1,y1, x2,y2 = Line

        if (slope < 0) and (x2 < WIDTH/2):
            left_lines.append(Line.tolist())    
        elif (slope > 0) and (x1 > WIDTH/2):
            right_lines.append(Line.tolist())

    return left_lines, right_lines

def get_left_lines(left_lines):
    m_left, b_left = 0.0, 0.0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

    size = len(left_lines)
    if size !=0:
        for line in left_lines:
            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2

            if(x2 != x1):
                m_sum += float(y2-y1)/float(x2-x1)
            else:
                m_sum += 0

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m_left = m_sum / size
        b_left = y_avg - m_left * x_avg

        if m_left != 0.0:
            x1 = int((0.0 - b_left) / m_left)
            x2 = int((ROI_HEIGHT - b_left) / m_left)

    return m_left, b_left

def get_right_lines(right_lines):
    m_right, b_right = 0.0, 0.0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
        
    size = len(right_lines)
    if size !=0:
        for line in right_lines:
            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2
            if(x2 != x1):
                m_sum += float(y2-y1)/float(x2-x1)
            else:
                m_sum += 0

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m_right = m_sum / size
        b_right = y_avg - m_right * x_avg

        if m_right != 0.0:
            x1 = int((0.0 - b_right) / m_right)
            x2 = int((ROI_HEIGHT - b_right) / m_right)

    return m_right, b_right

def start():
    global img_ready
    global image
    global motor
    global img
    prev_x_left, prev_x_right = 0, WIDTH

    # Initialize the ROS node
    rospy.init_node('h_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

     # Subscribe to the camera topic to get images
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print("--------------Xycar---------------")
    rospy.sleep(5)
    
    # Wait until the image size matches the expected size
    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    # Main loop
    while not rospy.is_shutdown():
        while img_ready == False:
            continue
        img = image.copy()
        display_img = img
        img_ready = False

        edge_img = get_edge_img(img)
        
        roi_edge_img = get_roi(edge_img)

        all_lines = get_lines(roi_edge_img, 1, math.pi/180, 50, 15, 10)
        if all_lines is None:
            continue


        slopes, filtered_lines = filter_lines_by_slope(all_lines)
        left_lines, right_lines = separate_lines(slopes, filtered_lines)

        m_left, b_left = get_left_lines(left_lines)
        m_right, b_right = get_right_lines(right_lines)

        if m_left == 0.0:
            x_left = prev_x_left
        else:
            x_left = int((L_ROW - b_left) / m_left)
            prev_x_left = x_left
        if m_right == 0.0:
            x_right = prev_x_right
        else:
            x_right = int((L_ROW - b_right) / m_right)
            prev_x_right = x_right
            
        prev_x_left = x_left
        prev_x_right = x_right
        x_midpoint = (x_left + x_right) // 2 
        
        angle = PID(x_midpoint,0.28,0.00058,0.1) # 핸들조향각 값
        speed = 6 # 차량속도 값
        drive(angle, speed)  

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break         

if __name__ == '__main__':
    start_time = time.time()
    start()
