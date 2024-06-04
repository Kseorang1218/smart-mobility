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

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])

bridge = CvBridge()

motor = None

img_ready = False

CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250 # ROI row 
ROI_HEIGHT = HEIGHT - ROI_ROW 
L_ROW = ROI_HEIGHT - 120 # Row for position detection

# class PID():
# 	def __init__(self, kp, ki, kd):
# 		self.Kp = kp
# 		self.Ki = ki
# 		self.Kd = kd
# 		self.p_error = 0.0
# 		self.i_error = 0.0
# 		self.d_error = 0.0

# 	def TotalError(self, cte):
# 		self.d_error = cte - self.p_error
# 		self.p_error = cte
# 		self.i_error += cte

# 		return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error

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

def PID(input_data, kp, ki, kd):

    global start_time, end_time, prev_error, i_error

    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = 320 - input_data
    derror = error - prev_error


    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error
    prev_error = error

    if output>50:
        output = 50
    elif output < -50:
        output = -50

    return -output

def start():
    global img_ready
    global image
    global motor
    prev_x_left, prev_x_right = 0, WIDTH

    rospy.init_node('h_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print("--------------Xycar---------------")
    rospy.sleep(5)
    # pid=PID(0.45,0.0007,0.15)

    while not image.size == (WIDTH * HEIGHT * 3):
        continue
    while not rospy.is_shutdown():
        while img_ready == False:
            continue
        img = image.copy()
        display_img = img
        img_ready = False

    # image = cv2.imread('sample.png', cv2.IMREAD_COLOR)
    # img = image.copy()
    # display_img = img
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), 30, 60)


        all_lines = cv2.HoughLinesP(edge_img, 1, math.pi/180, 30, 30, 10)
        if all_lines is None:
            continue 
        print("Number of lines : %d" % len(all_lines))
        # print(all_lines)

        line_draw_img = img.copy()
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_draw_img, (x1, y1), (x2, y2), (0,255,0), 2)
        # cv2.imshow("found lines", line_draw_img)
        # cv2.waitKey()


        roi_img = img[ROI_ROW:HEIGHT, 0:WIDTH]
        roi_edge_img = edge_img[ROI_ROW:HEIGHT, 0:WIDTH]
        # cv2.imshow("roi edge img", roi_edge_img)


        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180,50,15,10)
        if all_lines is None:
            continue
        print("After ROI, number of lines : %d" % len(all_lines))

        line_draw_img = roi_img.copy()
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,255,0), 2)
        # cv2.imshow("roi area lines", line_draw_img)
        # cv2.waitKey()

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
        print("Number of lines after slope filtering : %d" % len(filtered_lines))


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
        print("Number of left lines : %d" % len(left_lines))
        print("Number of right lines : %d" % len(right_lines))

        # right_lines in yellow_color
        line_draw_img = roi_img.copy()

        for line in left_lines:
            x1,y1, x2,y2 = line
            cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,0,255), 2)
        for line in right_lines:
            x1,y1, x2,y2 = line
            cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,255,255), 2)

        display_img[ROI_ROW:HEIGHT, 0:WIDTH] = line_draw_img
        # cv2.imshow("left & right lines", display_img)
        # cv2.waitKey()

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
                cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)

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
                cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)
                # cv2.imshow("left & right lines", line_draw_img)
                # cv2.waitKey()

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
        view_center = WIDTH//2

        print("Left/Right Lane Positions : %d %d" %(x_left, x_right))
        print("Lane Midpoint : %d" %(x_midpoint))
        print("Gap from the View_center : %d" %(x_midpoint-view_center))

        cv2.line(line_draw_img, (0,L_ROW), (WIDTH,L_ROW), (0,255,255), 2)
        cv2.rectangle(line_draw_img, (x_left-5,L_ROW-5), (x_left+5,L_ROW+5), (0,255,0), 4)
        cv2.rectangle(line_draw_img, (x_right-5,L_ROW-5), (x_right+5,L_ROW+5), (0,255,0), 4)
        cv2.rectangle(line_draw_img, (x_midpoint-5,L_ROW-5), (x_midpoint+5,L_ROW+5), (255,0,0), 4)
        cv2.rectangle(line_draw_img, (view_center-5,L_ROW-5), (view_center+5,L_ROW+5), (0,0,255), 4)

        display_img[ROI_ROW:HEIGHT, 0:WIDTH] = line_draw_img
        #cv2.imshow("Lanes positions", display_img)
        #cv2.waitKey(1)
        # error = 320 - x_midpoint
        angle = PID(x_midpoint,0.28,0.00058,0.1) # 핸들조향각 값
        speed = 6 # 차량속도 값
        drive(angle, speed)  

        if cv2.waitKey(1) & 0xFF == ord('q'):
			break         

if __name__ == '__main__':
    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()
    start()