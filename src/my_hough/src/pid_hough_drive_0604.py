#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
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

PARKING_AR_ID = 4
PARKING_DISTANCE = 1.15
image = np.empty(shape=[0])

bridge = CvBridge()

motor = None

img_ready = False

CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250 # ROI row 
ROI_HEIGHT = HEIGHT - ROI_ROW 
L_ROW = ROI_HEIGHT - 120 # Row for position detection
arID = None
arData = {"DX":0.0, "DY":0.0, "DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}

# class PID():
#    def __init__(self, kp, ki, kd):
#       self.Kp = kp
#       self.Ki = ki
#       self.Kd = kd
#       self.p_error = 0.0
#       self.i_error = 0.0
#       self.d_error = 0.0
   
#    def TotalError(self, cte):
#       self.d_error = cte - self.p_error
#       self.p_error = cte
#       self.i_error += cte

#       return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error

# camera image topic callback
def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True

# AR 태그 인식 topic(/ar_pose_marker) callback
def ar_callback(msg):
    global arData, arID
    global roll, pitch, yaw 
    for i in msg.markers:

        arID = i.id
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z
        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

        (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

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

# 사용할 평균필터 클래스(한 줄 주행에 사용)
class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]

    def get_mm(self):
        return float(sum(self.data)) / len(self.data)
    
    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])

def start():
    global img_ready
    global image
    global motor
    global arID
    prev_x_left, prev_x_right = 0, WIDTH
    
    # 평균필터 사용을 위해 인스턴스 변수 선언
    prev_l_mv = MovingAverage(15)
    prev_l_mv.add_sample(0)
    prev_r_mv = MovingAverage(15)
    prev_r_mv.add_sample(640)

    # 차선 간격을 알기 위해 인스턴스 변수 추가
    prev_lr_mv = MovingAverage(15)
    prev_lr_mv.add_sample(640)

    rospy.init_node('h_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
    print("--------------Xycar---------------")
    rospy.sleep(30)
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

################# 한 줄 주행 알고리즘 시작 ###################################
        # 2024-06-04 한 줄 주행 변경 -> 차선 간격을 받아 시도 but 실패

        # get left/right line positions
        # y = m * x + b
        # x = (y - b_left) / m_left
        # x = (y - b_right) / m_right
        
        # 왼쪽 차선이 없는 경우
        if m_left == 0.0:
            # x_left = prev_x_left
            # prev_lr_mv.get_mm()은 평균 필터를 통해 얻은 차선 간격
            x_left = x_right - prev_lr_mv.get_mm()
        else:
            x_left = int((L_ROW - b_left) / m_left)
            # prev_x_left = x_left
        
        # 오른쪽 차선이 없는 경우
        if m_right == 0.0 or len(right_lines) <= 3:
            # x_right = prev_x_right
            x_right =x_left + prev_lr_mv.get_mm()
            print("오른쪽 차선이 없습니다: 평균필터 가중평균 값: %d", prev_lr_mv.get_mm())  
        else:
            x_right = int((L_ROW - b_right) / m_right)
            # prev_x_right = x_right

        prev_l_mv.add_sample(x_left)
        prev_r_mv.add_sample(x_right)

        # 차선 간격 가중평균 추가
        prev_lr_mv.add_sample(x_right - x_left)
            
        # prev_x_left = x_left
        # prev_x_right = x_right

################## 한 줄 주행 코드 끝 #####################
        x_midpoint = (x_left + x_right) // 2
        view_center = WIDTH//2
        
        print("Left/Right Lane Positions : %d %d" %(x_left, x_right))
        print("Lane Midpoint : %d" %(x_midpoint))
        print("Gap from the View_center : %d" %(x_midpoint-view_center))

        # cv2.line(line_draw_img, (0,L_ROW), (WIDTH,L_ROW), (0,255,255), 2)
        # cv2.rectangle(line_draw_img, (x_left-5,L_ROW-5), (x_left+5,L_ROW+5), (0,255,0), 4)
        # cv2.rectangle(line_draw_img, (x_right-5,L_ROW-5), (x_right+5,L_ROW+5), (0,255,0), 4)
        # cv2.rectangle(line_draw_img, (x_midpoint-5,L_ROW-5), (x_midpoint+5,L_ROW+5), (255,0,0), 4)
        # cv2.rectangle(line_draw_img, (view_center-5,L_ROW-5), (view_center+5,L_ROW+5), (0,0,255), 4)

        # display_img[ROI_ROW:HEIGHT, 0:WIDTH] = line_draw_img
        #cv2.imshow("Lanes positions", display_img)
        #cv2.waitKey(1)
        # error = 320 - x_midpoint
        angle = PID(x_midpoint,0.28,0.00058,0.1) # 핸들조향각 값
        speed = 6 # 차량속도 값
        drive(angle, speed)
        print(prev_lr_mv.get_mm())  

        # 주차 AR 태그 인식
        # TODO: 0.55도 상수 이름 지정해주기 -> 완료
        if arID == PARKING_AR_ID and arData["DZ"] < PARKING_DISTANCE:
            drive(0, 0)
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break         

if __name__ == '__main__':
    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()
    start()
