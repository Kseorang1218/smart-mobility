#!/usr/bin/env python
# -*- coding: utf-8 -*-

#########################################################################
# 스모빌 자이카 경로 주행 메인 코드
# 제작 날짜: 2024-05-28
# 팀 이름: 김동이랑 팀
# 추가구현 필요
#########################################################################

from pickle import TRUE
import sys
import os
import signal

import rospy, math, random
import time
import numpy as np
import cv2

#from hough_drive.py import process_image , Width, Height, Offset, Gap
from cv_bridge import CvBridge
from std_msgs.msg import String, Int64
from sensor_msgs.msg import LaserScan, Image
from xycar_motor.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


image = np.empty(shape=[0])
bridge = CvBridge()
Width = 640
Height = 480
Offset = 340   # TODO: 자이카에 맞게 수정 필요
Gap = 40

defaultSpeed = 0

lidar_distance = []
motor_msg = xycar_motor()

image = np.empty(shape=[0])   # 카메라 이미지 저장 공간
bridge = CvBridge()   # OpenCV 영상처리에 필요

# h_drive_ex.py 변수
motor = None   # 모터 노드 처리에 필요
img_ready = False

trafficLeft = ''
trafficRight = ''
trafficTime = ''
ardict = {"ID":0, "DX":0.0, "DY":0.0, "DZ":0.0 }
arData = [ardict,ardict,ardict]
AR_marker_detected = False

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#  h_drive_ex.py -> publish xycar_motor msg
def drive(Angle, Speed): 
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed
    motor.publish(motor_msg)

def lidar_callback(data):
    global lidar_distance
    lidar_distance = data.ranges

def trafficLeft_callback(data):
    global trafficLeft
    trafficLeft = data.data
def trafficRight_callback(data):
    global trafficRight
    trafficRight = data.data
def trafficSingle_callback(data):
    global trafficSingle
    trafficSingle = data.data
def trafficTime_callback(data):
    global trafficTime
    trafficTime = data.data

def AR_callback(msg):
    global arData, AR_marker_detected

    for i in msg.markers:
        
        id = i.id
        if(id == 2):
            idx = 0
        elif(id == 4):
            idx = 1
        elif(id == 6):
            idx =2
        else :
            return
        AR_marker_detected = True
        arData[idx]["ID"] = i.id
        arData[idx]["DX"] = i.pose.pose.position.x
        arData[idx]["DY"] = i.pose.pose.position.y
        arData[idx]["DZ"] = i.pose.pose.position.z


def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

Subs = []
Subs.append(rospy.Subscriber("/usb_cam/image_raw", Image, img_callback))
Subs.append(rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1))
Subs.append(rospy.Subscriber("/Left_color", String, trafficLeft_callback, queue_size=1))
Subs.append(rospy.Subscriber("/Right_color", String, trafficRight_callback, queue_size=1))
Subs.append(rospy.Subscriber("/Right_color", String, trafficSingle_callback, queue_size=1))
Subs.append(rospy.Subscriber("/time_count", Int64, trafficTime_callback, queue_size=1))
Subs.append(rospy.Subscriber('ar_pose_marker', AlvarMarkers, AR_callback, queue_size=1))

MotorPub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

# 한 줄 주행 함수 정의
def one_line_drive():
    pass

# 장애물 감지 함수 정의(Lidar 활용)
def obstacle_detection():
    pass

# 장애물 회피 주행 함수 정의
def avoid_obstacle():
    pass

# AR Tag 인식 함수 정의
def AR_detection():
    pass

# 터널 주행 함수 정의
def turnel_drive():
    pass

# 횡단보도 인식 함수 정의
def crosswalk_detection():
    pass

# 갈림길 선택 함수 정의
def crossroad_decision():
    pass

# 주차 함수 정의
def parking():
    pass

# 메인 함수 정의(기본 hough 주행)
def start():
    while True:
        # 자이카 주행 시작
        #############자이카 기본 주행 코드####################
        ############# h_drive_ex.py #########################
        global image
        global motor
        prev_x_left, prev_x_right = 0, WIDTH
        rospy.init_node('h_drive')
        motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
        print("--------------Xycar---------------")

        while not image.size == (WIDTH * HEIGHT * 3):
            continue
        while not rospy.is_shutdown():
            while img_ready == False:
                continue

            img = image.copy()
            display_img = img

            img_ready = False 

            # ====================
            # Find line position
            # ====================
            image = cv2.imread('line_pic1.png', cv2.IMREAD_COLOR)
            img = image.copy()
            display_img = img

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)
            edge_img = cv2.Canny(np.uint8(blur_gray), 30, 60)

            # cv2.imshow("original", img)
            # cv2.imshow("gray", gray)
            # cv2.imshow("gaussian blur", blur_gray)
            # cv2.imshow("edge", edge_img)
            cv2.waitKey(1)

            angle = 0 # 핸들조향각 값
            speed = 0 # 차량속도 값
            drive(angle, speed)            

        #################################################################

        
        # 자이카가 주행하면서 장애물은 계속 detect + 횡단보도 detect 하기

        # TODO: 곡선 주행 추가 알고리즘 실행
        # TODO: if 한 줄만 있을 때, one_line_drive() 실행
        # TODO: if 장애물 감지, 장애물 회피 알고리즘 실행
        # TODO: if 터널 AR tag 감지, 터널 알고리즘 실행
        # TODO: if 횡단보도 감지, 횡단보도+신호등 알고리즘 실행
        # TODO: if 갈림길 AR tag 감지, 갈림길 선택 알고리즘 실행
        # TODO: if 주차 AR tag 감지, 주차 정지 알고리즘 실행
        pass

if __name__ == '__main__':
    start()