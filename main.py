#!/usr/bin/env python
# -*- coding: utf-8 -*-

#########################################################################
# 스모빌 자이카 경로 주행 메인 코드
# 제작 날짜: 2024-05-28
# 팀 이름: 문동이랑 팀
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


# 한 줄 주행 함수 정의
def one_line_drive():
    pass

# 장애물 회피 주행 함수 정의
def avoid_obstacle():
    pass

# AR Tag 인식 함수 정의
def ar_detection():
    pass

# 터널 주행 함수 정의
def turnel_drive():
    pass

# 횡단보도 인식 함수 정의
def crosswalk_detection():
    pass


# 메인 함수 정의
def start():
    while True:
        # 자이카 주행 시작
        pass

if __name__ == '__main__':
    start()