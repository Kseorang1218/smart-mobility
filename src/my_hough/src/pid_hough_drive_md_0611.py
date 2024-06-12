#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from std_msgs.msg import String, Int64
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
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

# Constants
CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250 
ROI_HEIGHT = HEIGHT - ROI_ROW 
L_ROW = ROI_HEIGHT - 120 # Row for position detection

# Constants for PID
i_error = 0.0
prev_error = 0.0

# AR values
arID = None
arData = {"DX":0.0, "DY":0.0, "DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
PARKING_AR_ID = 4
PARKING_DISTANCE = 1.15

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

# lidar callback
def lidar_callback(data):
        global lidar_points, motor_msg, front_distance, left_distance, right_distance
        lidar_points = data.ranges
        front_dist = 0
        left_dist = 0
        right_dist = 0
        num_data_f = 0
        num_data_r = 0
        num_data_l = 0
            
        for degree in range(170,190):
            if 0.0 < lidar_points[degree] < float('inf'):
                front_dist += lidar_points[degree]
                num_data_f += 1

        if num_data_f != 0:
            front_distance = front_dist / num_data_f
        else:
            front_distance = 1.0  # or some default value

        for degree in range(50,70):
            if 0.0 < lidar_points[degree] < float('inf'):
                left_dist += lidar_points[degree]
                num_data_l += 1

        if num_data_l != 0:
            left_distance = left_dist / num_data_l
        else:
            left_distance = 1.0   # or some default value

        for degree in range(230,250):
            if 0.0 < lidar_points[degree] < float('inf'):
                right_dist += lidar_points[degree]
                num_data_r += 1

        if num_data_r != 0:
            right_distance = right_dist / num_data_r
        else:
            right_distance = 1.0   # or some default value

# 신호등 콜백 함수
def traffic_light_callback(msg):
    global traffic_light_color
    traffic_light_color = msg.data
    print(traffic_light_color)

def time_callback(msg):
    global time_left
    time_left = msg.data

# 횡단보도 인식
def is_crosswalk_detected(image):
 
    x= 120
    y = 130
    width = 300
    height = 130
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5),0)
    roi = blur[320: 420, 180: 500]
    
    threshold_value = 100
    _, thresh = cv2.threshold(roi, threshold_value, 255, cv2.THRESH_BINARY)
    # print(thresh)
    # img_read = cv2.imread(roi)
    #cv2.imshow('thresh_img', thresh)
    #cv2.waitKey(1)

      # 픽셀 값을 255에서 1로 변환하고, 나머지는 0으로 변환
    binary_image = (thresh == 255).astype(int)
    # print(binary_image)
    # 1인 값들의 합을 계산
    white_area = np.sum(binary_image)

    # 전체 픽셀 수로 나눠서 비율을 계산
    total_pixels = binary_image.shape[0] * binary_image.shape[1]

    if white_area > 0:
        white_area_ratio_calculated = float(white_area) / float(total_pixels)
        threshold_ratio = 0.2  # 10% 기준

        # 흰색 영역 비율이 기준치를 넘으면 횡단보도 인식
        if white_area_ratio_calculated >= threshold_ratio:
            return True
    else:
        return False


    #min_line_length=50, max_line_gap=10, threshold=50

    # Canny 엣지 검출기 적용
    # edges = cv2.Canny(thresh, 30, 60)
    
    # # Hough Line Transform을 사용하여 선 검출
    # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, 10, 20)

    # if lines is None:
    #     return False
    
    # slopes, crosswalk_lines = filter_lines_by_slope(lines)

    # # 검출된 선을 이미지에 그리기 (디버깅 용도)
    # line_image = np.copy(thresh)
    # for line in crosswalk_lines:
    #     x1, y1, x2, y2 = line
    #     cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #     cv2.imshow("line_image", line_image)
    #     cv2.waitKey(1)
    
   

    # 횡단보도 검출 조건: 일정 간격을 가진 평행한 다수의 선이 존재해야 함
    # 이를 위해 선 간의 거리와 방향을 분석할 수 있음 (여기서는 단순히 선의 개수를 사용)
    # num_lines = len(crosswalk_lines)
    # print("Number of lines detected, crosswalk:", num_lines)
    
    # if num_lines >= 10:  # 예제 기준으로 최소 5개의 선이 검출되면 횡단보도로 인식
    #     return True
    # else:
    #     return False

# white_pixels = cv2.countNonZero(cv2.inRange(roi, 200, 255))

# 예제 사용
# image = cv2.imread('path_to_your_image.jpg')
# result = is_crosswalk_detected(image)
# print("Crosswalk detected:", result)

# 디버깅을 위해 검출된 선을 그린 이미지 보기
# cv2.imshow('Detected Lines', line_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

  
    # for i in white_pixels:
    #    for j in white_pixels:
    #        white_area = np.sum(white_pixels[i][j] == 255) / (white_pixels.shape[0] * white_pixels.shape[1])
    # print(white_area)
    # white_pixels = np.sum(thresh == 255)
    # total_pixels = thresh.shape[0] * thresh.shape[1]
    # white_area = white_pixels / total_pixels
    
    # print("White area ratio:", white_area)
    
    # if white_area >= 0:     # 횡단보도 O
    #     return True
    # else:                       # 횡단보도 X
    #     return False
    

    # _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    # white_area = np.sum(thresh == 255) / (thresh.shape[0] * thresh.shape[1])
    # if white_area > 0.1:
    #     return True
    # return False

def is_stop_line_detected(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray2 = gray[370: 420, 180: 500]
    edges = cv2.Canny(gray2, 50, 150)
    
    stop_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, 50, 20)

    if stop_lines is None:
        return False

    line_image = np.copy(gray2)
    # for line in stop_lines:
    #     x1, y1, x2, y2 = line[0]
    #     cv2.line(line_image, (x1,y1), (x2,y2), (0,255,0), 2)
    #     cv2.imshow("stop_line", line_image)
    #     cv2.waitKey(1)

    stop_num_lines = len(stop_lines)
    print("Number of lines detected, stoplines:", stop_num_lines)
    
    if stop_num_lines >= 10:  # 예제 기준으로 최소 5개의 선이 검출되면 횡단보도로 인식
        return True
   
    return True



# 신호등 상태에 따라 차량 정지 여부 결정
def should_stop_for_traffic_light(traffic_light_color):
    return traffic_light_color == "G" or traffic_light_color == "R" or ( traffic_light_color == "Y" and time_left < 3 )

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
    global arID
    prev_x_left, prev_x_right = 0, WIDTH

    # Initialize the ROS node
    rospy.init_node('h_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

     # Subscribe to the camera topic to get images
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
    single_sub = rospy.Subscriber("Single_color", String, traffic_light_callback)
    timecnt_sub = rospy.Subscriber("time_count", Int64, time_callback)
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

        # 오른쪽 차선이 없는 경우 -> 일단 한줄 주행 하드코딩적으로 구현함
        # if m_right == 0.0 or len(m_right) < 3:
        #     angle = -45
        #     speed = 4
        speed = 4 # 차량속도 값
        drive(angle, speed)

        # cv2.imshow("img", img)
        # is_crosswalk_detected(img)

        # if arID == 6 and arData["DZ"] < 0.55:
        #     #TODO: 터널 주행 알고리즘 주행

        # 횡단보도 인식 시 알고리즘
        if is_crosswalk_detected(img):
            print("crosswalk detected")
            #drive(0,0)
            if should_stop_for_traffic_light(traffic_light_color):
                print("!!!!!!!!!!!!!!!!!!!!!!!!")
                #drive(0,0)
            if is_stop_line_detected(img):
                print("Stop line detected. Stopping the vehicle.")
                drive(0,0)
            else:
                print("Crosswalk detected but no stop line found.")

        # if arID == 2 and arData["DZ"] < 0.55:
        #     #TODO: 갈림길 알고리즘 주행

        # 주차 AR 태그 인식
        # TODO: 0.55도 상수 이름 지정해주기 -> 완료
        if arID == PARKING_AR_ID and arData["DZ"] < PARKING_DISTANCE:
            drive(0, 0)
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break         

if __name__ == '__main__':
    start_time = time.time()
    start()
