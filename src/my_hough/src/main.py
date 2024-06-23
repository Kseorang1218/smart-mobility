#!/usr/bin/env python
# -*- coding: utf-8 -*-

from TunnelDrive import TunnelDriver
from MovingAverage import MovingAverage

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
import time

# Constants
CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250 
ROI_HEIGHT = HEIGHT - ROI_ROW 
L_ROW = ROI_HEIGHT - 120 # Row for position detection
SPEED = 5

# Constants for PID
i_error = 0.0
prev_error = 0.0

# AR values
arID = None
arData = {"DX":0.0, "DY":0.0, "DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
CROSSROAD_AR_ID = 2
PARKING_AR_ID = 4
TUNNEL_AR_ID = 6
PARKING_AR_DISTANCE = 1.0 
CROSSROAD_AR_DISTANCE = 1.0

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
img_ready = False
tunnel_driver = TunnelDriver()
left_color = None
right_color = None
time_left = None
crossroads_task_completed = False
cnt = 0
single_color = None
crossroad_left_drive = False
crossroad_right_drive = False
decide_fastest_path_flag = False
lidar_points = None

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

"""callback 함수들"""

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
        global lidar_points, front_distance
        lidar_points = data.ranges
        front_distance = float('inf')
      
        for degree in range(160, 200):
            if 0.10 < lidar_points[degree] < front_distance:
                front_distance = lidar_points[degree]
        
# 신호등 콜백 함수
def single_callback(msg):
    global single_color
    single_color = msg.data

def left_callback(msg):
    global left_color
    left_color = msg.data
    
def right_callback(msg):
    global right_color
    right_color = msg.data

def time_callback(msg):
    global time_left
    time_left = msg.data


"""주행에 필요한 함수들"""

# 횡단보도 인식 여부 확인 함수
# 이미지, threshold_ratio를 파라미터로 받아 이미지에서  흰색 영역 비율이 threshold_ratio를 넘으면 True,
# 그렇지 않다면 False를 반환
def is_crosswalk_detected(image, threshold_ratio):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5),0)

    # 횡단보도 인식을 위한 ROI 설정
    roi_img = blur[340: 420, 180: 500]
  
    threshold_value = 40
    _, thresh = cv2.threshold(roi_img, threshold_value, 255, cv2.THRESH_BINARY)

    # 픽셀 값을 255에서 1로 변환하고, 나머지는 0으로 변환
    binary_image = (thresh == 255).astype(int)

    # 1인 값들의 합을 계산
    white_area = np.sum(binary_image)

    # 전체 픽셀 수로 나눠서 비율을 계산
    total_pixels = binary_image.shape[0] * binary_image.shape[1]

    white_area_ratio = float(white_area) / float(total_pixels)

     # 흰색 영역 비율이 기준치를 넘으면 횡단보도 인식
    if white_area_ratio >= threshold_ratio:
        return True
    else:
        return False

# 신호등 상태에 따라 차량 정지 여부 결정
def should_stop_for_traffic_light(traffic_light_color):
    if traffic_light_color == "R" or traffic_light_color == "Y":
        return True
    else:
        return False

def go_for_traffic_light(traffic_light_color):
    if traffic_light_color == "G":
        return True
    else:
        False

# 갈림길 횡단보도에 도달했을 시 기다리는 시간을 계산하는 함수
def calculate_time_to_wait(color, time_left):
    if (color == "G" and time_left == 10) or (color == "R" and time_left < 10):
        return 0
    else:
        return float('inf')
    
# 갈림길 중 더 빠른 곳을 선택하는 함수
def decide_fastest_path():
    global decide_fastest_path_flag
    left_time = calculate_time_to_wait(left_color, time_left)
    right_time = calculate_time_to_wait(right_color, time_left)
    decide_fastest_path_flag = True
    if left_time < right_time:
        return "left"
    else:
        return "right"

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

# 이미지를 받아 전처리 후 Canny edge detection을 수행하는 함수
def get_edge_img(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 30, 60)

    return edge_img

# 이미지에 있는 직선을 검출하는 함수
def get_lines(image, rho, theta, threshold, minLineLength, maxLineGap):
    all_lines = cv2.HoughLinesP(image, rho, theta, threshold, minLineLength, maxLineGap)

    return all_lines

# 이미지에서 ROI를 설정하는 함수
def get_roi(img):
    roi_img = img[ROI_ROW:HEIGHT, 0:WIDTH]

    return roi_img

# 이미지에서 검출된 직선 중 기울기가 너무 작은 직선을 
# 필터링하는 함수
def filter_lines_by_slope(all_lines):
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

# 필터링된 직선과 기울기를 파라미터로 받아 해당 직선이
# 왼쪽 차선의 직선인지, 오른쪽 차선의 직선인지 판단하는 함수
def separate_lines(slopes, filtered_lines):
    left_lines = []
    right_lines = []
    
    for j in range(len(slopes)):
        line = filtered_lines[j]
        slope = slopes[j]
        
        x1, y1, x2, y2 = line

        if (slope < 0) and (x2 < WIDTH/2):
            left_lines.append(line.tolist())    
        elif (slope > 0) and (x1 > WIDTH/2):
            right_lines.append(line.tolist())

    return left_lines, right_lines

# 왼쪽 차선의 직선으로부터 하나의 대표 선을 선정하고
# 평균 기울기와 절편을 리턴하는 함수
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

# 오른쪽 차선의 직선으로부터 하나의 대표 선을 선정하고
# 평균 기울기와 절편을 리턴하는 함수
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

# 기울기, 절편, y좌표를 기준으로 x좌표를 검출하는 함수
# 기본적으로 본 함수로 계산된 오른쪽 x좌표, 왼쪽 y좌표의 평균값을 향해
# 주행하게 된다.
def calculate_x(m, b, y):
    if m == 0.0:
        return None
    return int((y - b) / m)
    
    
'''주행 시작 함수'''
def start():
    # global 변수들 선언
    global img_ready
    global image
    global motor
    global img
    global arID
    global tunnel_driver
    global crossroads_task_completed
    global cnt
    global single_color
    global crossroad_left_drive
    global crossroad_right_drive
    global decide_fastest_path_flag
    global lidar_points
    global SPEED
    global left_color, right_color
    
    # 변수 초기화
    obstacle_right_num = 0
    obstacle_left_num = 0
    sharp_curve_threshold = 50
    gap_mv = MovingAverage(15)
    gap_mv.add_sample(450)

    # Initialize the ROS node
    rospy.init_node('h_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

     # Subscribe to the camera topic to get images
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
    single_sub = rospy.Subscriber("Single_color", String, single_callback)
    left_Sub = rospy.Subscriber("Left_color",  String, left_callback)
    right_Sub = rospy.Subscriber("Right_color",  String, right_callback)
    timecnt_sub = rospy.Subscriber("time_count", Int64, time_callback)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1)


    print("--------------Xycar---------------")
    rospy.sleep(1)

    rate = rospy.Rate(30)
    # Wait until the image size matches the expected size
    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    # Main loop
    while not rospy.is_shutdown():
        while img_ready == False:
            continue
        img = image.copy()
        img_ready = False

        # 이미지 전처리
        edge_img = get_edge_img(img)
        roi_edge_img = get_roi(edge_img)

  
        # 직선 검출
        all_lines = get_lines(roi_edge_img, 1, math.pi/180, 50, 15, 10)
        if all_lines is None:
            continue


        # 직선 필터링, 기울기와 절편 계산
        slopes, filtered_lines = filter_lines_by_slope(all_lines)
        left_lines, right_lines = separate_lines(slopes, filtered_lines)
        
        m_left, b_left = get_left_lines(left_lines)
        m_right, b_right = get_right_lines(right_lines)


        # 갈림길에서 왼쪽 길이 더 빠르다는 판단을 했을 때
        if crossroad_left_drive and not crossroad_right_drive:
            m_right = 0     
        # 갈림길에서 오른쪽 길이 더 빠르다는 판단을 했을 때       
        elif crossroad_right_drive and not crossroad_left_drive:
            m_left = 0


        # 왼쪽 기울기가 0인 경우(=왼쪽 직선이 없는 경우), 차선폭의 가중평균을 이용해
        # 왼쪽 차선 설정 및 주행
        if m_left == 0.0 and m_right != 0.0:
            x_right = calculate_x(m_right, b_right, L_ROW)
            x_left = x_right - gap_mv.get_mm()
        # 오른쪽 기울기가 0인 경우(=오른쪽 직선이 없는 경우), 차선폭의 가중평균을 이용해
        # 오른쪽 차선 설정 및 주행
        elif m_right == 0.0 and m_left != 0.0:
            x_left = calculate_x(m_left, b_left, L_ROW)
            x_right = x_left + gap_mv.get_mm()
        # 두 차선 모두 없는 경우, 한 줄 주행의 경우이므로 왼쪽으로 꺾음
        elif m_right == 0.0 and m_left == 0.0:
            angle = -50
        # 그 외 모든 경우에는 기울기와 절편을 기반으로 x좌표를 계산해 주행 
        else:
            x_right = calculate_x(m_right, b_right, L_ROW)
            x_left = calculate_x(m_left, b_left, L_ROW)


        # x_left와 x_right의 평균. 해당 값을 목표로 주행하게 됨
        x_midpoint = (x_left + x_right) // 2 

        
        # 차선폭 계산 및 업데이트
        gap = x_right - x_left
        gap_mv.add_sample(gap) 


        # PID로 구한 핸들조향각 값
        angle = PID(x_midpoint, 0.4 ,0.03,0.01)

      
        # 차선의 위치가 너무 작거나 큰 경우 급커브 구간이라고 판단, steering angle 조정
        # 왼쪽으로 급커브
        if x_left < sharp_curve_threshold:
            angle = -50
        # 오른쪽으로 급커브
        if x_right > WIDTH - sharp_curve_threshold:
            angle = 50

        drive(angle, SPEED)


        # 장애물 회피 알고리즘
        obstacle_distance_threshold = 0.35
        obstacle_num_threshold = 3
        for degree in range(40,110):
            if (0.01 < lidar_points[180 + degree] <= obstacle_distance_threshold):
                obstacle_right_num += 1
            if (0.01 < lidar_points[180 - degree] <= obstacle_distance_threshold):
                obstacle_left_num += 1
        if obstacle_right_num > obstacle_num_threshold:
            print("avoid right obstacle")
            drive(-35, 4)
            time.sleep(0.3)
            obstacle_right_num = 0
        elif obstacle_left_num > obstacle_num_threshold:
            print("avoid left obstacle")
            drive(35, 4)
            time.sleep(0.3)
            obstacle_left_num = 0
        else:
            drive(angle, SPEED)


        # 터널 주행
        if arID == TUNNEL_AR_ID and arData["DZ"] < 0.7:
            print("Tunnel start")
            while True:
                drive(0,3)
                if (front_distance < 0.5):
                    break

            tunnel_driver.start()
            arID = -1
            SPEED = 5


        # 횡단보도 유무 판단
        if is_crosswalk_detected(img, 0.15) and len(all_lines) > 37: 
            print("횡단보도 감지")
            if crossroad_left_drive == True:
                while should_stop_for_traffic_light(left_color):
                    print("crosswalk detected and red color")
                    print("stop!")
                    drive(0,0)
                    if go_for_traffic_light(left_color):
                        break
            elif crossroad_right_drive == True:
                print("right color: ", right_color)
                while should_stop_for_traffic_light(right_color):
                    drive(0,0)
                    if go_for_traffic_light(right_color):
                        print("go!")
                        break
                if go_for_traffic_light(right_color):
                    print("go!")
            else:
                while should_stop_for_traffic_light(single_color):
                    drive(0,0)
                    if go_for_traffic_light(single_color):
                        print("go!")
                        break
                if go_for_traffic_light(single_color):
                    print("go!")


        # 갈림길 AR코드 인식 및 빠른 경로 계산    
        if arID == CROSSROAD_AR_ID and arData["DZ"] < CROSSROAD_AR_DISTANCE:
            if not decide_fastest_path_flag:
                fastest_path = decide_fastest_path()
                if fastest_path == 'left':
                    print("The fastest path is: left")
                    crossroad_left_drive = True
                elif fastest_path == 'right':
                    print("The fastest path is: right")
                    crossroad_right_drive = True
                else:
                    print("AR tag detected, but waiting for traffic light data...")

    
        # 주차 AR 태그 인식
        if arID == PARKING_AR_ID and arData["DZ"] < PARKING_AR_DISTANCE:
            print("Parking")
            drive(0, 0)
            break
    

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break         

        rate.sleep()

if __name__ == '__main__':
    start_time = time.time()
    start()