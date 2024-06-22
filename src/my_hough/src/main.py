#!/usr/bin/env python
# -*- coding: utf-8 -*-

from TunnelDriver import TunnelDriver
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
import random   
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
PARKING_AR_ID = 4
TUNNEL_AR_ID = 6
CROSSROAD_AR_ID = 2
PARKING_DISTANCE = 1.0 # 1.0->0.9->1.0
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
    #print("single traffic light color:", traffic_light_color)
 

def left_callback(msg):
    global left_color
    left_color = msg.data
    #print("left_color:" , left_color)
   

    
def right_callback(msg):
    global right_color
    right_color = msg.data
    #print("right_color:" , right_color)
   


def time_callback(msg):
    global time_left
    time_left = msg.data

# 횡단보도 인식
def is_crosswalk_detected(image, threshold_ratio):
 
    x= 120
    y = 130
    width = 300
    height = 130
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5),0)
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
        print("횡단보도가 인식되었습니다.")
    else:
        return False



def is_stop_line_detected(image):
    
    stop_lines = cv2.HoughLinesP(image, 1, np.pi / 180, 30, 50, 20)

    if stop_lines is None:
        return False

    stop_num_lines = len(stop_lines)
    print("Number of lines detected, stoplines:", stop_num_lines)
    
    if stop_num_lines >= 20:  # 예제 기준으로 최소 5개의 선이 검출되면 횡단보도로 인식
        return True
   
    return True


# 신호등 상태에 따라 차량 정지 여부 결정
def should_stop_for_traffic_light(traffic_light_color):
    global time_left
    return  traffic_light_color == "R" or traffic_light_color == "Y" 

def go_for_traffic_light(traffic_light_color):
    global time_left
    return  traffic_light_color == "G"

#갈림길 인식 알고리즘
def calculate_time_to_green(color, time_left):

    if (color == "G" and time_left == 10) or (color == "R" and time_left < 10):
        return 0
    else:
        return float('inf')


def decide_fastest_path():
    global decide_fastest_path_flag
    left_time = calculate_time_to_green(left_color, time_left)
    right_time = calculate_time_to_green(right_color, time_left)
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

def get_edge_img(img):
    """1"""
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
        line = filtered_lines[j]
        slope = slopes[j]
        
        x1, y1, x2, y2 = line

        if (slope < 0) and (x2 < WIDTH/2):
            left_lines.append(line.tolist())    
        elif (slope > 0) and (x1 > WIDTH/2):
            right_lines.append(line.tolist())

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

def calculate_x(m, b, y):
    if m == 0.0:
        return None
    return int((y - b) / m)
    
    

def start():
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
    
    prev_x_left, prev_x_right = 0, WIDTH
    prev_angle = 0
    obstacle_right_num = 0
    obstacle_left_num = 0
    obstacle_num = 0
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

        edge_img = get_edge_img(img)

        roi_edge_img = get_roi(edge_img)
  


        all_lines = get_lines(roi_edge_img, 1, math.pi/180, 50, 15, 10)
        if all_lines is None:
            continue

        slopes, filtered_lines = filter_lines_by_slope(all_lines)
        left_lines, right_lines = separate_lines(slopes, filtered_lines)
        m_left, b_left = get_left_lines(left_lines)
        m_right, b_right = get_right_lines(right_lines)

        if crossroad_left_drive and not crossroad_right_drive:
            m_right = 0
            
        elif crossroad_right_drive and not crossroad_left_drive:
            m_left = 0


        if m_left == 0.0 and m_right != 0.0:
            x_right = calculate_x(m_right, b_right, L_ROW)
            x_left = x_right - gap_mv.get_mm()

        elif m_right == 0.0 and m_left != 0.0:
            x_left = calculate_x(m_left, b_left, L_ROW)
            x_right = x_left + gap_mv.get_mm()

        # 한 줄 주행
        elif m_right == 0.0 and m_left == 0.0:
            drive(-50, 4)
        
        else:
            x_right = calculate_x(m_right, b_right, L_ROW)
            x_left = calculate_x(m_left, b_left, L_ROW)

        x_midpoint = (x_left + x_right) // 2 
        
        gap = x_right - x_left
        gap_mv.add_sample(gap) 
        angle = PID(x_midpoint, 0.4 ,0.03,0.01) # PID로 구한 핸들조향각 값
      
        if x_left < 50:
            SPEED = 4
            angle = -50
        
        drive(angle, SPEED)
        prev_angle = angle

        # 장애물 회피 알고리즘 주행
        meter = 0.35
        for degree in range(40,110):
            if (0.01 < lidar_points[180 + degree] <= meter):
                obstacle_right_num += 1
            if (0.01 < lidar_points[180 - degree] <= meter):
                obstacle_left_num += 1
        if obstacle_right_num > 3:
            print("avoid right obstacle")
            drive(-35, 4)
            time.sleep(0.3)
            obstacle_right_num = 0
        elif obstacle_left_num >3:
            print("avoid left obstacle")
            drive(35, 4)
            time.sleep(0.3)
            obstacle_left_num = 0
        else:
            drive(angle, SPEED)


            

        #TODO: 터널 주행 알고리즘 주행
        if arID == TUNNEL_AR_ID and arData["DZ"] < 0.7:
            print("Tunnel start")
            while True:
                drive(0,3)
                if (front_distance < 0.5):
                    break

            tunnel_driver.start()
            arID = -1
            SPEED = 5
    
        if is_crosswalk_detected(img, 0.15) and len(all_lines) > 37: 
            print("횡단보도 감지")
            if crossroad_left_drive == True:
               
                while should_stop_for_traffic_light(left_color):
                    print("crosswalk detected and red color")
                    print("stop!")
                    drive(0,0)
                    if go_for_traffic_light(left_color):
           
                        break

                if go_for_traffic_light(left_color):
                
                   pass
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


        # 갈림길 AR코드 인식 시 알고리즘       
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
        if arID == PARKING_AR_ID and arData["DZ"] < PARKING_DISTANCE:
            print("Parking")
            drive(0, 0)
            break
    

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break         

        rate.sleep()

if __name__ == '__main__':
    start_time = time.time()
    start()