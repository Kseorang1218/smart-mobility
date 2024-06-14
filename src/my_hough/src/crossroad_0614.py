#!/usr/bin/env python
# -*- coding: utf-8 -*-

from turnel_drive_class import TunnelDriver
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

SPEED = 6
# Constants for PID
i_error = 0.0
prev_error = 0.0

# AR values
arID = None
arData = {"DX":0.0, "DY":0.0, "DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
PARKING_AR_ID = 4
TUNNEL_AR_ID = 6
CROSSROAD_AR_ID = 2
PARKING_DISTANCE = 1.0

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
traffic_light_color = None
crossroad_left_drive = False
crossroad_right_drive = False
decide_fastest_path_flag = True

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
    print("single traffic light color:", traffic_light_color)

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
def is_crosswalk_detected(image):
 
    x= 120
    y = 130
    width = 300
    height = 130
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5),0)
    roi_img = blur[340: 420, 180: 500]
    
    threshold_value = 120
    _, thresh = cv2.threshold(roi_img, threshold_value, 255, cv2.THRESH_BINARY)
    # cv2.imshow('thresh_img', thresh)
    # cv2.waitKey(1)

      # 픽셀 값을 255에서 1로 변환하고, 나머지는 0으로 변환
    binary_image = (thresh == 255).astype(int)

    # 1인 값들의 합을 계산
    white_area = np.sum(binary_image)

    # 전체 픽셀 수로 나눠서 비율을 계산
    total_pixels = binary_image.shape[0] * binary_image.shape[1]

    white_area_ratio = float(white_area) / float(total_pixels)
    # print(white_area_ratio)
    threshold_ratio = 0.3 
     # 흰색 영역 비율이 기준치를 넘으면 횡단보도 인식
    if white_area_ratio >= threshold_ratio:
        return True
    else:
        return False



def is_stop_line_detected(image):
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # gray2 = gray[370: 420, 180: 500]
    # edges = cv2.Canny(gray2, 50, 150)
    
    stop_lines = cv2.HoughLinesP(image, 1, np.pi / 180, 30, 50, 20)

    if stop_lines is None:
        return False

    # line_image = np.copy(gray2)
    # for line in stop_lines:
    #     x1, y1, x2, y2 = line[0]
    #     cv2.line(line_image, (x1,y1), (x2,y2), (0,255,0), 2)
    #     cv2.imshow("stop_line", line_image)
    #     cv2.waitKey(1)

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
    if color == "G":
        return 0
    elif color == "R":
        return time_left
    elif color == "Y":
        return time_left
    else:
        return float('inf')

def decide_fastest_path():
    global decide_fastest_path_flag
    left_time = calculate_time_to_green(left_color, time_left)
    right_time = calculate_time_to_green(right_color, time_left)
    decide_fastest_path_flag = False
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

    # """2"""
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # # Apply Gaussian Blur to reduce noise and improve the Otsu thresholding
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # # Apply Otsu's thresholding
    # _, otsu_thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # # Invert the binary image if necessary (white lines on black background)
    # otsu_thresh = cv2.bitwise_not(otsu_thresh)
    
    # # Apply morphological operations to remove small noise and enhance the lines
    # kernel = np.ones((3, 3), np.uint8)
    # morph = cv2.morphologyEx(otsu_thresh, cv2.MORPH_CLOSE, kernel)
    
    # # Apply Canny Edge Detection
    # edge_img = cv2.Canny(morph, 50, 150)

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
    # slopes_horizental = []
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
        # else:
        #     slopes_horizental(slope)

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
    global traffic_light_color
    global crossroad_left_drive
    global crossroad_right_drive
    global decide_fastest_path_flag
    
    prev_x_left, prev_x_right = 0, WIDTH
    prev_angle = 0
    gap_mv = MovingAverage(15)
    gap_mv.add_sample(450)

    # Initialize the ROS node
    rospy.init_node('h_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

     # Subscribe to the camera topic to get images
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
    single_sub = rospy.Subscriber("Single_color", String, traffic_light_callback)
    left_Sub = rospy.Subscriber("Left_color",  String, left_callback)
    right_Sub = rospy.Subscriber("Right_color",  String, right_callback)
    timecnt_sub = rospy.Subscriber("time_count", Int64, time_callback)


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
        # cv2.imshow("original img", img)
        # cv2.waitKey()
        # cv2.imshow("edge_img", edge_img)
        # cv2.waitKey(1)

        # if crossroad_left_drive:
        #     roi_edge_img = edge_img[ROI_ROW:HEIGHT, 0:WIDTH//2]
        #     crossroad_left_drive = False
        # elif crossroad_right_drive:
        #     roi_edge_img = edge_img[ROI_ROW:HEIGHT, WIDTH//2:WIDTH]
        #     crossroad_right_drive = False
        # else:
        roi_edge_img = get_roi(edge_img)
        # cv2.imshow("roi_edge_img",  roi_edge_img)
        # cv2.waitKey(1)


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
     
        #print("m_left", m_left)
        line_draw_img = get_roi(img)
        # cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)
        # cv2.imshow("left & right lines", line_draw_img)
        # cv2.waitKey(1)
        # m_right, b_right = get_right_lines(right_lines)
        # m_left, b_left = get_left_lines(left_lines)
        # m_right, b_right = get_right_lines(right_lines)

        if m_left == 0.0 and m_right != 0.0:
            x_right = calculate_x(m_right, b_right, L_ROW)
            x_left = x_right - gap_mv.get_mm()
        elif m_right == 0.0 and m_left != 0.0:
            x_left = calculate_x(m_left, b_left, L_ROW)
            x_right = x_left + gap_mv.get_mm()
        elif m_right == 0.0 and m_left == 0.0:
            speed = SPEED
            drive(-50, speed)
        # elif m_left == 0 and m_right == 0:
        #     # x_right = calculate_x(m_right, b_right, L_ROW)
        #     # x_left = calculate_x(m_left, b_left, L_ROW)
        #     # print("x_right:",x_right)
        #     # print("x_left: ", x_left)
        # #     # angle = PID((prev_x_left+prev_x_right)//2, 0.28, 0.00058, 0.1)
        # #     # drive(angle, 4)
        #     x_right = prev_x_right
        #     x_left = prev_x_left
        # #     print("x_right:",x_right)
        # #     print("x_left: ", x_left)
        else:
            x_right = calculate_x(m_right, b_right, L_ROW)
            x_left = calculate_x(m_left, b_left, L_ROW)
            

        # if x_right == None and x_left == None:
        #     speed = 4

        #     is_stop_line_detected(roi_edge_img)
        #     drive(prev_angle, speed)
        #     #print("prev_angle:", prev_angle)

        # else:        
        # gap = x_right - x_left
        

        x_midpoint = (x_left + x_right) // 2 
        
        gap = x_right - x_left
        gap_mv.add_sample(gap)
        angle = PID(x_midpoint,0.28 ,0.00058,0.1) # 핸들조향각 값
        

        speed = SPEED # 차량속도 값
        drive(angle, speed)
        #print("angle:", angle)
        prev_angle = angle

        view_center = WIDTH//2
        line_draw_img = get_roi(img)
        cv2.line(line_draw_img, (0,L_ROW), (WIDTH,L_ROW), (0,255,255), 2)
        # cv2.rectangle(line_draw_img, (x_left-5,L_ROW-5), (x_left+5,L_ROW+5), (0,255,0), 4)
        # cv2.rectangle(line_draw_img, (x_right-5,L_ROW-5), (x_right+5,L_ROW+5), (0,255,0), 4)
        # cv2.rectangle(line_draw_img, (x_midpoint-5,L_ROW-5), (x_midpoint+5,L_ROW+5), (255,0,0), 4)
        # cv2.rectangle(line_draw_img, (view_center-5,L_ROW-5), (view_center+5,L_ROW+5), (0,0,255), 4)

        # cv2.imshow("Lines positions", line_draw_img)
        # cv2.waitKey(1)

        if arID == TUNNEL_AR_ID and arData["DZ"] < 0.55:
            #
            print("Tunnel start")
            tunnel_driver.start()
        #     #TODO: 터널 주행 알고리즘 주행

        # 횡단보도 인식 시 알고리즘
        if is_crosswalk_detected(img): 
            while should_stop_for_traffic_light(traffic_light_color):
                print("crosswalk detected and red color")
                print("stop!")
                drive(0,0)
                if go_for_traffic_light(traffic_light_color):
                    print("Crosswalk detected but green color.")
                    print("go!")
                    break
            
            if go_for_traffic_light(traffic_light_color):
                print("Crosswalk detected but green color.")
                print("go!")

        # else:
        #     print("Crosswalk no detected.")

    # 갈림길 AR코드 인식 시 알고리즘
      
        
       # print("gap", gap)

        if arID == CROSSROAD_AR_ID and arData["DZ"] < 1.0:
            #print("갈림길 인식!!")
            # print("AR tag id:", arID)
            # print("ar tag distance:", arData["DZ"])
            # if not crossroads_task_completed:
            if crossroads_task_completed == False:
                if decide_fastest_path_flag:
                    fastest_path = decide_fastest_path()
                    fastest_path = 'left'
                    
                print("hi1")
                # crossroads_task_completed = True
                # print("hi2")
                if fastest_path == 'left':
                    # print("hi3")
                    # print("current Gap:", gap)
                    print("The fastest path is: left")
                    crossroad_left_drive = True
                    # if gap>450:
                    #     print("Gap is over 600:", gap)
                    #     cnt+= 1
                    #     print("cnt:", cnt)
                        
                    # elif gap <= 450:
                    #     continue
                    # if cnt >=1 and cnt <10:
                    #     print("왼쪽으로 주행합니다.")
                    #     drive(-30,4)
                    #     #crossroad_left_drive = True
                    #     if cnt ==10:
                    #         crossroads_task_completed = True
                    #         break
                                
                        # if (x_right - x_left > )
                        # drive(0, 4)
                        # time.sleep(4)
                        # drive(20, 4)
                        # time.sleep(2)
                        # crossroads_task_completed = True
                        # break
                elif fastest_path == 'right':
                    print("The fastest path is: right")
                
                    # print("hi4")
                    # print("Gap:", gap)
                    # if gap>450:
                    #     print("Gap is over 600:", gap)
                    #     cnt+= 1
                    #     print("cnt:", cnt)
                        
                    # elif gap <= 450:
                    #     continue
                    # while cnt >=1 and cnt <10:
                    # if gap>900:
                    #     cnt+= 1
                    # if cnt == 3:
                    print("오른쪽으로 주행합니다.")
                        # drive(30,4)
                    crossroad_right_drive = True
                        # if cnt ==10:
                        #     crossroads_task_completed = True
                        #     break
                        
                            
                        # drive(0, 4)
                        # time.sleep(4)
                        # drive(-20, 4)
                        # time.sleep(2)
                        # crossroads_task_completed = True
                        # break

                else:
                    print("AR tag detected, but waiting for traffic light data...")

        

                # else: 
                #     break


        # if arID == 2 and arData["DZ"] < 0.55:
        #TODO: 갈림길 알고리즘 주행

        # 주차 AR 태그 인식
        # TODO: 0.55도 상수 이름 지정해주기 -> 완료
        if arID == PARKING_AR_ID and arData["DZ"] < PARKING_DISTANCE:
            print("Parking start")
            drive(0, 0)
            break
    

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break         

        rate.sleep()

if __name__ == '__main__':
    start_time = time.time()
    start()
