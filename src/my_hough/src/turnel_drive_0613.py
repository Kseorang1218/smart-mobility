#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
import rospy, rospkg, time
from math import *
import time
import signal
import sys
import os



def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
motor = None
pre_angle = 0




def __init__():
    pass


def lidar_callback(data):
    global lidar_points, motor_msg, front_distance, left_distance, right_distance, num_data_f, num_data_l, num_data_r
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
        front_distance = 5.0  # or some default value

    for degree in range(40,70):
        if 0.0 < lidar_points[degree] < float('inf'):
            left_dist += lidar_points[degree]
            num_data_l += 1

    if num_data_l != 0:
        left_distance = left_dist / num_data_l
    else:
        left_distance = 0.0   # or some default value

    for degree in range(220,250):
        if 0.0 < lidar_points[degree] < float('inf'):
            right_dist += lidar_points[degree]
            num_data_r += 1

    if num_data_r != 0:
        right_distance = right_dist / num_data_r
    else:
        right_distance = 0.0   # or some default value

def drive(Angle, Speed): 
    global motor, pre_angle
    motor_msg = xycar_motor()
    if Angle == None:
        Angle  = pre_angle
    motor_msg.angle = Angle
    motor_msg.speed = Speed
    # print(motor_msg)
    motor.publish(motor_msg)
    pre_angle = Angle
    # rospy.sleep(1)

def PID(left, right, kp, ki, kd):

    global start_time, end_time, prev_error, i_error

    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = (left * cos(30) + right * cos(30)) / 2 - left
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

    return 100*output



def start():
    global motor
    global lidar_points, motor_msg, front_distance, left_distance, right_distance, num_data_l, num_data_f, num_data_r

    rospy.init_node('drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)    
    lidar_points = None
    # rospy.init_node('lidar_driver')
    rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1)
    rate = rospy.Rate(30)
    while lidar_points is None:
        continue

    while not rospy.is_shutdown():
        if left_distance > front_distance > right_distance:
            angle = -45  # 핸들조향각 값
            speed = 0  # 차량속도 값
            drive(angle, speed)
            
            print('오른쪽만')
        elif left_distance < front_distance < right_distance:
            angle = 45  # 핸들조향각 값
            speed = 0 # 차량속도 값
            drive(angle, speed)
            
            print('왼쪽만')
        else:
            angle = PID(left_distance, right_distance, 0.2, 0.00058, 0.1)  # 핸들조향각 값
            speed = 0  # 차량속도 값
            print(angle)
            drive(angle, speed)
            
            print('정상')
        print(left_distance, num_data_l)  
        print(front_distance, num_data_f)
        print(right_distance, num_data_r)
        rate.sleep()

if __name__ == '__main__':
    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()
    start()
