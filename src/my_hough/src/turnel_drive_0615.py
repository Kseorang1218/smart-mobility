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

def __init__():
    pass


def lidar_callback(data):
    global lidar_points, motor_msg, front_distance, left_distance, right_distance
    lidar_points = data.ranges
    front_distance = float('inf')
    left_distance = float('inf')
    right_distance = float('inf')

    for degree in range(160,200):
        if 0.10< lidar_points[degree] < front_distance:
            front_distance = lidar_points[degree]

    for degree in range(20,50):
        if 0.10< lidar_points[degree] < left_distance:
                left_distance = lidar_points[degree]

    for degree in range(310,340):
        if 0.10< lidar_points[degree] < right_distance:
                right_distance = lidar_points[degree]

def drive(Angle, Speed): 
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed
    motor.publish(motor_msg)
   
def start():
    global motor
    global lidar_points, motor_msg, front_distance, left_distance, right_distance
    rospy.init_node('drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)    
    lidar_points = None
    
    # rospy.init_node('lidar_driver')
    rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1)
    rate = rospy.Rate(30)
    while lidar_points is None:
        continue

    while not rospy.is_shutdown():
        
        if (right_distance == float('inf') or right_distance < 0.4) :
            angle = -45  # 핸들조향각 값
            speed = 3 # 차량속도 값
            if front_distance < 0.65 :
                angle = -50 
                print('전방 위험')       
            drive(angle, speed)
            print('오른쪽 위험')
        
        elif (left_distance < 0.4):
            angle = 45  # 핸들조향각 값
            speed = 3 # 차량속도 값
            if front_distance < 0.7 :
                angle = 50 
                print('전방 위험')
            drive(angle, speed)          
            print('왼쪽 위험')
             
        else:
         
            drive(0, 3)
            print('정상')   
        print('L',left_distance)  
        print('F',front_distance)
        print('R',right_distance)
        rate.sleep()

        # elif (left_distance == float('inf') or left_distance < 0.35):
        #     angle = -40  # 핸들조향각 값
        #     speed = 3 # 차량속도 값
        #     if front_distance < 0.65 :
        #         angle = -40 
        #         print('전방 위험')
        #     drive(angle, speed)          
        #     print('왼쪽 위험')
             
        # else:
        #     angle = PID(left_distance, right_distance, 0.2, 0.00058, 0.1) 
        #     drive(angle, 0)
        #     print('정상')   

if __name__ == '__main__':
    start_time = time.time()
    start()