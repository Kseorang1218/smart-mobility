#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
import signal
import rospkg
import sys
import os
from math import cos

class TurnelDriver:
    def __init__(self):
        # rospy.init_node('turnel_driver')
        # self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        # rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        self.lidar_points = None
        self.motor_msg = xycar_motor()
        self.pre_angle = 0
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.start_time = time.time()
        self.prev_error = 0.0
        self.i_error = 0.0
        self.rate = rospy.Rate(30)

    def lidar_callback(self, data):
        self.lidar_points = data.ranges
        front_min_dist = float('inf')
        left_min_dist = float('inf')
        right_min_dist = float('inf')

        for degree in range(170, 190):
            if 0.05 < self.lidar_points[degree] < front_min_dist:
                front_min_dist = self.lidar_points[degree]

        for degree in range(60, 80):
            if 0.05 < self.lidar_points[degree] < left_min_dist:
                left_min_dist = self.lidar_points[degree]

        for degree in range(240, 260):
            if 0.05 < self.lidar_points[degree] < right_min_dist:
                right_min_dist = self.lidar_points[degree]

        self.front_distance = front_min_dist
        self.left_distance = left_min_dist
        self.right_distance = right_min_dist

    def drive(self, angle, speed):
        if angle is None:
            angle = self.pre_angle
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        self.motor_pub.publish(self.motor_msg)
        self.pre_angle = angle

    def PID(self, left, right, kp, ki, kd):
        end_time = time.time()
        dt = end_time - self.start_time
        self.start_time = end_time

        error = (left * cos(30) + right * cos(30)) / 2 - left
        derror = error - self.prev_error

        p_error = kp * error
        self.i_error = self.i_error + ki * error * dt
        d_error = kd * derror / dt

        output = p_error + self.i_error + d_error
        self.prev_error = error

        if output > 50:
            output = 50
        elif output < -50:
            output = -50

        return 100 * output

    def start(self):

        rospy.init_node('h_drive')
        motor_msg = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1)
        rate = rospy.Rate(30)

        while self.lidar_points is None:
            continue

        while not rospy.is_shutdown():
            if self.left_distance > self.front_distance > self.right_distance:
                angle = -45
                speed = 0
                self.drive(angle, speed)
                print('오른쪽만')
            elif self.left_distance < self.front_distance < self.right_distance:
                angle = 45
                speed = 0
                self.drive(angle, speed)
                print('왼쪽만')
            elif self.front_distance > self.right_distance and self.front_distance > self.left_distance:
                angle = self.PID(self.left_distance, self.right_distance, 0.2, 0.00058, 0.1)
                speed = 0
                print(angle)
                self.drive(angle, speed)
                print('정상')
            else:
                self.drive(self.pre_angle, speed)
                print(self.left_distance)
                print(self.front_distance)
                print(self.right_distance)
                self.pre_angle = angle

            self.rate.sleep()

if __name__ == '__main__':
    turnel_driver = TurnelDriver()
    turnel_driver.start()