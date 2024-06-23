#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
import time
import signal
import sys
import os

class TunnelDriver:
    def __init__(self):
        self.motor = None
        self.lidar_points = None
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        time.sleep(3)
        os.system('killall -9 python rosout')
        sys.exit(0)

    def lidar_callback(self, data):
        self.lidar_points = data.ranges
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        for degree in range(160, 200):
            if 0.10 < self.lidar_points[degree] < self.front_distance:
                self.front_distance = self.lidar_points[degree]

        for degree in range(20, 60):
            if 0.10 < self.lidar_points[degree] < self.left_distance:
                self.left_distance = self.lidar_points[degree]

        for degree in range(300, 340):
            if 0.10 < self.lidar_points[degree] < self.right_distance:
                self.right_distance = self.lidar_points[degree]

    def drive(self, Angle, Speed):
        motor_msg = xycar_motor()
        motor_msg.angle = Angle
        motor_msg.speed = Speed
        self.motor.publish(motor_msg)

    def start(self):
        self.motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        rate = rospy.Rate(30)

        while self.lidar_points is None:
            continue

        while not rospy.is_shutdown():
            speed = 3
            if self.right_distance == float('inf') or self.right_distance < 0.45:
                angle = -45
                print('Right danger')
                if self.front_distance < 0.45: 
                    angle = -50
                    print('Front danger')
                

                if self.left_distance < 0.35 :
                    angle = 30
                    print('Left danger')
                    
            elif self.left_distance > 0.65 and self.right_distance > 0.65:
                print('escape')
                return

            else: 
                angle = -35

            self.drive(angle, speed)
            rate.sleep()

if __name__ == '__main__':
    tunnel_driver = TunnelDriver()
    tunnel_driver.start()