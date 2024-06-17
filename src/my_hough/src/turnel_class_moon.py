#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

class LidarController:
    def __init__(self):
        rospy.init_node('drive')
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        self.lidar_points = None
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.rate = rospy.Rate(30)

    def lidar_callback(self, data):
        self.lidar_points = data.ranges
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        for degree in range(160, 200):
            if 0.10 < self.lidar_points[degree] < self.front_distance:
                self.front_distance = self.lidar_points[degree]

        for degree in range(15, 30):
            if 0.10 < self.lidar_points[degree] < self.left_distance:
                self.left_distance = self.lidar_points[degree]

        for degree in range(290, 300):
            if 0.10 < self.lidar_points[degree] < self.right_distance:
                self.right_distance = self.lidar_points[degree]

    def drive(self, angle, speed):
        motor_msg = xycar_motor()
        motor_msg.angle = angle
        motor_msg.speed = speed
        self.motor_pub.publish(motor_msg)

    def start(self):
        while self.lidar_points is None:
            continue

        while not rospy.is_shutdown():
            if self.right_distance == float('inf') or self.right_distance < 0.4:
                angle = -45
                speed = 3
                if self.front_distance < 0.65:
                    angle = -50
                    print('전방 위험')
                self.drive(angle, speed)
                print('오른쪽 위험')
            elif self.left_distance < 0.4:
                angle = 45
                speed = 3
                if self.front_distance < 0.7:
                    angle = 50
                    print('전방 위험')
                self.drive(angle, speed)
                print('왼쪽 위험')
            else:
                self.drive(0, 3)
                print('정상')

            print('L', self.left_distance)
            print('F', self.front_distance)
            print('R', self.right_distance)
            self.rate.sleep()

if __name__ == '__main__':
    lidar_controller = LidarController()
    lidar_controller.start()