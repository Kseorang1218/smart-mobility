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

        for degree in range(20, 50):
            if 0.10 < self.lidar_points[degree] < self.left_distance:
                self.left_distance = self.lidar_points[degree]

        for degree in range(310, 340):
            if 0.10 < self.lidar_points[degree] < self.right_distance:
                self.right_distance = self.lidar_points[degree]

    # for degree in range(160,200):
    #     if 0.10< lidar_points[degree] < front_distance:
    #         front_distance = lidar_points[degree]

    # for degree in range(15,30):
    #     if 0.10< lidar_points[degree] < left_distance:
    #             left_distance = lidar_points[degree]

    # for degree in range(290,300):
    #     if 0.10< lidar_points[degree] < right_distance:
    #             right_distance = lidar_points[degree]




    def drive(self, Angle, Speed):
        motor_msg = xycar_motor()
        motor_msg.angle = Angle
        motor_msg.speed = Speed
        self.motor.publish(motor_msg)

    def start(self):
        # rospy.init_node('drive')
        self.motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        rate = rospy.Rate(30)

        while self.lidar_points is None:
            continue

        while not rospy.is_shutdown():
            if self.right_distance == float('inf') or self.right_distance < 0.3:
                angle = -45
                speed = 3
                if self.front_distance < 0.65:
                    angle = -50
                    # print('Front danger')
                self.drive(angle, speed)
                if self.left_distance < 0.3 :
                    print("tunnel end")
                    return
                # print('Right danger')

            elif self.left_distance < 0.3:
                angle = 45
                speed = 3
                if self.front_distance < 0.7:
                    angle = 50
                    # print('Front danger')
                self.drive(angle, speed)
                if self.right_distance < 0.4 :
                    print("tunnel end")
                    return
                # print('Left danger')

            else:
                self.drive(0, 3)

       
             





                # print('Normal')

            # if self.right_distance == float('inf') and self.front_distance == float('inf') and self.left_distance == float('inf'):
            #     print("Exiting tunnel driver")
            #     return
                

            #         if self.right_distance == float('inf') or self.right_distance < 0.4:
            #     angle = -45
            #     speed = 3
            #     if self.front_distance < 0.65:
            #         angle = -50
            #         # print('Front danger')
            #     self.drive(angle, speed)
            #     # print('Right danger')

            # elif self.left_distance < 0.5:
            #     angle = 50
            #     speed = 3
            #     # if self.front_distance < 0.7:
            #     #     angle = 50
            #         # print('Front danger')
            #     self.drive(angle, speed)
            #     # print('Left danger')

            # else:
            #     self.drive(0, 3)
            #     # print('Normal')


            # if self.front_distance != float('inf') and self.front_distance > 3.5:
            #     print("Exiting tunnel driver")
            #     return
            # # if self.right_distance == float('inf') and self.front_distance == float('inf') and self.left_distance == float('inf'):
            # #     print("Exiting tunnel driver")
            # #     return
            # print('L', self.left_distance)
            # print('F', self.front_distance)
            # print('R', self.right_distance)
            rate.sleep()

if __name__ == '__main__':
    tunnel_driver = TunnelDriver()
    tunnel_driver.start()
