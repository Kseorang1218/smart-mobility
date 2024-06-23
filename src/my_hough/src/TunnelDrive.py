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
        # 초기화: 모터, LiDAR 데이터, 거리 변수 설정
        self.motor = None
        self.lidar_points = None
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        # Ctrl+C 시그널 핸들러 설정
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        # Ctrl+C 입력 시 프로그램 종료 처리
        time.sleep(3)
        os.system('killall -9 python rosout')
        sys.exit(0)

    def lidar_callback(self, data):
        # LiDAR 데이터 처리 콜백 함수
        self.lidar_points = data.ranges
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # 전방 거리 계산 (160도~200도)
        for degree in range(160, 200):
            if 0.10 < self.lidar_points[degree] < self.front_distance:
                self.front_distance = self.lidar_points[degree]

        # 좌측 거리 계산 (20도~60도)
        for degree in range(20, 60):
            if 0.10 < self.lidar_points[degree] < self.left_distance:
                self.left_distance = self.lidar_points[degree]

        # 우측 거리 계산 (300도~340도)
        for degree in range(300, 340):
            if 0.10 < self.lidar_points[degree] < self.right_distance:
                self.right_distance = self.lidar_points[degree]

    def drive(self, Angle, Speed):
        # 모터 제어 메시지 발행
        motor_msg = xycar_motor()
        motor_msg.angle = Angle
        motor_msg.speed = Speed
        self.motor.publish(motor_msg)

    def start(self):
        # ROS 토픽 발행자 및 구독자 설정
        self.motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        rate = rospy.Rate(30)

        # LiDAR 데이터가 수신될 때까지 대기
        while self.lidar_points is None:
            continue

        # 메인 주행 로직
        while not rospy.is_shutdown():
            speed = 3
            if self.right_distance == float('inf') or self.right_distance < 0.45:
                # 우측 벽이 없거나 너무 가까울 때
                angle = -45
                print('Right danger')
                if self.front_distance < 0.45:
                    # 전방 장애물이 가까울 때
                    angle = -50
                    print('Front danger')
                
                if self.left_distance < 0.35:
                    # 좌측 벽이 너무 가까울 때
                    angle = 30
                    print('Left danger')
                
            elif self.left_distance > 0.65 and self.right_distance > 0.65:
                # 양쪽 벽이 멀어졌을 때 (터널 탈출)
                print('escape')
                return
            else:
                # 일반적인 주행 상황
                angle = -35

            # 계산된 각도와 속도로 주행
            self.drive(angle, speed)
            rate.sleep()

if __name__ == '__main__':
    tunnel_driver = TunnelDriver()
    tunnel_driver.start()