import rospy
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
import rospkg
import time
from math import *
import signal
import sys
import os

class TunnelDriver:
    def __init__(self):
        self.motor = None
        self.pre_angle = 0
        self.lidar_points = None
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None
        self.i_error = 0.0
        self.prev_error = 0.0
        self.start_time = time.time()

    def signal_handler(self, sig, frame):
        time.sleep(3)
        os.system('killall -9 python rosout')
        sys.exit(0)

    def lidar_callback(self, data):
        self.lidar_points = data.ranges
        front_dist = 0
        left_dist = 0
        right_dist = 0
        num_data_f = 0
        num_data_r = 0
        num_data_l = 0
        
        for degree in range(170, 190):
            if 0.0 < self.lidar_points[degree] < float('inf'):
                front_dist += self.lidar_points[degree]
                num_data_f += 1

        if num_data_f != 0:
            self.front_distance = front_dist / num_data_f
        else:
            self.front_distance = 1.0  # or some default value

        for degree in range(50, 70):
            if 0.0 < self.lidar_points[degree] < float('inf'):
                left_dist += self.lidar_points[degree]
                num_data_l += 1

        if num_data_l != 0:
            self.left_distance = left_dist / num_data_l
        else:
            self.left_distance = 1.0   # or some default value

        for degree in range(230, 250):
            if 0.0 < self.lidar_points[degree] < float('inf'):
                right_dist += self.lidar_points[degree]
                num_data_r += 1

        if num_data_r != 0:
            self.right_distance = right_dist / num_data_r
        else:
            self.right_distance = 1.0   # or some default value

    def drive(self, Angle, Speed):
        if Angle is None:
            Angle = self.pre_angle
        motor_msg = xycar_motor()
        motor_msg.angle = Angle
        motor_msg.speed = Speed
        print(motor_msg)
        self.motor.publish(motor_msg)
        self.pre_angle = Angle

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
        self.motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)

        signal.signal(signal.SIGINT, self.signal_handler)

        while self.lidar_points is None:
            continue

        while not rospy.is_shutdown():
            if self.left_distance > self.front_distance > self.right_distance:
                angle = -50  # 핸들조향각 값
                speed = 3  # 차량속도 값
                self.drive(angle, speed)
                print('오른쪽만')
            elif self.left_distance < self.front_distance < self.right_distance:
                angle = 50  # 핸들조향각 값
                speed = 3  # 차량속도 값
                self.drive(angle, speed)
                print('왼쪽만')
            else:
                angle = self.PID(self.left_distance, self.right_distance, 0.28, 0.00058, 0.1)  # 핸들조향각 값
                speed = 3  # 차량속도 값
                print(angle)
                self.drive(angle, speed)
                print('정상')

if __name__ == '__main__':
    control = TunnelDriver()
    control.start()