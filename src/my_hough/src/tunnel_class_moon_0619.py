#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import signal
import sys
import os

# Constants
CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250 
ROI_HEIGHT = HEIGHT - ROI_ROW 
L_ROW = ROI_HEIGHT - 120 # Row for position detection
image = np.empty(shape=[0])
bridge = CvBridge()
img_ready = False

def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True

class TunnelDriver:
    def __init__(self):
        self.motor = None
        self.lidar_points = None
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.CAM_FPS = 30
        self.WIDTH, self.HEIGHT = 640, 480
        self.ROI_ROW = 250 
        self.ROI_HEIGHT = self.HEIGHT - self.ROI_ROW 
        self.L_ROW = self.ROI_HEIGHT - 120 # Row for position detection
        self.image = np.empty(shape=[0])
        self.bridge = CvBridge()
        self.img_ready = False

    def signal_handler(self, sig, frame):
        time.sleep(3)
        os.system('killall -9 python rosout')
        sys.exit(0)

    def img_callback(self, data):        
        self.image = bridge.imgmsg_to_cv2(data, "bgr8")
        self.img_ready = True

    def get_edge_img(self, img):
    
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.blur_gray = cv2.GaussianBlur(self.gray, (5, 5), 0)
        self.edge_img = cv2.Canny(np.uint8(self.blur_gray), 30, 60)
        return self.edge_img
        
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
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
        self.timecnt_sub = rospy.Subscriber("time_count", Int64, time_callback)

        while self.lidar_points is None:
            continue

        while not rospy.is_shutdown():
            if self.right_distance == float('inf') or self.right_distance < 0.35:
                angle = -50
                speed = 3
                if self.front_distance < 0.7:
                    angle = -50
                    # print('Front danger')
                self.drive(angle, speed)
                if self.left_distance < 0.4 :
                    angle = -50
                # print('Right danger')

            elif self.left_distance < 0.35:
                angle = 50
                speed = 3
                if self.front_distance < 0.7:
                    angle = 50
                    # print('Front danger')
                self.drive(angle, speed)
                if self.right_distance < 0.4 :
                    angle = 45
                # print('Left danger')

            else:
                self.drive(0, 3)
                
            if flag:
                return
            
             





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
