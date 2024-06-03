#!/usr/bin/env python

import rospy
import numpy as np
import time
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan

class Lidar:

    S_ru = [0,0]
    S_ld = [0,0]

    def __init__(self, nodeName, topic, queue_size, ld, ru):
        self.topic = topic
        self.queue_size = queue_size
        self.S_ru = ru
        self.S_ld = ld

        rospy.init_node(nodeName, anonymous=False)
        plt.ion()
            
    def sub_start(self):
        print(self.topic, self.queue_size)
        rospy.Subscriber(self.topic, LaserScan, self.callback, queue_size=self.queue_size)

    def detect_degree_func(self, x, y):
        OK = 0

        if (self.S_ld[0] <= x <= self.S_ru[0]) or (self.S_ru[0] <= x <= self.S_ld[0]):
            OK += 1
        if (self.S_ld[1] <= y <= self.S_ru[1]) or (self.S_ru[1] <= y <= self.S_ld[1]):
            OK += 1

        if OK == 2:
            return True

        return False

    def callback(self, data):
        ranges = np.zeros(505, dtype=float)
        ran = data.ranges
                
        lidar_increment = data.angle_increment
        mode = 0

        if len(ran) != 505:
            return

        ranges = ran

        #for i in range(0,253):
        #    ranges[i+252] = ran[i]
        #for i in range(252,504):
        #    ranges[i-252] = ran[i]
  

        X = []
        Y = []
        self.detect_degrees = []

        for i in range(0, 505):
            radian = i * lidar_increment

            x = ranges[i] * np.cos(radian)
            y = -ranges[i] * np.sin(radian)
                
            if self.detect_degree_func(x, y):
                self.detect_degrees.append(np.degrees(radian))
 
            X.append(x)
            Y.append(y)

        X_graph = np.array(Y)
        Y_graph = np.array(X)
        
        xlim = [self.S_ld[0], self.S_ru[0]]
        ylim = [self.S_ld[1], self.S_ru[1]]

        plt.xlim([min(xlim), max(xlim)])
        plt.ylim([min(ylim), max(ylim)])

        # if len(self.detect_degrees) != 0:
            # print(self.detect_degrees)

        dot = plt.scatter(X_graph,Y_graph) 

        plt.show()
        plt.pause(0.001)
        dot.remove()

if __name__ == '__main__':
    lidar = Lidar("viewer", "/scan", 1, [-0.8, -0.8],[0.8,0.8])
    lidar.sub_start()
    rospy.spin()

