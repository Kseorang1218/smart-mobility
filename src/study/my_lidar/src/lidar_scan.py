#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan

lidar_points = None

def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges

rospy.init_node('Lidar', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

while not rospy.is_shutdown():
    if lidar_points == None:
        continue
    
    rtn = ""
    for i in range(12):
        rtn += str(format(lidar_points[i*42+1],'.2f')) + ", "

    print(rtn[:-2])
    time.sleep(0.5)
    
