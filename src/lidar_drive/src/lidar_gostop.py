#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

obstacle_detected_time = 0
motor_msg = xycar_motor()
lidar_points = None
meter = 0.8

def callback(data):
   global lidar_points, motor_msg
   lidar_points = data.ranges

def drive_go():
   global motor_msg
   motor_msg.speed = 4
   motor_msg.angle = 0
   pub.publish(motor_msg)

def drive_aviod():
    global motor_msg
    motor_msg.speed = 4    
    motor_msg.angle = 45
    for i in range(10):
        pub.publish(motor_msg)
        
        time.sleep(0.2)

    motor_msg.angle = -45
    for i in range(14):
        pub.publish(motor_msg)
        time.sleep(0.2)

    motor_msg.angle = 45
    for i in range(7):
        pub.publish(motor_msg)
        time.sleep(0.2)

    motor_msg.angle = 0
    while(True):
        pub.publish(motor_msg)
        



def drive_stop():
   global motor_msg
   motor_msg.speed = 0
   motor_msg.angle = 0
   pub.publish(motor_msg)


rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1)

while lidar_points is None:
    continue

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    ok = 0
    for degree in range(0,46):
        if (0.01 < lidar_points[180 + degree] <= meter):
            ok += 1
        if (0.01 < lidar_points[180 - degree] <= meter):
            ok += 1
    if ok > 5:
        drive_stop()
        obstacle_detected_time = time.time()
        break
    else:
        drive_go()
    #rate.sleep()
    
ok_2 = 0        

while(True):
    if time.time() - obstacle_detected_time < 5:
    
        for degree in range(0,46):
            if (0.01 < lidar_points[180 + degree] <= meter):
                ok_2 += 1
            if (0.01 < lidar_points[180 - degree] <= meter):
                ok_2 += 1
        if ok_2 > 5:
            drive_stop()
            ok_2 = 0
            continue
        else: 

            break

    else:
        #ok_3 = 0
        #for degree in range(0,46):
        #    if (0.01 < lidar_points[180 + degree] <= meter):
        #        ok_3 += 1
        #    if (0.01 < lidar_points[180 - degree] <= meter):
        #        ok_3 += 1
        #if ok_3 > 5:
        drive_aviod()
        #drive_go() 

while(True):
    drive_go()

rate.sleep()
    # else:
    #     drive_go()



# #!/usr/bin/env python

# import rospy, time
# from sensor_msgs.msg import LaserScan
# from xycar_msgs.msg import xycar_motor

# motor_msg = xycar_motor()
# lidar_points = None

# def callback(data):
#    global lidar_points, motor_msg
#    lidar_points = data.ranges

# def drive_go():
#    global motor_msg
#    motor_msg.speed = 5
#    motor_msg.angle = 0
#    pub.publish(motor_msg)

# def drive_stop():
#    global motor_msg
#    motor_msg.speed = 0
#    motor_msg.angle = 0
#    pub.publish(motor_msg)


# rospy.init_node('lidar_driver')
# rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
# pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1)

# while lidar_points is None:
#     continue

# rate = rospy.Rate(5)
# while not rospy.is_shutdown():
#     ok = 0
#     for degree in range(0,46):
#         if (0.01 < lidar_points[ 180 + degree] <= 0.3):
#             ok += 1
#         if (0.01 < lidar_points[ 180 - degree] <= 0.3):
#             ok += 1
#         if ok > 5:
#             drive_stop()
#             break

#     if ok <= 5:
#         drive_go()

#     rate.sleep()