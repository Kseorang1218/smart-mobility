#!/usr/bin/env python

from env.xycarRL import *
import signal, sys, os, time, rospy

def string2list(ListShapeString):
    ListShapeString = ListShapeString.replace(' ',''). \
                           replace('[',''). \
                           replace(']',''). \
                           split(",")

    returnList = []
    for i in ListShapeString:
        returnList.append(int(i))

    return returnList

rospy.init_node("dqn_demo")

view_epi = rospy.get_param('~save_epi', 10271)
sel_map = rospy.get_param('~map_name', "snake")
pose_random = rospy.get_param('~pose_random', True)
lidar_laser_cnt = rospy.get_param('~lidar_laser_cnt', 7)
car_sensor = rospy.get_param('~car_sensor', True)
car_yaw = rospy.get_param('~car_yaw', False)
car_position = rospy.get_param('~car_position', False)
car_steer = rospy.get_param('~car_steer', True)
model_type = rospy.get_param('~model_type', "DQN")
hidden_layer_str = rospy.get_param('~hidden_layer', '[1024,1024]')
hidden_layer = string2list(hidden_layer_str)

xycar = learning_xycar(False)
xycar.set_map(sel_map) # snake, square

xycar.pygame_init()

xycar.set_lidar_cnt(lidar_laser_cnt)
xycar.set_hidden_size(hidden_layer)

state_select = {
    "car sensor" : car_sensor,
    "car yaw" : car_yaw,
    "car position" : car_position,
    "car steer" : car_steer
}

xycar.state_setup(state_select)

xycar.screen_init()
xycar.ML_init(model_type)

xycar.set_init_location_pose_random(True) 
xycar.load_model(view_epi)

time.sleep(0.5)

while (not xycar.pygame_exit) and (not rospy.is_shutdown()):
    state = xycar.episode_init()

    while (xycar.get_episode_done()) or (not xycar.pygame_exit):
        xycar.pygame_exit_chk()
        xycar.calibration_time()
        action = xycar.get_action_viewer(state)
        next_state = xycar.step(action)

        if xycar.get_episode_done():
            break

        state = next_state
        xycar.display_flip()
        
