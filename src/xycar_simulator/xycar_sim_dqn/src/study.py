#!/usr/bin/env python
# -*- coding: utf-8 -*-

import my_reward
import rospy
from env.xycarRL import *
from env.visual import *

def string2list(ListShapeString):
    ListShapeString = ListShapeString.replace(' ',''). \
                           replace('[',''). \
                           replace(']',''). \
                           split(",")

    returnList = []
    for i in ListShapeString:
        returnList.append(int(i))

    return returnList

rospy.init_node("dqn_study_demo")
xycar = learning_xycar()

view_epi = rospy.get_param('~save_epi', 0)
sel_map = rospy.get_param('~map_name', "snake")
pose_random = rospy.get_param('~pose_random', True)
lidar_laser_cnt = rospy.get_param('~lidar_laser_cnt', 7)
car_sensor = rospy.get_param('~car_sensor', True)
car_yaw = rospy.get_param('~car_yaw', False)
car_position = rospy.get_param('~car_position', False)
car_steer = rospy.get_param('~car_steer', True)
learning_rate = rospy.get_param('~learning_rate', 0.002)
discount_factor = rospy.get_param('~discount_factor', 0.98)
optimizer_steps = rospy.get_param('~optimizer_steps', 50)
batch_size = rospy.get_param('~batch_size', 32)
min_history = rospy.get_param('~min_history', 32)
buffer_limit = rospy.get_param('~buffer_limit', 100000)
max_episode = rospy.get_param('~max_episode', 9999999)
update_cycle = rospy.get_param('~update_cycle', 25)
model_type = rospy.get_param('~model_type', "DQN")
hidden_layer_str = rospy.get_param('~hidden_layer', '[1024,1024]')
hidden_layer = string2list(hidden_layer_str)

xycar.set_map(sel_map) # snake, square

hyper_param = {
    "sensor_num" : lidar_laser_cnt,
    "learning_rate" : learning_rate,
    "discount_factor" : discount_factor,
    "optimizer_steps" : optimizer_steps,
    "batch_size" : batch_size,
    "min_history" : min_history,
    "buffer_limit" : buffer_limit,
    "max_episode" : max_episode,
    "update_cycle" : update_cycle,
    "hidden_size" : hidden_layer
}

xycar.set_hyperparam(hyper_param)

state_select = {
    "car sensor" : car_sensor,
    "car yaw" : car_yaw,
    "car position" : car_position,
    "car steer" : car_steer
}

xycar.state_setup(state_select)
xycar.Experience_replay_init()
xycar.ML_init(model_type) # "DDQN" "Duel DQN"

xycar.set_init_location_pose_random(pose_random) 

visual = visualize(port=8888)
visual.chart_init()

ALL_STEPS, ALL_REWARD = 0, 0
episode = 0

while (0 <= episode <= int(hyper_param["max_episode"])) and (not rospy.is_shutdown()):
    try:
        episode += 1
        epsilon = max(0.01, 0.2 - 0.01*(float(episode)/150.0))
        xycar.set_E_greedy_func(epsilon)

        state = xycar.episode_init()

        reward, score = 0, 0.0

        while True:
            action = xycar.get_action(state)
            next_state = xycar.step(action)

            if xycar.get_episode_done():
                for_reward = [xycar.get_round_count()]
                reward += my_reward.reward_end_game(for_reward)

                xycar.Experience_replay_memory_input(state, action, next_state, reward)
                break

            for_reward = [xycar.get_sensor_value()]
            reward = my_reward.reward_in_game(for_reward)
            
            xycar.Experience_replay_memory_input(state, action, next_state, reward)
            
            state = next_state
            score += reward

        ALL_STEPS += xycar.get_step_number()
        ALL_REWARD += score

        if xycar.get_max_score() < score:
            xycar.max_score_update(score)
            xycar.model_save(episode)
            xycar.making_video(episode, score)

        if xycar.get_memory_size() > hyper_param["min_history"]:
            loss = xycar.train(hyper_param["optimizer_steps"])
            visual.loss_graph_update(episode, loss)

        visual.dead_position_update(xycar.get_xycar_position())
        visual.reward_update(episode, score)
        visual.learning_curve_update(episode, ALL_REWARD)

        if (xycar.get_memory_size() > hyper_param["min_history"]) and ((episode % hyper_param["update_cycle"]) == 0) and (episode != 0):
            xycar.mainQ2targetQ()

        if (episode % 10 == 0) and (episode != 0):
            print("episode :{}, memory size : {}, epsilon : {:.1f}%".format(episode, xycar.get_memory_size(), epsilon*100))

    except KeyboardInterrupt:
        visual.exit_ready()
        break

xycar.model_save(episode)
xycar.making_video(episode, score)
sys.exit()
