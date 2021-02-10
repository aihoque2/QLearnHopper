#!/usr/bin/env python

import gym
import time
import numpy as np
import random
import qlearn
from gym import wrappers

#ROS packages
from std_msgs import Float64
import rospy
import rospkg

#training environment
import monoped_env

if __name__ == '__main__':
    
    rospy.init('monoped_gym', anonymous=True, log_level=rospy.INFO)

    #create gym env
    env = gym.make('Monoped-v0')
    rospy.logdebug("Gym env created")

    #we create a publisher that tells the robot the reward it receives for an action at a state
    reward_pub = rospy.Publisher('/monoped/reward', Float64, queue_size=1)

    #this is the publisher that tells the robot Q_local reward
    episode_reward_pub = rospy.Publisher('/monoped/episode_reward', Float64, queue_size=1)


    #logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('my_hopper_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.monitor(env, outdir, force=True)
    rospy.logdebug("Monitor Wrapper started")

    last_time_steps = np.ndarry(0)


    # load our paramters from the yaml
    alpha = rospy.get_param("/alpha")
    epsilon = rospy.get_param("/epsilon")
    gamma = rospy.get_param("/gamma")
    epsilon_discount = rospy.get_param("/epsilon_discount")
    nepisodes = rospy.get_param("/nepisodes")
    nsteps = rospy.get_param("/nsteps")