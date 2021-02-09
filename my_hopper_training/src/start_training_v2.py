#!/usr/bin/env python


import gym
import time
import numpy
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
     