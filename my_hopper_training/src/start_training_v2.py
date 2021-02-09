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
    
    rospy.init('monoped_gym', anonymous=True, log_level=)