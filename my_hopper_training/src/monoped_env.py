"""TODO: INCOMPLETE """"
import gym
import rospy
import numpy as np
import time
from gym import utils, spaces
from geometry_msgs.msg import Pose #poses are a message type
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from joint_publisher import JointPub
from monoped_state import MonopedState
from controllers_connection import ControllersConnection

#register training environment in gym 
reg = register(
    id='Monoped-v0',
    entry_point = 'monoped_env::MonopedEnv'
    timestep_limit = 50,
)

class MonopedEnv(gym.Env): #MonopedEnv has a parent class of the gym.env
    def __init__(self):
        ##we assume the ROS node is already created before init

        #get training parameters from yaml file
        print("here are paramters: ")

        #these are all in the /config/qlearn_params.yaml
        
        #here we discuss the pose
        self.desired_pose = Pose()
        self.desired_pose.position.x = rospy.get_param("/desired_pose/x")
        self.desired_pose.position.y = rospy.get_param("/desired_pose/y")
        self.desired_pose.position.z = rospy.get_param("/desired_pose/z")

        #here we initialize the running step
        self.desired_force = rospy.get_param("/desired_force")
        self.desired_yaw = rospy.get_param("/desired_yaw")
        self.max_height = rospy.get_param("/max_height")
        self.min_height = rospy.get_param("/min_height")
        self.max_incl = rospy.get_param("/max_incl")
        self.running_step = rospy.get_param("/running_step")
        self.joint_increment_value = rospy.get_param("/joint_incement_value")
        
        #our reward values
        self.done_reward = rospy.get_param("/done_reward")
        self.alive_reward = rospy.get_param("/alive_reward")

        #weight params
        self.weight_r1 = rospy.get_param("/weight_r1")
        self.weight_r2 = rospy.get_param("/weight_r2")
        self.weight_r3 = rospy.get_param("/weight_r3")
        self.weight_r4 = rospy.get_param("/weight_r4")
        self.weight_r5 = rospy.get_param("/weight_r5")

        #connect to the simulator from gazebo_connection.py
        self.gazebo = GazeboConnection()

        self.controllers_object =  ControllersConnection(namespace="monoped")