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
    entry_point = 'monoped_env::MonopedEnv',
    max_episode_steps = 50,
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

        self.controllers_object = ControllersConnection(namespace="monoped")

        self.monoped_state_object = MonopedState(max_height = self.max_height, min_height=self.min_height, abs_max_roll = self.max_incl,
                                                    abs_max_pitch = self.max_incl, joint_increment_value=self.joint_increment_value,
                                                    done_reward=self.done_reward, alive_reward = self.alive_reward, desired_force=self.desired_force,
                                                    desired_yaw=self.desired_yaw, weight_r1 = self.weight_r1, weight_r2 = self.weight_r2,
                                                    weight_r3 = self.weight_r3, weight_r4 = self.weight_r4, weight_r5 = self.weight_r5)
        
        self.monoped_state_object.set_starting_pont(self.desired_pose.position.x, self.desired_pose.position.y, self.desired_pose.position.z)

        self.monoped_joint_publisher_object = JointPub()

        """
        We have 6 actions, labeled 1, 2, 3, 4, 5, 6
        axn 1-2: increment/decrement haa_joint
        axn 3-4: increment/decrement hfe_joint
        axn 5-6: increment/decrement kfe_joint
        """

        self.action_space = spaces.Discrete(6)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def _seed(self, seed=None): #random generator for picking an action
        self.np_random, seed = np.seeding.np_random(seed)
        return [seed]

    def _reset(self):

        #pause the simulator
        rospy.logdebug("Pausing the sim")
        self.gazebo.pauseSim()

        #reset the simulator
        rospy.logdebug("resetting the sim")
        self.gazebo.resetSim()

        #unpause the sim and set the gravity to zero
        rospy.logdebug("removing the gravity")
        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        #reset joint state controllers
        rospy.logdebug("resetting monoped joint controllers")
        self.controllers_object.reset_monoped_joint_controllers()

        rospy.logdebug("set init pose")
        self.monoped_joint_publisher_object.set_init_pose()

        #check all the subscribers work
        rospy.logdebug("check all systems ready...")
        self.monoped_state_object.check_all_systems_ready()
        rospy.logdebug("get observations...")
        observation = self.monoped_state_object.get_observations()

        #restore gravity back to -9.81
        rospy.logdebug("restoring gravity")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        #pause simulation
        rospy.logdebug("pause the sim")
        self.gazebo.pauseSim()


        state = self.get_state(observation)

        return state

    def _step(self, action):

        # given action selected by learning algo,
        # perform corresponding movement of the robot

        # pick which action corresponds to which joint to move
        next_action_position = self.monoped_state_object.get_action_to_position(action)
        
        # move the joint to the position
        self.gazebo.unPauseSim()
        self.monoped_joint_publisher_object.move_joints(next_action_position)

        #send it to the robot
        observation = self.monoped_state_object.get_observations()

        #finally evaluate based on what happened to the sim
        reward, done = self.monoped_state_object.process_data()

        state = self.get_state(observation)

        return state, reward, done, {}

    def get_state(self, observation):
        """
        Helper for _step
        retrieve discretized version of the given state (or observation)
        """

        return self.monoped_state_object.get_state_as_string(observation)