#!/usr/bin/env python

import gym
import rospy
import numpy as np
import time
from gym import utils, spaces
from geometry_msgs.msg import Pose
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from joint_publisher import JointPub
from monoped_state import MonopedState
from controllers_connection import ControllersConnection

#register the training environment in the gym as an available one
reg = register(
    id='Monoped-v0',
    entry_point='monoped_env:MonopedEnv',
    max_episode_steps=50,
    )


class MonopedEnv(gym.Env):

    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment

        # gets training parameters from param server
        self.desired_pose = Pose()
        self.desired_pose.position.x = rospy.get_param("/desired_pose/x")
        self.desired_pose.position.y = rospy.get_param("/desired_pose/y")
        self.desired_pose.position.z = rospy.get_param("/desired_pose/z")
        self.running_step = rospy.get_param("/running_step")
        self.max_incl = rospy.get_param("/max_incl")
        self.max_height = rospy.get_param("/max_height")
        self.min_height = rospy.get_param("/min_height")
        self.joint_increment_value = rospy.get_param("/joint_increment_value")
        self.done_reward = rospy.get_param("/done_reward")
        self.alive_reward = rospy.get_param("/alive_reward")
        self.desired_force = rospy.get_param("/desired_force")
        self.desired_yaw = rospy.get_param("/desired_yaw")

        self.weight_r1 = rospy.get_param("/weight_r1")
        self.weight_r2 = rospy.get_param("/weight_r2")
        self.weight_r3 = rospy.get_param("/weight_r3")
        self.weight_r4 = rospy.get_param("/weight_r4")
        self.weight_r5 = rospy.get_param("/weight_r5")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.controllers_object = ControllersConnection(namespace="monoped")

        self.monoped_state_object = MonopedState(   max_height=self.max_height, min_height=self.min_height,
                                                    abs_max_roll=self.max_incl, abs_max_pitch=self.max_incl,
                                                    joint_increment_value=self.joint_increment_value, done_reward=self.done_reward,
                                                    alive_reward=self.alive_reward, desired_force=self.desired_force,
                                                    desired_yaw=self.desired_yaw, weight_r1=self.weight_r1,
                                                    weight_r2=self.weight_r2, weight_r3=self.weight_r3,
                                                    weight_r4=self.weight_r4, weight_r5=self.weight_r5
                                                )

        self.monoped_state_object.set_desired_world_point(self.desired_pose.position.x, self.desired_pose.position.y, self.desired_pose.position.z)

        self.monoped_joint_pubisher_object = JointPub()
        


        """
        We have 6 actions, labeled 1, 2, 3, 4, 5, 6
        axn 1-2: increment/decrement haa_joint
        axn 3-4: increment/decrement hfe_joint
        axn 5-6: increment/decrement kfe_joint
        """
        self.action_space = spaces.Discrete(6)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # Resets the state of the environment and returns an initial observation.
    def _reset(self):

        # pause the Simulator
        rospy.logdebug("Pausing the sim...")
        self.gazebo.pauseSim()

        #reset the simulation to initial values
        rospy.logdebug("Resetting the sim...")
        self.gazebo.resetSim()

        # We Set the gravity to 0.0 so that we dont fall when reseting joints
        # It also UNPAUSES the simulation
        rospy.logdebug("removing the gravity")
        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        rospy.logdebug("resetting the monoped_joint_controllers...")
        self.controllers_object.reset_monoped_joint_controllers()

        # 3rd: resets the robot to initial conditions
        rospy.logdebug("set initial pose")
        self.monoped_joint_pubisher_object.set_init_pose()

        # Check all subscribers work.
        # Get the state of the Robot defined by its RPY orientation, distance from
        # desired point, contact force and JointState of the three joints
        rospy.logdebug("check_all_systems_ready...")
        self.monoped_state_object.check_all_systems_ready()
        rospy.logdebug("get_observations...")
        observation = self.monoped_state_object.get_observations()

        # We restore the gravity to original
        rospy.logdebug("restoring gravity")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        # pauses simulation
        rospy.logdebug("pause the sim")
        self.gazebo.pauseSim()

        # Get the State Discrete Stringuified version of the observations
        state = self.get_state(observation)

        return state

    def _step(self, action):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot

        # [ocl] which action corresponsd to which joint is incremented
        next_action_position = self.monoped_state_object.get_action_to_position(action)

        # We move the joint to the position
        self.gazebo.unpauseSim()
        self.monoped_joint_pubisher_object.move_joints(next_action_position)
        # Then we send the command to the robot and let it go
        # for running_step seconds
        time.sleep(self.running_step)
        self.gazebo.pauseSim()

        # We now process the latest data saved in the class state to calculate
        # the state and the rewards. This way we guarantee that they work
        # with the same exact data.
        # Generate State based on observations
        observation = self.monoped_state_object.get_observations()

        # finally we get an evaluation based on what happened in the sim
        reward,done = self.monoped_state_object.process_data()

        # Get the State Discrete Stringuified version of the observations
        state = self.get_state(observation)

        return state, reward, done, {}

    def get_state(self, observation):
        """
        We retrieve the Stringuified-Discrete version of the given observation
        :return: state
        """
        return self.monoped_state_object.get_state_as_string(observation)

    def _render(self, mode='human', close=False):
        reward, done = self.monoped_state_object.process_data()
        print("here's the reward: ")
        print(reward)