#!/usr/bin/env python

import rospy
from gazebo_msgs import ContactState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs import JointState
import tf #transformations, not TensorFlow
import numpy as np
import math


class MonopedState(object):

    def __init__(self, max_height, min_height, abs_max_roll, abs_max_pitch, joint_increment_value = 0.05, done_reward = -1000.0, alive_reward=10.0, desired_force=7.08, desired_yaw=0.0, weight_r1=1.0, weight_r2=1.0, weight_r3=1.0, weight_r4=1.0, weight_r5=1.0, discrete_division=10):
        
        rospy.logdebug("Starting MonopedState object...")
        self.starting_point = Vector3(0.0, 0.0, 0.0)
        self.max_height = max_height
        self.min_height = min_height
        self.abs_max_roll = abs_max_roll
        self.abs_max_pitch = abs_max_pitch
        self.joint_increment_value = joint_increment_value
        self.done_reward = done_reward
        self.alive_reward = alive_reward
        self.desired_force = desired_force
        self.desired_yaw = desired_yaw

        self.weight_r1 = weight_r1
        self.weight_r2 = weight_r2
        self.weight_r3 = weight_r3
        self.weight_r4 = weight_r4
        self.weight_r5 = weight_r5

        self.list_of_observations = ["distance_from_desired_point",
                 "base_roll",
                 "base_pitch",
                 "base_yaw",
                 "contact_force",
                 "joint_states_haa",
                 "joint_states_hfe",
                 "joint_states_kfe"]
        
        self.discrete_division = discrete_division

        self.init_bins() ##check the helper

        #initialize the different state vars for the MonopedState
        self.base_position = Point()
        self.base_orientation = Quaternion() #quaternions for rotation
        self.base_linear_acceleration = Vector3()
        self.contact_force = Vector3()
        self.joints_state = JointState()

        #use odom for height detection and planar position
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        #use IMU for orientation and linear acceleration detection
        rospy.Subscriber("/monoped/imu/data", Imu, self.imu_callback)

        #use contact state to see wether the hopper is in the air or on the ground
        rospy.Subscriber("/lowerleg_contact_sensor_state", ContactState, self.contact_callback)

        #use JoinState to get the joint positions and calculate reward associated with it
        rospy.Subscriber("/monoped/joint_states", JointState, self.joint_state_callback)


    def check_all_systems_ready(self):
        """as the title of the function says"""
        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                imu_data = rospy.wait_for_message("/monoped/imu/data", Imu, timeout=0.1)
                self.base_orientation = imu_data.orientation
                self.base_linear_acceleration = imu_data.base_linear_acceleration
                rospy.logdebug("current imu_data READY")

            except:
                rospy.logdebug("Current imu_data not ready yet, retrying for getting robot base_orientation, and base_linear_acceleration")
        

        contacts_data = None
        while contacts_data is None and not rospy.is_shutdown():
            try:
                contacts_data = rospy.wait_for_message("/lowerleg_contact_sensor_state", ContactState, timeout=0.1)
                for state in contacts_data.state:
                    self.contact_force = state.total_wrench.force
                rospy.logdebug("Current contacts_data READY")
            
            except:
                rospy.logdebug("Current contacts_data not ready yet. retrying...")

        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message("/monoped/joint_states", JointState, timeout=0.1)
                self.joints_state = joint_states_msg
                rospy.logdebug("Current joint_states READY")
            except:
                rospy.logdebug("Current joint_states not ready yet, retrying==>"+str(e))

        rospy.logdebug("ALL SYSTEMS READY")

        def set_starting_point(self, x, y, z):
            """set the monoped at the point you want it to be"""
            self.starting_point.x = x
            self.starting_point.y = y
            self.starting_point.z = z


        def get_base_height(self):
            return self.base_position.z
        
        def get_base_rpy(self):
            euler_rpy=Vector3()
            euler = tf.transformations.euler_from_quaternion([self.base_orientation.x, self.base_orientation.y, self.base_orientation.z, self.base_orientation.w])

            euler_rpy.x = euler[0]
            euler_rpy.y = euler[1]
            euler_rpy.z = euler[2]
            return euler.rpy()

        def get_distance_from_point(self, some_point):
            """given Vector3() somePoint, calculate distance from it""""

            a = np.array((self.base_position.x, self.base_position.y, self.base_position.z)) #we pass in a tuple because it is ordered
            b = np.array((some_point.x, some_point.y, some_point.z))
            distance = np.linalg.norm(a - b)

            return distance
        
        def get_contact_force_magnitude(self):
            contact_force = np.array((self.contact_force.x, self.contact_force.y, self.contact_force.z))
            force_magnitude = np.linalg.norm(contact_force)
            return force_magnitude
        
        def get_joint_states(self):

        
        def odom_callback(self):
        
        
        def imu_callback(self):


        def contact_callback(self):


        def joints_state_callback(self):

        
        def monoped_height_ok(self):

        
        def monoped_orientation_ok(self):

        
        def calculate_reward_joint_position(self, weight=1.0):

        
        def calculate_reward_joint_effort(self, weight=1.0):


        def calculate_reward_contact_force(self, weight=1.0):

        
        def calculate_reward_orientation(self, weight=1.0):


        def calculate_reward_distance_from_des_point(self, weight=1.0):


        def calculate_total_reward(self):


        def get_observations(self):


        def get_state_as_string(self):


        def assign_bins(self, observation):


        def init_bins(self):

        
        def fill_observation_ranges(self):


        def create_bins(self):


        def get_action_to_position(self, action):

        
        def process_data(self):


        def testing_loop(self):


if __name__ == "__main__":
    rospy.init_node("monoped_state_node", anonymous=True)
    
    monoped_state = MonopedState(max_height=3.0,
                                min_height=0.6,
                                abs_max_roll=0.7
                                abs_max_pitch=0.7)
    
    monoped_state.testing_loop()