#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
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
        rospy.Subscriber("/lowerleg_contact_sensor_state", ContactsState, self.contact_callback)

        #use JoinState to get the joint positions and calculate reward associated with it
        rospy.Subscriber("/monoped/joint_states", JointState, self.joints_state_callback)


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
        return euler_rpy

    def get_distance_from_point(self, some_point):
        """given Vector3() somePoint, calculate distance from it"""

        a = np.array((self.base_position.x, self.base_position.y, self.base_position.z)) #we pass in a tuple because it is ordered
        b = np.array((some_point.x, some_point.y, some_point.z))
        distance = np.linalg.norm(a - b)

        return distance
    
    def get_contact_force_magnitude(self):
        contact_force = np.array((self.contact_force.x, self.contact_force.y, self.contact_force.z))
        force_magnitude = np.linalg.norm(contact_force)
        return force_magnitude
    
    def get_joint_states(self):
        return self.joints_state

    
    def odom_callback(self):
        self.base_position = msg.pose.pose.position
    
    
    def imu_callback(self):
        self.base_orientation = msg.orientation
        self.base_linear_acceleration = msg.base_linear_acceleration


    def contact_callback(self):
        """
        /lowerleg_contactsensor_state/states[0]/contact_positions ==> PointContact in World
        /lowerleg_contactsensor_state/states[0]/contact_normals ==> NormalContact in World

        ==> One is an array of all the forces, the other total,
        and are relative to the contact link referred to in the sensor.
        /lowerleg_contactsensor_state/states[0]/wrenches[]
        /lowerleg_contactsensor_state/states[0]/total_wrench
        :param msg:
        :return:
        """

        for state in msg.states:
            self.contact_force = state.total_wrench.force


    def joints_state_callback(self, msg):
        self.joints_state = msg
    
    def monoped_height_ok(self):
        height_ok = self.min_height <= self.get_base_height() < self.max_height
        return height_ok

    
    def monoped_orientation_ok(self):
        orientation_rpy = self.get_base_rpy()
        roll_ok = self.abs_max_roll > abs(orientation_rpy.x)
        pitch_ok = self.abs_max_pitch > abs(orientation_rpy.y)
        orientation_ok = roll_ok and pitch_ok
        return orientation_ok

    def calculate_reward_joint_position(self, weight=1.0):
        accumulated_joint_pos = 0.0
        for joint_pos in self.joints_state.position:
            accumulated_joint_pos += abs(joint_pos)
            rospy.logdebug("calculate reward_joint_position >> accumulated_joint_pos" + str(accumulated_joint_pos))
        reward = weight * accumulated_joint_pos
        rospy.logdebug("calculate_reward_joint_position>>reward=" + str(reward))
        return reward        

    def calculate_reward_joint_effort(self, weight=1.0):
        """
        We calculate reward base on the joints effort readings. The more near 0 the better.
        :return:
        """
        accumulated_joint_effort = 0.0
        for joint_effort in self.joints_state.effort:
            accumulated_joint_effort +=abs(joint_effort)
            rospy.logdebug("calculate_reward_joint_effort>>joint_effort=" + str(joint_effort))
            rospy.logdebug("calculate_reward_joint_effort>>acumulated_joint_effort=" + str(acumulated_joint_effort))
        reward = weight * accumulated_joint_effort
        rospy.logdebug("calculate_reward_joint_effort>>reward=" + str(reward))
        return reward

    def calculate_reward_contact_force(self, weight=1.0):
        """
        calculate reward based on contact force.
        closer we are to the desired force the better
        desired force: 7.08 Newtons
        """

        force_magnitude = self.get_contact_force_magnitude()
        force_displacement = force_magnitude - self.desired_force

        rospy.logdebug("calculate_reward_contact_force>>force_magnitude=" + str(force_magnitude))
        rospy.logdebug("calculate_reward_contact_force>>force_displacement=" + str(force_displacement))

        reward = weight * abs(force_displacement)
        rospy.logdebug("calculate_reward_contact_force>>reward=" + str(reward))
        return reward

    def calculate_reward_orientation(self, weight=1.0):
        current_orientation = self.get_base_rpy()
        yaw_displacement = current_orientation.z - self.desired_yaw
        rospy.logdebug("calculate reward oriantation>>[R,P,Y]=" + str(current_orientation))
        accumulated_orientation_displacement = abs(current_orientation.x) + abs(current_orientation.y) + abs(yaw_displacement)
        reward = weight * accumulated_orientation_displacement
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def calculate_reward_distance_from_des_point(self, weight=1.0):
        distance = self.get_distance_from_point(self.starting_point)
        reward = weight * distance
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))

    def calculate_total_reward(self):

        r1 = self.calculate_reward_joint_position(self.weight_r1)
        r2 = self.calculate_reward_joint_effort(self.weight_r2)

        #desired force in newtons, taken from idel contact with 9.81 gracity
        r3 = self.calculate_reward_contact_force(self.weight_r3)
        r4 = self.calculate_reward_orientation(self.weight_r4)
        r5 = self.calculate_reward_distance_from_des_point(self.weight_r5)
        total_reward = self._alive_reward - r1 - r2 - r3 - r4 - r5

        rospy.logdebug("###############")
        rospy.logdebug("alive_bonus=" + str(self._alive_reward))
        rospy.logdebug("r1 joint_position=" + str(r1))
        rospy.logdebug("r2 joint_effort=" + str(r2))
        rospy.logdebug("r3 contact_force=" + str(r3))
        rospy.logdebug("r4 orientation=" + str(r4))
        rospy.logdebug("r5 distance=" + str(r5))
        rospy.logdebug("total_reward=" + str(total_reward))
        rospy.logdebug("###############")

        return total_reward


    def get_observations(self):
        distance_from_desired_point = self.get_distance_from_point(self.starting_point)

        base_orientation = self.get_base_rpy()
        base_roll = base_orientation.x
        base_pitch = base_orientation.y
        base_yaw = base_orientation.z

        contact_force = self.get_contact_force_magnitude()

        joint_states = self.get_joint_states()
        joint_states_haa = joint_states.position[0]
        joint_states_hfe = joint_states.position[1]
        joint_states_kfe = joint_states.position[2]

        observation = []
        for obs_name in self.list_of_observations:
            if obs_name == "distance_from_desired_point":
                observation.append(distance_from_desired_point)
            elif obs_name == "base_roll":
                observation.append(base_roll)
            elif obs_name == "base_pitch":
                observation.append(base_pitch)
            elif obs_name == "base_yaw":
                observation.append(base_yaw)
            elif obs_name == "contact_force":
                observation.append(contact_force)
            elif obs_name == "joint_states_haa":
                observation.append(joint_states_haa)
            elif obs_name == "joint_states_hfe":
                observation.append(joint_states_hfe)
            elif obs_name == "joint_states_kfe":
                observation.append(joint_states_kfe)
            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

        return observation

    def get_state_as_string(self, observation):
        """discretize the observations and 
        convert them to state tags strings"""

        observations_discrete = self.assign_bins(observation)
        string_state = ''.join(map(str, observations_discrete))
        return string_state

    def assign_bins(self, observation):
        state_discrete = np.zeros(len(self.list_of_observations))
        for i in range(len(self.list_of_observations)):
            state_discrete[i] = np.digitize(observation[i], self.bins[i])
        return state_discrete

    def init_bins(self):
        self.fill_observation_ranges()
        self.create_bins()

    
    def fill_observation_ranges(self):
        self.obs_range_dict = {}
        for obs_name in self.list_of_observations:

            if obs_name == "distance_from_desired_point":

                delta = self.max_height - self.min_height
                max_value = delta
                min_value = -delta
            
            elif obs_name == "base_roll":
                max_value = self.abs_max_roll
                min_value = -self.abs_max_roll
            
            elif obs_name == "base_pitch":
                max_value = self.abs_max_pitch
                min_value = -self.abs_max_pitch
            
            elif obs_name == "base_yaw":
                max_value = 2 * math.pi
                min_value = -2 * math.pi

            elif obs_name == "contact_force":
                max_value = 2 * self.desired_force
                min_value = 0.0

            elif obs_name == "joint_states_haa":
                max_value = 1.6
                min_value = -1.6
            
            elif obs_name == "joint_states_hfe":
                max_value = 1.6
                min_value = -1.6

            elif obs_name == "joint_states_kfe":
                max_value = 0.0
                min_value = -1.6
            
            else:
                raise NameError("observation does not exist" + str(obs_name))
            
            self.obs_range_dict[obs_name] = [min_value, max_value]  

    def create_bins(self):
        number_of_observations = len(self.list_of_observations)
        parts_we_discretize = self.discrete_division

        self.bins = np.zeros((number_of_observations, parts_we_discretize))
        for counter in range(number_of_observations):
            obs_name = self.list_of_observations[counter]
            min_value = self.obs_range_dict[obs_name][0]
            max_value = self.obs_range_dict[obs_name][1]
            self.bins[counter] = np.linspace(min_value, max_value, parts_we_discretize)
        


    def get_action_to_position(self, action):
        joint_states = self.get_joint_states()
        joint_states_position = joint_states
        
        action_position = [0.0, 0.0, 0.0]
        rospy.logdebug("get_action_to_position>>>"+str(joint_states_position))

        if action == 0: #haa_joint 
            action_position[0] = joint_states_position[0] + self.joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]

        elif action == 1: #haa_joint
            action_position[0] = joint_states_position[0] - self.joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
        
        elif action == 2: #hfe_joint
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1] + self.joint_increment_value
            action_position[2] = joint_states_position[2]
        
        elif action == 3: #hfe_joint
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1] - self.joint_increment_value
            action_position[2] = joint_states_position[2]
        
        elif action == 4: #kfe_joint
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2] + self.joint_increment_value
        
        elif action == 5: #kfe joint
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2] - self.joint_increment_value

        return action_position

    
    def process_data(self):
        """
        return the total reward based on state we're in and wether the robot is done or not (i.e. if it fell)
        """
        monoped_height_ok = self.monoped_height_ok()
        monoped_orientation_ok = self.monoped_orientation_ok()

        done = not(monoped_height_ok and monoped_orientation_ok)

        if done:
            rospy.logdebug("it fell, so reward is low")
            total_reward = self.done_reward
        else:
            rospy.logdebug("calculate normal reward because it didn't fall")
            total_reward = self.calculate_total_reward()
        
        return total_reward, done

    def testing_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown:
            self.calculate_total_reward
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("monoped_state_node", anonymous=True)
    
    monoped_state = MonopedState(max_height=3.0,
                                min_height=0.6,
                                abs_max_roll=0.7,
                                abs_max_pitch=0.7)
    
    monoped_state.testing_loop()