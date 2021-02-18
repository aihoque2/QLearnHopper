#!/usr/bin/env python3
"""
TODO: INCOMPLETE
for Q-learning, the joint publisher needs some extra functions in order to be operational

joint_publisher.py
this is how the joints move in the robot
"""

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64


class JointPub(object):
    def __init__(self):
        self.publishers_array = []

        #our different joint publishers
        self.haa_joint_pub = rospy.Publisher('/monoped/haa_joint_position_controller/command', Float64, queue_size=1)
        self.hfe_joint_pub = rospy.Publisher('/monoped/hfe_joint_position_controller/command', Float64, queue_size=1)
        self.kfe_joint_pub = rospy.Publisher('/monoped/kfe_joint_position_controller/command', Float64, queue_size=1)

        #publishers_array is a 2D list that holds each of our publishers
        self.publishers_array.append(self.haa_joint_pub)
        self.publishers_array.append(self.hfe_joint_pub)
        self.publishers_array.append(self.kfe_joint_pub)

        self.init_pos = [0.0, 0.0, 0.0]

    def set_init_pose():
        """
        set the joints back to init_pos
        """

        self.check_publishers_connection()
        self.move_joints(self.init_pos)


    def check_publishers_connection(self):
        """
            bool function to make sure all
            the joint publishers are working
            you check if the publishers are 
            working by seeing if they are
            connected 
        """
        rate = rospy.rate(10)
        while(self.haa_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No subscribers to haa_joint_pub. Wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass  
        rospy.logdebug("haa_joint_publisher connected")

        while(self.hfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No subscribers to hfe_joint_pub. wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("hfe_joint_publisher connected")
        
        while(self.kfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No subscribers to kfe_joint_pub. wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("kfe_joint_publisher connected")

        rospy.logdebug("All publishers ready!")

    def joint_mono_des_callback(self, msg):
        rospy.logdebug(str(msg.joint_state.position))
        self.move_joints(msg.joint_state.position)

    def move_joints(self, joints_array):
        i = 0
        for publisher_object in self.publishers_array:
            joint_value = Float64()
            joint_value.data = joints_array[i]
            rospy.loginfo(str(joint_value))
            publisher_object.publish(joint_value)
            i += 1
    

    def start_loop(self, rate_value = 2.0):
        #the locomotion loop that this robot will utilize
        rospy.loginfo("start loop")
        pos1 = [0.0, 0.0, 1.57]
        pos2 = [0.0, 0.0, -1.57]
        position = "pos1"

        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            if position == "pos1":
                self.move_joints(pos1)
            else:
                position = "pos2"
                self.move_joints(pos2)
                position = "pos1"
            rate.sleep()

    def start_sinus_loop(self, rate_value = 2.0):
        rospy.logdebug("start loop")
        w = 0.0
        x = 2.0*math.sin(w)

        pos_x = [0.0, x, 0.0]
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            self.move_joints(pos_x)
            w += 0.05
            x = 2.0 * math.sin(w)

            pos_x = [0.0, x, 0.0]
            rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 50.0 #4 cycles a second
    joint_publisher.start_sinus_loop(rate_value)