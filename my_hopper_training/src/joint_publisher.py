#!/usr/bin/env python3
"""TODO: INCOMPLETE"""

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
        pos1 = [0.0, 1.57, -1.57]
        pos2 = [0.0, 0.0, 0.0]
        position = "pos1"

        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            if position == "pos1":
                self.move_joints(pos1)
                position = "pos2"
            else:
                self.move_joints(pos2)
                position = "pos1"
            rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 4.0 #4 cycles a second
    joint_publisher.start_loop(rate_value)