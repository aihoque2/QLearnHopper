#!/usr/bin/env python
"""TODO: FIRST TASK IS THIS ONE"""

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3   

class GazeboConnection():

    def __init__(self):
        """Pause, Unpause, and Reset simulation"""

    def pauseSim(self):

    def unPauseSim(self):
    
    def resetSim(self):

    def resetWorld(self):
    
    def init_values(self):

    def update_gravity_call(self):
        """at first, we don't put gravity and we let the robot run its actions, 
        then we activate the gravity and see the result of the actions we did"""

    def change_gravity(self):