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
        """copy constructor"""
        
        #Initialize service proxies to Pause, Unpause, and Reset simulation"""
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        #settting up our gravity console system
        service_name = '/gazebo/set_physics_properties'
        rospy.logdebug("waiting for service" + str(service_name))
        rospy.wait_for_service(service_name)
        rospy.logdebug("Found service!")

        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.init_values()

        self.pauseSim() #pause the sim to help the robot learn

    def pauseSim(self):
        rospy.wait_for_service('gazebo/pause_physics')
        try: 
            self.pause_proxy() #call unpause service
        except rospy.ServiceException, e:
            print("gazebo pause service failed to call")

    def unPauseSim(self):
        rospy.wait_for_service('gazebo/unpause_physics')
        try:
            self.unpause_proxy()
        except rospy.ServiceException, e:
            print("gazebo unpause service failed to call")
    
    def resetSim(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException, e:
            print("gazebo resetSim() failed to call")

    def resetWorld(self):
        try:
            rospy.wait_for_service('/gazebo/reset_world')
        except: rospy.ServiceException, e:
            print("gazebo resetWorld() failed to call")
    
    def init_values(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()

        except rospy.ServiceException, e:
            print("gazebo resetSim() failed to call")

        self.time_step = Float64(0.0001)
        self.max_update_rate = Float64(1000.0)

        #initialize gravity
        self.gravity = Vector3()
        self.gravity.x = 0.0
        self.gravity.y = 0.0
        self.gravity.z = 0.0

        #config the ODE see: Gazebo Msg/Srv Documentation 
        self.ode_config = ODEPhysics() 
        self.ode_config.auto_disable_bodies = False 
        self.ode_config.sor_pgs_iters = 0 # inner iterations when uisng projected Gauss Seidel
        self.ode_config.sor_pgs_w # relaxation parameter when using projected Gauss Seidel, 1 = no relaxation
        self.ode_config.sor_pgs_rms_error_tol = 0.0
        self.ode_config.contact_surface_layer = 0.001 # contact "dead-band" width
        self.ode_config.contact_max_correcting_vel = 0.0
        self.ode_config.cfm = 0.0
        self.ode_config.erp = 0.2
        self.ode_config.max_contacts = 20

        self.update_gravity_call()
    

    def update_gravity_call(self):
        """at first, we don't put gravity and we let the robot run its actions, 
        then we activate the gravity and see the result of the actions we did"""
        self.pauseSim()

        set_physics_request = SetPhysicsProperties
        set_physics_request.time_step = self.time_step.data
        set_physics_request.max_update_rate = self.max_update_rate.data
        set_physics_request.gravity = self.gravity
        set_physics_request.ode_config = self.ode_config

        rospy.logdebug(str(set_physics_request.gravity))
        result = self.set_physics(set_physics_request)
        rospy.logdebug("Gravity update result==" + str(result.success) + ", message =="+ str(results.status_message))
        
        self.unPauseSim()

    def change_gravity(self, x, y, z): #here is where we call update_gravity_call()
        self.gravity.x = x
        self.gravity.y = y
        self.gravity.z = z

        self.update_gravity_call()