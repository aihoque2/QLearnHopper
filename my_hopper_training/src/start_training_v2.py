#!/usr/bin/env python

""""TODO: INCOMPLETE"""

import gym
import time
import numpy as np
import random
import qlearn
from gym import wrappers

#ROS packages
from std_msgs.msg import Float64
import rospy
import rospkg

#training environment
import monoped_env
from monoped_env import MonopedEnv

if __name__ == '__main__':
    
    rospy.init_node('monoped_gym', anonymous=True, log_level=rospy.INFO)

    #create gym env
    env = gym.make('Monoped-v0')
    rospy.logdebug("Gym env created")

    #we create a publisher that tells the robot the reward it receives for an action at a state
    reward_pub = rospy.Publisher('/monoped/reward', Float64, queue_size=1)

    #this is the publisher that tells the robot Q_local reward
    episode_reward_pub = rospy.Publisher('/monoped/episode_reward', Float64, queue_size=1)


    #logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('my_hopper_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.monitor(env, outdir, force=True)
    rospy.logdebug("Monitor Wrapper started")

    last_time_steps = np.ndarry(0)


    # load our paramters from the yaml
    alpha = rospy.get_param("/alpha")
    epsilon = rospy.get_param("/epsilon")
    gamma = rospy.get_param("/gamma")
    epsilon_discount = rospy.get_param("/epsilon_discount")
    nepisodes = rospy.get_param("/nepisodes")
    nsteps = rospy.get_param("/nsteps")

    qlearn = qlearn.QLearn(actions=range(env.action_space.n), alpha=alpha, gamma=gamma, epsilon=epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    for x in range(nepisodes):
        rospy.loginfo ("STARTING EPISODE #" + str(x))
        cumulated_reward = 0
        cumulated_reward_msg = Float64()
        episode_reward_msg = Float64()
        done = False

        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        #initialize environment and get first state of the robot
        rospy.logdebug("env.reset....")

        #return directly to stringified observations called state
        state = env.reset()

        rospy.logdebug("env.get_state...===>" + str(state))
        for i in range(nsteps):

            #pick action based on current state
            action = qlearn.chooseAction(state)

            #execute action environment and see the feedback
            rospy.logdebug("######## Start step...[" + str(i) + "]")
            rospy.logdebug("haa+, haa-, hfe+, hfe-, kfe+, kfe- >> [0,1,2,3,4,5]") #our list of actions
            rospy.logdebug("Action to perform >> " + str(action))
            nextState, reward, done, info = env.step(action)
            rospy.logdebug("END Step...")
            rospy.logdebug("Reward ====> " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            
            rospy.logdebug("env.get_state...[distance_from_desired_point,base_roll,base_pitch,base_yaw,contact_force,joint_states_haa,joint_states_hfe,joint_states_kfe]==>" + str(nextState))

            #make algo learn based on next results
            qlearn.learn(state, action, reward, nextState)

            #we publish the cumulated reward
            cumulated_reward_msg.data = cumulated_reward
            reward_pub.publish(cumulated_reward_msg)

            if not(done):
                state = nextState
            else:
                rospy.logdebug("DONE")
                last_time_steps = np.append(last_time_steps, [int(i+1)])
                break

            rospy.logdebug("###################### END Step...["+str(i)+"]")

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        episode_reward_msg.data = cumulated_reward
        episode_reward_pub.publish(episode_reward_msg)
        rospy.loginfo( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))

    rospy.loginfo(("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()