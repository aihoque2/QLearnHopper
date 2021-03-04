# IlliniHopper

Behold my IlliniHopper! Hail to the orange, Hail to the blue...

Following the constructsim tutorial, I decided to create my own monoped robot 
in order to teach myself more about ROS and using OpenAI in a Gazebo Environment

I used the monoped robot from ETH Zurich's xpp package and tried to teach it to
hop with by using Q-learning. The algorithm did not converge, but the result was
quite interesting:

![Alt Text](https://media4.giphy.com/media/lIsgQpeQiUnZ7GnSNP/giphy.gif)



Requirements:

ROS-melodic
Gazebo

to run please use the following commands:

`catkin_make`

to launch the monoped:

`roslaunch my_legged_robots_sims main.launch`

to launch the Qlearning:

`roslaunch my_hopper_training main.launch`


Here is the node graph with the topics and subscribers:

<img src="https://github.com/aihoque2/IlliniHopper/blob/main/pics/node_and_topic_graph.png">
