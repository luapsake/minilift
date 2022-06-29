# minilift

This directory contains the base LFL and examples with a UR3 and EZGrippers



We created multiple launch files as it helped us with easy launch/debugging during development. I'd recommend using just the following,

    roslaunch minilift_moveit_config demo_gazebo.launch

This brings up the entire system in Gazebo with Moveit! interface and control plugins for movement and lift actuation. Other launch files are to bring up individual components in isolation, I am realising that they shouldn't be part of README.md either as it just creates confusion for the user. Regardless, to answer your specific queries - 

1. Regarding the missing rqt_joint_trajectory_controller , it should have been installed using the following command from README.md-
 
       rosdep install --from-paths src --ignore-src -r -y

2. The minilift robot actuates only when the Gazebo environment is launched as well, since the plugins are attached in the Gazebo simulation.We defined an alternative RViz-based independent launch file as it enables us to test and debug things very quickly without the need of running the entire integrated launch system. Therefore, running roslaunch minilift_moveit_config demo.launch will not allow you to move the robot in Gazebo and that's why no /cmd_vel subscriber shows up.

Put simply, please use roslaunch minilift_moveit_config demo_gazebo.launch and follow the instructions [here](https://github.com/luapsake/minilift/blob/devel/minilift_moveit_config/README.md#modes-of-actuation).

What follows is old

#   roslaunch minilift_gazebo minilift_world.launch
#   rosrun teleop_twist_keyboard teleop_twist_keyboard.py


