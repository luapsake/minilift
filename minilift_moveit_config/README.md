<img src="https://img.shields.io/badge/melodic-passing-green&style=plastic">

# minilift_moveit_config

![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)

A ROS moveit configuration package designed to simulate the minilift robot in action.

## Tutorial

* Install all the ROS dependencies:

      cd $WORKSPACE && rosdep install --from-paths src --ignore-src -r -y

* Launch RViz independently with planning control:

      roslaunch minilift_moveit_config demo.launch

* Launch Gazebo independently with the attached controllers:

      # In the first terminal
      roslaunch minilift_moveit_config gazebo.launch   

      # In the second terminal
      rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller

* Launch integrated moveit control:

      roslaunch minilift_moveit_config demo_gazebo.launch
      