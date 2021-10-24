<img src="https://img.shields.io/badge/noetic-passing-green&style=plastic"> <img src="https://img.shields.io/badge/melodic-passing-green&style=plastic">

# minilift_moveit_config

![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)

A ROS moveit configuration package designed to simulate the minilift robot in action.

## Tutorial

### Installation

* Install all the ROS dependencies:

      cd $WORKSPACE && rosdep install --from-paths src --ignore-src -r -y

### Launch

* Launch integrated Moveit! control with both Rviz and Gazebo:

      roslaunch minilift_moveit_config demo_gazebo.launch

### Modes of Actuation

* To drive the minilift bot around:

      rosrun teleop_twist_keyboard teleop_twist_keyboard.py

* To actuate the liftplate (`$JOINT_VALUE` ranges from `0.15` to `0.56`):

      rostopic pub /minilift/lift_position_controller/command std_msgs/Float64 "data: $JOINT_VALUE"

#### Alternative modes of launch

* Launch the Gazebo simulation without Moveit! (Rviz optional):

      roslaunch minilift_gazebo minilift_world.launch rviz_enabled:=False

* Launch RViz independently with Moveit! and planning control:

      roslaunch minilift_moveit_config demo.launch

* Launch Gazebo independently without Moveit! but with the attached controllers:

      # In the first terminal
      roslaunch minilift_moveit_config gazebo.launch   

      # In the second terminal
      rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
