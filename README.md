# linorobot-overlay
This overlay is installed over a linorobot2 installation and includes Sokul robots etc

# Robot Stack

[Ubuntu 22.04](https://releases.ubuntu.com/jammy/)

[ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

Gazebo Fortress 
sudo apt-get install ros-${ROS_DISTRO}-ros-gz

[Linorobot2](https://github.com/linorobot/linorobot2)

HOLD  [Open-RMF](https://github.com/open-rmf/rmf)

HOLD  [RMF-web](https://github.com/open-rmf/rmf-web)

Robot only   [ROS2 laser scan merge](https://github.com/mich1342/ros2_laser_scan_merger) (for dual lidar robots)

[linorobot-overlay (Sokul IP)](https://github.com/SAKErobotics/linorobot-overlay/tree/main)

Robot only [VESC stack](https://github.com/SAKErobotics/vesc_ros)

Sokul sw

[Talk to ChatGPT](https://github.com/C-Nedelcu/talk-to-chatgpt)


# Installation of linorobot-overlay

make a directory AWAY FROM xxx.ws

cd into new directory

git clone https://github.com/SAKErobotics/linorobot-overlay

cd linorobot-overlay

cp -r lino* ~/####_ws/src/linorobot2

cd ~/####_ws

colcon build

source install/setup.bash

# How to run an example robot - tr

ros2 launch linorobot2_description tr.launch.py rviz:=true

separate terminals

ros2 launch linorobot2_gazebo tr_gazebo.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run joint_state_publisher_gui joint_state_publisher_gui

# How to run an example robot - mini_tr

ros2 launch linorobot2_description mini_tr.launch.py rviz:=true

separate terminals

ros2 launch linorobot2_gazebo mini_tr_gazebo.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run joint_state_publisher_gui joint_state_publisher_gui


# How to run an example robot - minilift

ros2 launch linorobot2_description minilift.launch.py rviz:=true

ros2 launch linorobot2_gazebo minilift_gazebo.launch.py rviz:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run joint_state_publisher_gui joint_state_publisher_gui



# How to run an example robot - minilift_scara_gripper

requires installation of

Linorobot_overlay
 
Clone the following into your_ws/src
      https://github.com/SAKErobotics/scara_arm
      https://github.com/SAKErobotics/EZGripper_ros2


ros2 launch linorobot2_description minilift.launch.py rviz:=true

ros2 launch linorobot2_gazebo minilift_gazebo.launch.py rviz:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run joint_state_publisher_gui joint_state_publisher_gui


