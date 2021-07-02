[![pylint](https://gitlab.com/bcrllp/minilift/minilift_gazebo/-/jobs/artifacts/devel/raw/public/badges/pylint.svg?job=pylint)](https://gitlab.com/bcrllp/minilift/minilift_gazebo/-/jobs/artifacts/devel/browse/public/lint?job=pylint)

<img src="https://img.shields.io/badge/melodic-passing-green&style=plastic">

# minilift_gazebo

![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)

This package contains all the tools related to the UR3 deployment in Gazebo.

## Tutorial

* Install all the ROS dependencies:

      cd $WORKSPACE && rosdep install --from-paths src --ignore-src -r -y

* Install all python dependencies:

      pip install -r requirements.txt

* Launch the Gazebo simulation (Rviz optional):

      roslaunch minilift_gazebo minilift_world.launch rviz_enabled:=False

* To drive the minilift bot around:

      rosrun minilift_gazebo key_drive.py

* To actuate the liftplate ($JOINT_VALUE ranges from `0.15` to `0.56`):

      rostopic pub /minilift/lift_position_controller/command std_msgs/Float64 "data: $JOINT_VALUE"