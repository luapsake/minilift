<img src="https://img.shields.io/badge/melodic-passing-green&style=plastic">

# minilift_description

![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)

This directory contains the base LFL and examples with a UR3 and EZGrippers.

## Tutorial

* Install all the ROS dependencies:

      cd $WORKSPACE && rosdep install --from-paths src --ignore-src -r -y

* Launch RViz:

      roslaunch minilift_description lfl_ur3_ezg_tri_real.launch