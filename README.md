# COMP3766-Project-Implementation.

By: Maham Mahmood and Maiza Asif.

This repository contains the code and implementation for our PUMA 560 robot arm. This repo was taken from SantiagoRG2401/puma560. The main movement logic can be found in control.py file. 

Note:To run this project successfully, a ROS workspace (catkin workspace) must be created and configured properly.
To run this project, these are the commands to follow:
catkin_make
source devel/setup.bash
roslaunch puma560_gazebo gazebo.launch

Then in a new terminal
catkin_make
source devel/setup.bash
roslaunch puma560_description display.launch

and then open a new terminal to run out main code:
catkin_make
source devel/setup.bash
rosrun puma560_control control.py




