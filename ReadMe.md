# Hydrogen AUV – Software and Automation Subsystem

This branch is for the Software and Automation Subsystem, and we are building our 2nd prototype — **Hydrogen**.

We are tasked with simulating the entire underwater environment in Gazebo, and getting sensor data from the simulation through ROS.

We are making the actual competition track plus a side-by-side test track to learn and implement early techniques that we want our actual bot to perform.


# NOTE :-

before statring the simulation you need to set the enviorinment variable for ur gazebo plugin directory


```
echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:<path to your plugin folder>' >> ~/.bashrc
```
use pwd command to find 