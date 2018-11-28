# The Explorer Robot
[![Build Status](https://travis-ci.org/ajeetwankhede/The-Explorer-Robot.svg?branch=master)](https://travis-ci.org/ajeetwankhede/The-Explorer-Robot)

[![Coverage Status](https://coveralls.io/repos/github/ajeetwankhede/The-Explorer-Robot/badge.svg?branch=master)](https://coveralls.io/github/ajeetwankhede/The-Explorer-Robot?branch=master)

<p align="right">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>
---

## Author Name for Sprint 1
Driver: Ajeet Wankhede

Navigator: Likhita Madiraju

## Project Overview
This autonomous robotic system can be applied for TurtleBot in any indoor facility for 2D mapping and path planning. Simultaneous Localization and Mapping (SLAM) is developed, for Acme Robotics, for the navigation of their TurtleBot, in an unknown environment. Gazebo is used to create a model of a warehouse to be explored by the TurtleBot. SLAM enables the location and building of a map while the robot is moving through the warehouse safely. Once the floorplan is created, a path planner is implemented for autonomous navigation, which can be used to locate and transport packages within the warehouse. The generated map and the optimal trajectory is visualized with the help of rviz 3D visualization environment. A demo is provided, implementing the exploration of unknown environment and path planning, in a warehouse setting.  

## Link for SIP document
[SIP document](https://docs.google.com/spreadsheets/d/10SLYF6b-H8jBe_Sn3X3n9NG67jSoSL41Iv0T1WjI9ok/edit?usp=sharing)

## Link for Sprint review document 
[Sprint review document](https://docs.google.com/document/d/1Z08-iwbV5uTY5A9PehAq-EnedAIHFBOwVHvecpv49y0/edit?usp=sharing)

## Dependencies
1. ROS Kinetic - to install ROS follow the [link](http://wiki.ros.org/kinetic/Installation)

2. catkin - to install catkin run the following command
```
sudo apt-get install ros-kinetic-catkin
```

3. Package Dependencies

 	a. roscpp
 
 	b. std_msgs
 
 	c. rostest
 
 	d. turtlebot simulation stack - to install turtlebot run the following command
 ```
 sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
 ```
 
 4. Gazebo (v7.1) - Select while installing ROS Kinetic
 
 ## Build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/ajeetwankhede/The-Explorer-Robot.git
cd ..
cd ..
catkin_make
```
