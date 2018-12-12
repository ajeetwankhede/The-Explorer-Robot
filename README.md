# The Explorer Robot
[![Build Status](https://travis-ci.org/ajeetwankhede/The-Explorer-Robot.svg?branch=master)](https://travis-ci.org/ajeetwankhede/The-Explorer-Robot)

[![Coverage Status](https://coveralls.io/repos/github/ajeetwankhede/The-Explorer-Robot/badge.svg)](https://coveralls.io/github/ajeetwankhede/The-Explorer-Robot)

<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>

## Team Members
1. Likhita Madiraju: I am an M.Eng. Robotics student at the University of Maryland. 
2. Ajeet Wankhede: I am a master's student at the University of Maryland, studying Robotics.

## Author Name for Sprint 1
Driver: Ajeet Wankhede

Navigator: Likhita Madiraju

## Author Name for Sprint 2
Driver: Likhita Madiraju

Navigator: Ajeet Wankhede

## Author Name for Sprint 3
Driver: Ajeet Wankhede

Navigator: Likhita Madiraju

## Project Overview
This autonomous robotic system can be applied for TurtleBot in any indoor facility for 2D mapping and path planning. Simultaneous Localization and Mapping (SLAM) is developed, for Acme Robotics, for the navigation of their TurtleBot, in an unknown environment. Gazebo is used to create a model of a warehouse to be explored by the TurtleBot. SLAM enables the location and building of a map while the robot is moving through the warehouse safely. Once the floorplan is created, a path planner is implemented for autonomous navigation, which can be used to locate and transport packages within the warehouse. The generated map and the optimal trajectory is visualized with the help of rviz visualization environment. A demo is provided, implementing the exploration of unknown environment and path planning, in a warehouse setting.  

## Disclaimer
This project comes with MIT license. To view the entire license content please visit the following [link](https://opensource.org/licenses/MIT)

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
source ./devel/setup.bash
cd src/
git clone --recursive https://github.com/ajeetwankhede/The-Explorer-Robot.git
cd ../..
catkin_make
```
## Running a Gazebo demo
We have created a warehouse world in Gazebo with TurtleBot. To view this world enter the following commands:
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/<user name>/catkin_ws/src/the_explorer_robot/world/warehouse.world 
```
The world should look as shown below:
<p align="center">
<img src="/output/warehouse.png">
</p>

This world is mapped using gmapping which uses SLAM. A explorer node is created for autonomous SLAM which subscribes to the laser data of turtle bot provided by topic /scan, and publishes the command velocities for turtle bot. The algorithm for explorer is explained in the activity daigram. It can also record the messages for approx 30 sec using rosbag. This process is visualized in Rviz. Enter following commands in terminal to launch the demo:

a. Autonomous SLAM:
```
cd ~/catkin_ws/
source ./devel/setup.bash
roslaunch the_explorer_robot demo_launch.launch
```
The TurtleBot can also be controlled by using teleop for generating the 2D map. If teleop is launched then it overrides the messages published by explorer node. To launch teleop enter the following command in new terminal
```
roslaunch turtlebot_teleop keyboard_teleop.launch --screen
```

## Using the Service to change the speed
While using gmapping the linear speed and angular speed of the robot can be changed using the service change_speed. First argument is the linear speed and second is the angular speed. It can be called by entering the following commands in a new terminal.
```
rosservice call /change_speed <linear_speed> <angular_speed>
```

Once the mapping generates a rough map, it can be saved by running the service provided by gmapping. Enter the following command in new terminal.
```
rosrun map_server map_saver -f <file_path>
``` 
Output of the gmapping is as shown below:
<p align="center">
<img src="/output/slam.png">
</p>

This map can be made better either using teleop or editing the saved map. The map generated after moving the TurtleBot to the parts where the area in the map was grey, we the the below map:
<p align="center">
<img src="/output/slam_final.png">
</p>

b. Path planning using ROS navigation stack
The saved map is used for path planning using ROS navigation stack. Enter the following commands to launch the navigation stack demo:
```
cd ~/catkin_ws/
source ./devel/setup.bash
roslaunch the_explorer_robot path_launch.launch
```
In Rviz the TurtleBot pose can be estimated using 2D pose estimate. Once the pose is approximately as in Gazebo, the path planner can be used by setting a goal with the help of 2D Nav goal button. The TutleBot navigates to the pose that has been set.

## Stop the nodes
To stop, press Ctrl+C and then enter the following command to clean up the nodes from the rosnode list
```
rosnode cleanup
```

## Visualize in rqt
To visualize the publish-subscribe relationships between the nodes as a graph run the following command in a new terminal
```
rosrun rqt_graph rqt_graph
```

## Using rosbag to record and replay the messages
To start recoding the messages expect /camera/* topics, using rosbag run the following commands in a new terminal. It will record for approx 30 sec and dave a bag file in results folder. Make sure roscore is running.
```
roslaunch the_explorer_robot demo_launch_file.launch record:=true
```
The recording can be stopped by pressing Ctrl+C and the messages will be saved in results/recordedData.bag file. To disable the recording option while launching the launch_file run the following command.
```
roslaunch the_explorer_robot demo_launch_file.launch record:=false
```

To play the recorded messages run the following command in a new terminal.
```
cd ~/catkin_ws/src/the_explorer_robot/results
rosbag play recordedData.bag
```

## Running the tests
To run the tests written for explorer node run the following commands in a new terminal. It will show the output after ROS runs the tests.
```
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make run_tests_the_explorer_robot
```
The test ouput could also be seen by launching the test launch file by entering the following commands:
```
rostest the_explorer_robot explorerTest.launch
```