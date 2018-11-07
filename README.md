# ENPM808X - Programming Assignment - ROS Services, Logging and Launch files
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

# License
```
MIT License

Copyright (c) 2018 Akash Guha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
# Overview
Project to create ROS service client and server to modify text published by talker, create launch file for both talker and listener nodes and to make use of all five ROS logging levels.

# Dependencies
Following dependencies need to be installed before running the above package
- Ubuntu 16.04
- ROS Kinetic

# Build Instructions
Create and build a catkin workspace.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Source your new setup.*sh file.
```
source devel/setup.bash
```
Clone the package in the src folder and build.
```
cd src/
git clone --recursive https://github.com/akaguha/beginner_tutorials.git
cd ..
catkin_make
```

# Demo Instructions - rosrun
For running the publisher, make sure the roscore is up and running
```
roscore
```
Make sure you have sourced your workspace's setup.sh file.
```
cd ~/catkin_ws
source ./devel/setup.bash
```
Run the node now.
```
rosrun beginner_tutorials pub 1
```
The publisher node is up and runningat 1Hz frequency. Now we need a subscriber to receive messages from the publisher
for running the subscriber.
```
rosrun beginner_tutorials sub
```
Run the service client node now. Here the text is modified to 'Ruben' from default 'Akash'. 
```
rosrun beginner_tutorials strChange Ruben
```
Output on the terminal.
```
[ INFO] [1541555368.452736427]: Hello Akash
[ INFO] [1541555369.452564036]: Hello Akash
[ INFO] [1541555370.452744030]: Hello Akash
[ INFO] [1541555371.452666956]: Hello Ruben
[ INFO] [1541555372.452716308]: Hello Ruben
[ INFO] [1541555373.452757756]: Hello Ruben
[ INFO] [1541555374.452784603]: Hello Ruben
[ INFO] [1541555375.452756956]: Hello Ruben
[ INFO] [1541555376.452731217]: Hello Ruben
[ INFO] [1541555377.452753678]: Hello Ruben
```
 Press CTRL+C to terminate the nodes.
 
# Demo Instructions - roslaunch
Make sure you have sourced your workspace's setup.sh file in the terminal.
```
cd ~/catkin_ws
source ./devel/setup.bash
```
No need run roscore explicitly. To run the publisher and subscriber nodes together at default frequency of 5Hz.
```
roslaunch beginner_tutorials publisherSubscriber.launch 
```
To run the nodes at a specific frequency,pass the frequency as an argument. Here the frequency argument is set to 1Hz. 
```
roslaunch beginner_tutorials publisherSubscriber.launch freq:=1
```
To change the published message using a service call from terminal. Here the text is modified to 'Ruben' from default 'Akash'.
```
rosservice call /changeString Ruben
```
Press CTRL+C to terminate the nodes.

# Logger Levels
To start rqt_console and logger_level GUI.
```
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
```
