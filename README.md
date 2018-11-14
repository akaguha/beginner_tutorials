# ENPM808X - Programming Assignment - ROS TF, unit testing, bag files
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
Project to write a talker and listener node, create ROS service client and server to modify text published by talker, create launch file for both talker and listener nodes and to make use of all five ROS logging levels. The Talker node broadcasts a tf frame called /talk with parent /world. Unit tests written using gtest and rostest for testing ROS service call. Recording all published messages in a bag file.

# Dependencies
Following dependencies need to be installed before running the packages
- Ubuntu 16.04
- ROS Kinetic
- catkin
- roscpp
- message_generation
- tf

# Build Instructions
Create and build a catkin workspace.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Source your new setup.*sh file
```
source devel/setup.bash
```
Clone the package in the src folder and build
```
cd src/
git clone --recursive https://github.com/akaguha/beginner_tutorials.git --branch Week11_HW
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

# Rrecording and Playing back data
To record all published topics in a bag file through a launch file run the following command
```
roslaunch beginner_tutorials publisherSubscriber.launch freq:=1 record_bag:=true
```
To play back the recorded messages in the bag file
```
cd <path to catkin_ws>/src/beginner_tutorials/results
rosbag play allTopics.bag
```
Output on the terminal
```
Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [DELAYED]  Bag Time: 1542159242.490944   Duration: 0.000000 / 79.912262   Delay [RUNNING]  Bag Time: 1542159242.490944   Duration: 0.000000 / 79.912262         [RUNNING]  Bag Time: 1542159242.490944   Duration: 0.000000 / 79.912262         [RUNNING]  Bag Time: 1542159242.491358   Duration: 0.000414 / 79.912262         [RUNNING]  Bag Time: 1542159242.591543   Duration: 0.100599 / 79.912262         [RUNNING]  Bag Time: 1542159242.691735   Duration: 0.200791 / 79.912262         [RUNNING]  Bag Time: 1542159242.708552   Duration: 0.217608 / 79.912262         [RUNNING]  Bag Time: 1542159242.709033   Duration: 0.218090 / 79.912262         [RUNNING]  Bag Time: 1542159242.809177   Duration: 0.318234 / 79.912262         [RUNNING]  Bag Time: 1542159242.909394   Duration: 0.418450 / 79.912262         [RUNNING]  Bag Time: 1542159243.009611   Duration: 0.518667 / 79.912262         [RUNNING]  Bag Time: 1542159243.109828   Duration: 0.618884 / 79.912262         [RUNNING]  Bag Time: 1542159243.210021   Duration: 0.719077 / 79.912262         [RUNNING]  Bag Time: 1542159243.310202
 ```
 In a different terminal run the following commands to see the published string
 ```
 rostopic echo /stringPub
 ```
 or
 ```
  rosrun beginner_tutorials sub
  ```

# TF Broadcaster
The talker node broadcasts a tf frame called /talk with parent /world. Run the launch file in a terminal
```
roslaunch beginner_tutorials publisherSubscriber.launch freq:=1
```
To see the broadcasted tf message using tf_echo,run
```
rosrun tf tf_echo /world /talk
```
Output on the terminal
```
At time 1542159322.402
- Translation: [5.000, 5.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.650, 0.760]
            in RPY (radian) [0.000, 0.000, -1.416]
            in RPY (degree) [0.000, 0.000, -81.127]
At time 1542159322.402
- Translation: [5.000, 5.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.650, 0.760]
            in RPY (radian) [0.000, 0.000, -1.416]
            in RPY (degree) [0.000, 0.000, -81.127]
At time 1542159322.402
- Translation: [5.000, 5.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.650, 0.760]
            in RPY (radian) [0.000, 0.000, -1.416]
            in RPY (degree) [0.000, 0.000, -81.127]
At time 1542159322.402
- Translation: [5.000, 5.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.650, 0.760]
            in RPY (radian) [0.000, 0.000, -1.416]
```
Using view_frames creates a diagram of the frames being broadcast by tf over ROS.
```
rosrun tf view_frames
```
To view the tree
```
evince frames.pdf
```
Using rqt_tf_tree visualizing the tree of frames being broadcast over ROS.
```
rosrun rqt_tf_tree rqt_tf_tree
```

# Integration Testing using rostest
To run unit test for ROS service call
```
cd <path to catkin_ws>
catkin_make run_tests
```
Output on terminal
```
[ROSUNIT] Outputting test results to /home/akash/ENPM808X_ROS_workspace/build/test_results/beginner_tutorials/rostest-test_talkerUtest.xml
[Testcase: testtalkerTester] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerTester/changeString][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/akash/.ros/log/rostest-akash-Inspiron-7577-16123.log
-- run_tests.py: verify result "/home/akash/ENPM808X_ROS_workspace/build/test_results/beginner_tutorials/rostest-test_talkerUtest.xml"
```
To run the unit tests using the launch file, run the following commands in the catkin workspace after all the packages are succesfully built.
```
cd <path to catkin_ws>
source devel/setup.bash
rostest beginner_tutorials talkerUtest.launch 
```
