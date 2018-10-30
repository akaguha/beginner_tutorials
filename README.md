# ENPM808X - Programming Assignment: ROS Publisher/Subscriber
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

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
Project to write a publisher and subscriber node in C++.

# Dependencies
Following dependencies need to be installed before running the above package
- Ubuntu 16.04
- ROS Kinetic

# Build Instructions
create and build a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
source your new setup.*sh file
```
source devel/setup.bash
```
clone the package in the src folder and build
```
cd src/
git clone --recursive https://github.com/akaguha/beginner_tutorials.git
cd ..
catkin_make
```

# Demo Instructions
for running the publisher, make sure the roscore is up and running
```
roscore
```
make sure you have sourced your workspace's setup.sh file
```
cd ~/catkin_ws
source ./devel/setup.bash
```
run the node now
```
rosrun beginner_tutorials pub
```
The publisher node is up and running. Now we need a subscriber to receive messages from the publisher
for running the subscriber
```
rosrun beginner_tutorials sub
```
You will now see something similar to
```
[ INFO] [1540938462.387663572]: Hello Akash
[ INFO] [1540938462.887433020]: Hello Akash
[ INFO] [1540938463.387298427]: Hello Akash
[ INFO] [1540938463.887199737]: Hello Akash
[ INFO] [1540938464.387362588]: Hello Akash
[ INFO] [1540938464.887498541]: Hello Akash
[ INFO] [1540938465.387481945]: Hello Akash
[ INFO] [1540938465.887488903]: Hello Akash
```
When you are done, press Ctrl-C to terminate both the publisher and subscriber
