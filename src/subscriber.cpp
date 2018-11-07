/**
 *  MIT License
 *
 *  Copyright (c) 2018 Akash Guha
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    subscriber.cpp
 *  @author  Akash Guha
 *  @copyright MIT License
 *
 *  @brief ENPM808X, Programming Assignment: ROS Services, Logging and Launch files
 *
 *  @section DESCRIPTION
 *
 *  This program is a subscriber node in C++
 *
 */
#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * @brief      Callback function that will get called when a 
 *				new message has arrived
 *
 * @param  msg, message received from the publisher
 */
void stringReceivedCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO_STREAM("Hello " << msg->data);
}
/**
 * @brief      main function
 *
 * @param      argc,  number of arguments
 * @param      argv,  vector of string arguments
 *
 * @return     int, program execution status
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "stringListener");  // initialize ROS
  ros::NodeHandle nh;  // handle to this process node

  // Subscribe to the stringPub topic with the master
  // ROS will call the stringReceivedCallback() function whenever a
  // new message arrives
  ros::Subscriber sub = nh.subscribe("stringPub", 1000,
  &stringReceivedCallback);

  ros::spin();  // gives control to ROS until the node shuts down

  return 0;
}
