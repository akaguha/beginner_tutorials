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
 *  @file    publisher.cpp
 *  @author  Akash Guha
 *  @copyright MIT License
 *
 *  @brief ENPM808X, Programming Assignment: ROS Publisher/Subscriber
 *
 *  @section DESCRIPTION
 *
 *  This program is a publisher node in C++
 *
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdlib.h>

/**
 * @brief      main function
 *
 * @param      argc,  number of arguments
 * @param      argv,  vector of string arguments
 *
 * @return     int, program execution status
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "publishString");  // initialize ROS
  ros::NodeHandle nh;  // handle to this process node

  // Tells master about message type and the topic on which message is published
  ros::Publisher pub = nh.advertise<std_msgs::String>("stringPub", 1000);

  ros::Rate rate(2);  // loop at 2Hz until the node is shut down
  while (ros::ok()) {  // check to loop until the node is up
    std_msgs::String msg;  // variable to store string
    msg.data = "Akash";

    pub.publish(msg);  // publish msg

    // ROS_INFO_STREAM(msg.data); // print message on the console using rosout

    rate.sleep();  // wait until another iteration
  }
  return 0;
}
