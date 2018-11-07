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
 *  @file    stringChangeClient.cpp
 *  @author  Akash Guha
 *  @copyright MIT License
 *
 *  @brief ENPM808X, Programming Assignment: ROS Services, Logging and Launch files
 *
 *  @section DESCRIPTION
 *
 *  This program is a service client to change the string passed by the publisher node
 *
 */
#include <ros/ros.h>
#include <beginner_tutorials/changeString.h>  //  srv class for the service
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
  ros::init(argc, argv, "changeStringClient");  // initialize ROS
  ros::NodeHandle nh;  //  handle to this process node
  //  create a client object for the string change service.
  //  Need to specify the data type of the service
  //  and its name
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::changeString>
  ("changeString");

  beginner_tutorials::changeString::Request req;  //  request object
  beginner_tutorials::changeString::Response resp;  //  reponse object
  if (argc == 1) {  //  loop when no arguments are passed
    ROS_ERROR_STREAM("Text to be changed not passed, changing string "
  "to default...");
    req.str = "Akash";  //  assign values to request data members
  } else {
    req.str = argv[1];  //  assign the argument value to request data member
  }
  bool success = client.call(req, resp);  //  calling the service

  //  check the success of the service call and use the response data member
  if (success) {
    ROS_INFO_STREAM("Changed string: " << resp.str);
  } else {
    ROS_FATAL_STREAM("Failed to call service");
    return 1;
  }
  return 0;
}
