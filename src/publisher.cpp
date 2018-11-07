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
 *  @brief ENPM808X, Programming Assignment: ROS Services, Logging and Launch files
 *
 *  @section DESCRIPTION
 *
 *  This program publishes a string on stringPub topic for the subscrber node to print a
 *  message on the console. This also acts as a service server for a string change request. 
 *
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <beginner_tutorials/changeString.h>  //  srv class for the service
#include <ros/console.h>  //  to implement logging features

extern std::string pubTxt = "Akash";  //  default string to be published

/**
 * @brief      changeTxt
 *
 * @param      req  request data member
 * @param      res  response data member
 *
 * @return     bool success or failure
 */
bool changeTxt(beginner_tutorials::changeString::Request& req,
  beginner_tutorials::changeString::Response& res) {
  pubTxt = req.str;  //  assign requested string to publish variable
  res.str = pubTxt;  //  assigning value to response data member
  ROS_DEBUG_STREAM("Message to be published is, " << pubTxt);
  return true;
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
  ros::init(argc, argv, "publishString");  // initialize ROS
  ros::NodeHandle nh;  // handle to this process node

  //  setting the logger level to DEBUG
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  //  register service with the master
  ros::ServiceServer service = nh.advertiseService("changeString", changeTxt);

  // Tells master about message type and the topic on which message is published
  ros::Publisher pub = nh.advertise<std_msgs::String>("stringPub", 1000);

  int freq;
  if (argc == 2) {  //  check for the number of arguments passed
    if (std::atoi(argv[1]) <= 0) {
      ROS_ERROR_STREAM("Entered frequency value is either negative or zero");
      ROS_INFO_STREAM("Setting the frequency to the default value");
      } else {
          freq = std::atoi(argv[1]);
          ROS_DEBUG_STREAM("User entered frequency value is " << freq);
      }
  } else {
      ROS_FATAL_STREAM("Single frequency argument required");
      return 1;  //  terminate the node
  }

  ros::Rate rate(freq);  // loop at 'freq'Hz until the node is shut down
  while (ros::ok()) {  // check to loop until the node is up
    std_msgs::String msg;  // variable to store string
    std::stringstream ss;
    ss << pubTxt;
    msg.data = ss.str();

    pub.publish(msg);  // publish msg

    // ROS_INFO_STREAM(msg.data); // print message on the console using rosout
    ros::spinOnce();
    rate.sleep();  // wait until another iteration
  }
  return 0;
}
