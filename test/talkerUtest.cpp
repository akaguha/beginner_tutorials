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
 *  @brief ENPM808X, Programming Assignment - ROS TF, unit testing, bag files
 *
 *  @section DESCRIPTION
 *
 *  This is a test program to check the working of the change string service. 
 *
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/changeString.h"

TEST(TestSuite, changeString) {
  ros::NodeHandle nh;  //  Create a node handle
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::changeString>
  ("changeString");
  //  Test to check for existence of the service
  bool exists(client.waitForExistence(ros::Duration(20)));
  EXPECT_TRUE(exists);

  beginner_tutorials::changeString srv;
  srv.request.str = "UMD";  //  Set the request string
  client.call(srv);  //  Call the Service
  //  Test to check if received response is same as sent string
  EXPECT_EQ(srv.response.str, "UMD");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talkerTester");  //  initialize the node
  ros::NodeHandle nh;  //  Create a node handle
  testing::InitGoogleTest(&argc, argv);
  //  Run all the declared tests with TEST()
  return RUN_ALL_TESTS();
}
