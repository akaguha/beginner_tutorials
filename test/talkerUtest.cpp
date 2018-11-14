#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeString.h"
#include <gtest/gtest.h>

TEST(TestSuite, changeString) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::changeString>("changeString");
  bool exists(client.waitForExistence(ros::Duration(20)));
  EXPECT_TRUE(exists);

  beginner_tutorials::changeString srv;
  srv.request.str = "UMD";
  client.call(srv);

  EXPECT_EQ(srv.response.str, "UMD");

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talkerTester");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}