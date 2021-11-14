#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/ServiceFile.h"

std::shared_ptr<ros::NodeHandle> nh;

TEST (TESTSuite, serviceFileCheck)
{
  ros::ServiceClient client = nh->serviceClient<simple_rostest::AddTwoInts>(
      "NewService");
  bool exists(client.waitForExixtence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

TEST (TESTuite, serviceTestOutput)
  {
    ros::ServiceClient client = nh->serviceClient<simple_rostest::AddTwoInts>(
      "NewService");
    beginner_tutorials::ServiceFile srv;
    client.call(srv);
    EXPECT_EQ("hi", srv.response.output_msg)
  }

int main(int argc, char **argv) {
  ros::init(argc, argv, "service_test_node");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
