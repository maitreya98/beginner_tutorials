/**
 * MIT License
 *
 * Copyright (c) 2021 Maitreya Kulkarni
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the  
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE. 
 * 
 * @file talker.cpp
 * @author Maitreya Kulkarni
 * @brief Program to start a publisher node which connects to ROS Network
 * @brief to publish a custom string
 * @version 0.1
 * @date 2021-11-07
 * 
 * @copyright Copyright (c) 2021 Maitreya Kulkarni
 */
#include <sstream>
#include "std_msgs/String.h"
#include "ros/ros.h"
// #include <iostream>
// Include service
#include "beginner_tutorials/ServiceFile.h"
#include <tf/transform_broadcaster.h>

extern std::string change = "Changed String: ";

bool setMessage(beginner_tutorials::ServiceFile::Request &req,
                beginner_tutorials::ServiceFile::Response &res) {
  // Stored the new requested string in the default string
  change = req.input_msg;
  res.output_msg  = req.input_msg;
  return true;
}
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // The service is created and advertised over ROS
  ros::ServiceServer server = n.advertiseService("NewService", setMessage);

  // Setting the loop frequency to a default of 10Hz
  int loop = 10;
  // Check if an argument for parsing a different loop rate is there.
  if (argc > 1) {
    loop = atoi(argv[1]);
  }
  // Log the frequency that is set
  ROS_DEBUG_STREAM("Loop frequency is set to :" << loop << "Hz");
  // Check if the frequency obtained is usable for the system
  if (loop > 0) {
    ROS_DEBUG_STREAM("Loop frequency of the loop is :" << loop << "Hz");
    // If the loop rate is too high: ERROR
    if (loop >= 100) {
      ROS_ERROR("Loop frequency is too high for this application");
      loop = 10;
      ROS_WARN_STREAM("Set to default frequency of " << loop << "Hz");
    }
  } else {
      // If the loop rate is negative, frequency becomes very high: ERROR
      if (loop < 0) {
        ROS_ERROR("Loop frequency cannot be negative");
      // If loop rate is zero, program won't run: FATAL
      } else if (loop == 0) {
        ROS_FATAL("Loop frequency cannot be zero");
      }
      // Set the loop rate to a safer frequency
      loop = 10;
      ROS_WARN_STREAM("Set to default frequency of " << loop << "Hz");
  }
/*
 * Specifies a frequency. It will keep track of how long it has been since the last
 * call to Rate::sleep(), and sleep for the correct amount of time.
 */
  ros::Rate loop_rate(loop);
  static tf::TransformBroadcaster pub;
  tf::Transform transform;
  tf::Quaternion quat;

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    transform.setOrigin(tf::Vector3(1, 4, 3));
    quat.setRPY(0, 0, 90);
    pub.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << change <<" " << count;
    // Store string in data
    msg.data = ss.str();
    // Stream information about sting being published
    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    // Confirmation that string has been published
    ROS_DEBUG_STREAM("Published the message");

    ros::spinOnce();
    // Sleep for the time remaining to let us hit our loop_rate publish rate
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
