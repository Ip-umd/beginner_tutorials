/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Ishan Patel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @file listener.cpp
 *
 * @date 28 October 2019
 *
 * @author Ishan Patel
 *
 * @brief Implementation of ROS publisher.
 *
 * @version 1
 *
 */

#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeBaseString.h"

struct msgString{
  std::string message;
};

/// Initialize object for the message
msgString msgString1;

/**
 * @brief changes the base string of talker
 * @param req new string from service client
 * @param res response of the service
 * @return bool 
 */
bool change(beginner_tutorials::changeBaseString::Request &req,
            beginner_tutorials::changeBaseString::Response &res) {
res.outString = req.inString;
msgString1.message = req.inString;
return true;
}

/**
 * @brief main function
 * @param argc count of arguments passed on command line
 * @param argv Stores all commandline arguments
 * @return 0 for successful execution
 *         
 */
int main(int argc, char **argv) {
  msgString1.message = "Base string changed.";
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
  ros::Publisher chatterPub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer service = n.advertiseService("change_string", change);
  
  /// Creating a tf broadcaster object
  tf::TransformBroadcaster br;

  /// Creating a tf transform object
  tf::Transform transform;

  int freq = atoi(argv[1]);
  if (freq <= 0) {
    ROS_FATAL_STREAM("This argument is not valid.Setting frequency to default value i.e. 10.");
    freq = 10;
  } else if (freq > 100) {
    ROS_ERROR_STREAM("Frequency is above maximum limit.Setting frequency to default value i.e. 10.");
    freq = 10;
  } else if (freq > 60 && freq < 100) {
    ROS_WARN_STREAM("Frequency is too high.");
  } else {
    ROS_INFO_STREAM("Frequency : " << freq);
  }

  ros::Rate loop_rate(freq);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << msgString1.message << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatterPub.publish(msg);
    
    /// Setting Origin
    transform.setOrigin( tf::Vector3(1.0, 2.0, 3.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 15);
    /// Setting Rotation
    transform.setRotation(q);
    /// Sending a transform with a TransformBroadcaster
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}


