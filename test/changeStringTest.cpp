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
 * @file changeStringTest.cpp
 *
 * @date 11 November 2019
 *
 * @author Ishan Patel
 *
 * @brief Implementation of Level 2 integration test
 *
 * @version 1
 *
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/changeBaseString.h>

/**
 * @brief      Tests to determine whether the service changes text
 *
 */
TEST(ChangeBaseStringTest, checkChangeString) {
  /// Creating a ROS node handle
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient
      < beginner_tutorials::changeBaseString > ("change_string");
  /// Checking if the service exists
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);

  /// Creating service object
  beginner_tutorials::changeBaseString srv;
  srv.request.inString = "ENPM808X";
  client.call(srv);
  /// comparing the service response with the requested string
  EXPECT_STREQ("ENPM808X", srv.response.outString.c_str());
}

/**
 * @brief main function
 * @param argc count of arguments passed on command line
 * @param argv Stores all commandline arguments
 * @return Run all the tests that were declared with TEST()
 *         
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "changeStringTest");
  return RUN_ALL_TESTS();
}
