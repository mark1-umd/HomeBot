/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAHvacActionServer_test.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 2, 2017 - Creation
 *
 * @brief ROS test node for the Home Automation HVAC Action Server
 *
 * This ROS node is executed under the rostest environment to test the Home Automation HVAC Action Server.
 *
 * *
 * * BSD 3-Clause License
 *
 * Copyright (c) 2017, Mark Jenkins
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
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
 */

#include "ros/ros.h"
#include "gtest/gtest.h"
#include "actionlib/client/simple_action_client.h"
#include "homebot/HAHvacAction.h"

typedef actionlib::SimpleActionClient<homebot::HAHvacAction> Client;

/*******************************************************************************/
// Set up the HAHvacActionServer  action test using the GoogleTest macros
TEST(HAHvacActionServer, HAHvacAction) {
  // Set up as an action client
  ros::NodeHandle nh;
  // Create a simple action client; 1st argument is the action name; second is true for no ros::spin()
  Client client("ha_hvac", true);  // true -> don't need to run ros::spin() on our own

  // Make sure the server is running
  bool serviceAvailable = client.waitForServer(ros::Duration(1));
  EXPECT_TRUE(serviceAvailable);

  // Create a goal object for testing
  homebot::HAHvacGoal goal;

  // Test invalid action
  goal.mode = 2;
  goal.tempDegF = 70.0;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  bool isAborted = (client.getState()
      == actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_TRUE(isAborted);  // This hack required because == comparison fails inside gtest

  // Test temp too low
  goal.mode = homebot::HAHvacGoal::COOL;
  goal.tempDegF = homebot::HAHvacGoal::MINTEMPDEGF - 0.1;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  isAborted = (client.getState() == actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_TRUE(isAborted);  // This hack required because == comparison fails inside gtest

  // Test temp too high
  goal.mode = homebot::HAHvacGoal::HEAT;
  goal.tempDegF = homebot::HAHvacGoal::MAXTEMPDEGF + 0.1;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  isAborted = (client.getState() == actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_TRUE(isAborted);  // This hack required because == comparison fails inside gtest

  // Test valid increased setting (based on initial temperature of 70.0 deg F the change should complete in 1 second)
  goal.mode = homebot::HAHvacGoal::HEAT;
  goal.tempDegF = 71.0;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(2));
  bool isSucceeded = (client.getState()
      == actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(isSucceeded);  // This hack required because == comparison fails inside gtest

  // Test that result is within tolerance
  homebot::HAHvacResultConstPtr result = client.getResult();
  double tempVariance = abs(goal.tempDegF - result->tempDegF);
  EXPECT_GE(0.2, tempVariance);

  // Test valid decreased setting (based on initial temperature of 71.0 deg F the change should complete in 1 second)
  goal.mode = homebot::HAHvacGoal::HEAT;
  goal.tempDegF = 70.0;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(2));
  isSucceeded = (client.getState()
      == actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(isSucceeded);  // This hack required because == comparison fails inside gtest

  // Test that result is within tolerance
  result = client.getResult();
  tempVariance = abs(goal.tempDegF - result->tempDegF);
  EXPECT_GE(0.2, tempVariance);
}

/*******************************************************************************/

/*
 * @brief Initialize ROS and run all of the tests defined in the test macros
 */
int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "HAHvacActionServer_test");

  // Run all of the Google Test defined tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
