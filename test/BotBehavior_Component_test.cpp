/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAHvacActionServer_test.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 2, 2017 - Creation
 *
 * @brief ROS test node for the HomeBot Bot Behavior Components
 *
 * This ROS node is executed under the rostest environment to test the HomeBot Bot Behavior Components
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

#include <string>
#include "gtest/gtest.h"
#include "ros/ros.h"

#include "homebot/BotAffectHADoorOpr.hpp"
#include "homebot/BotAffectHASceneOpr.hpp"
#include "homebot/BotAffectHAShadeOpr.hpp"
#include "homebot/BotOperation.hpp"
#include "homebot/BotMoveBaseOpr.hpp"
#include "homebot/BotOprClients.hpp"


/*******************************************************************************/
TEST (HomeBotBehavior, BotMoveBase) {

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;

  std::string oprCode("BotMoveBase");
  std::string frame_id("map");
  int xPos(23);
  int yPos(17);
  int zPos(11);
  int xOrient(2);
  int yOrient(3);
  int zOrient(5);
  int wOrient(1);

  // Construct a BotMoveBase operation and determine if the components are as specified
  BotMoveBaseOpr mbOpr(oprCode, frame_id, xPos, yPos, zPos, xOrient, yOrient,
                       zOrient, wOrient);
  EXPECT_EQ(oprCode, mbOpr.getCode());
  move_base_msgs::MoveBaseGoal goal = mbOpr.details();
  EXPECT_EQ(frame_id, goal.target_pose.header.frame_id);
  EXPECT_EQ(xPos, goal.target_pose.pose.position.x);
  EXPECT_EQ(yPos, goal.target_pose.pose.position.y);
  EXPECT_EQ(zPos, goal.target_pose.pose.position.z);
  EXPECT_EQ(xOrient, goal.target_pose.pose.orientation.x);
  EXPECT_EQ(yOrient, goal.target_pose.pose.orientation.y);
  EXPECT_EQ(zOrient, goal.target_pose.pose.orientation.z);
  EXPECT_EQ(wOrient, goal.target_pose.pose.orientation.w);

  // Now try and execute the BotMoveBase operation without a corresponding server
  BotOprClients botOprClients;
  EXPECT_FALSE(botOprClients.allStarted());

  // And this should fail, too
  EXPECT_FALSE(mbOpr.execute(botOprClients));
}

/*******************************************************************************/
TEST (HomeBotBehavior, BotAffectHADoorConstruction) {

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;

  std::string oprCode = "HADoor";
  int doorNumber(1);
  int action(2);

  // Construct a HADoor operation and determine if the components are as specified
  BotAffectHADoorOpr doorOpr(oprCode, doorNumber, action);
  EXPECT_EQ(oprCode, doorOpr.getCode());
  homebot::HADoorRequest doorReq = doorOpr.details();
  EXPECT_EQ(doorNumber, doorReq.doorNumber);
  EXPECT_EQ(action, doorReq.action);
}

/*******************************************************************************/
// Set up the HARequestServer HADoor service test using the GoogleTest macros
// This test assumes that the HARequestServer has been initialized with 5 doors
// (argument -doors 5)
TEST (HomeBotBehavior, HADoorService) {
  // Set the number of doors in the server to be tested
  int doorCount = 5;

  // Set up as a service client
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<homebot::HADoor>("ha_door");

  // Test the availability of the service
  bool serviceAvailable(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(serviceAvailable);

  // Now test service requests and responses
  bool success;
  homebot::HADoor srv;

  // Should not be door number 0 or a door greater than doorCount
  srv.request.action = homebot::HADoorRequest::STATUS;
  srv.request.doorNumber = 0;
  success = client.call(srv);
  EXPECT_FALSE(success);
  srv.request.doorNumber = doorCount + 1;
  success = client.call(srv);
  EXPECT_FALSE(success);

  // The doors should all initialize to closed
  srv.request.action = homebot::HADoorRequest::STATUS;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::CLOSED, srv.response.state);
  }

  // Should be able to open all of the doors
  srv.request.action = homebot::HADoorRequest::OPEN;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::OPENED, srv.response.state);
  }

  // Now all of the doors should be open
  srv.request.action = homebot::HADoorRequest::STATUS;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::OPENED, srv.response.state);
  }

  // Should be able to close all of the doors
  srv.request.action = homebot::HADoorRequest::CLOSE;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::CLOSED, srv.response.state);
  }

  // Now all of the doors should be closed
  srv.request.action = homebot::HADoorRequest::STATUS;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::CLOSED, srv.response.state);
  }
}

/*******************************************************************************/
TEST (HomeBotBehavior, BotAffectHADoorExecution) {

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;
  // Spin up some clients
  BotOprClients botOprClients;

  std::string oprCode = "HADoor";
  int doorNumber(1);
  int action(homebot::HADoorRequest::STATUS);

  // Test HADoor execution for status
  {
    BotAffectHADoorOpr doorOpr(oprCode, doorNumber, action);
    EXPECT_EQ(oprCode, doorOpr.getCode());
    homebot::HADoorRequest doorReq = doorOpr.details();
    EXPECT_EQ(doorNumber, doorReq.doorNumber);
    EXPECT_EQ(action, doorReq.action);

    // This should work if we have a server running...
    EXPECT_TRUE(doorOpr.execute(botOprClients));
  }

  // Test HADoor execution for close
  action = homebot::HADoorRequest::CLOSE;
  {
    BotAffectHADoorOpr doorOpr(oprCode, doorNumber, action);
    EXPECT_EQ(oprCode, doorOpr.getCode());
    homebot::HADoorRequest doorReq = doorOpr.details();
    EXPECT_EQ(doorNumber, doorReq.doorNumber);
    EXPECT_EQ(action, doorReq.action);

    // This should work if we have a server running...
    EXPECT_TRUE(doorOpr.execute(botOprClients));
  }

  // Test HADoor execution for open
  action = homebot::HADoorRequest::OPEN;
  {
    BotAffectHADoorOpr doorOpr(oprCode, doorNumber, action);
    EXPECT_EQ(oprCode, doorOpr.getCode());
    homebot::HADoorRequest doorReq = doorOpr.details();
    EXPECT_EQ(doorNumber, doorReq.doorNumber);
    EXPECT_EQ(action, doorReq.action);

    // This should work if we have a server running...
    EXPECT_TRUE(doorOpr.execute(botOprClients));
  }

  // Test HADoor execution for invalid action
  action = 5;
  {
    BotAffectHADoorOpr doorOpr(oprCode, doorNumber, action);
    EXPECT_EQ(oprCode, doorOpr.getCode());
    homebot::HADoorRequest doorReq = doorOpr.details();
    EXPECT_EQ(doorNumber, doorReq.doorNumber);
    EXPECT_EQ(action, doorReq.action);

    // This should work if we have a server running...
    EXPECT_FALSE(doorOpr.execute(botOprClients));
  }

  // Test HADoor execution for invalid door number
  // (assumes server was started with only 5 doors)
  action = homebot::HADoorRequest::OPEN;
  doorNumber = 99;
  {
    BotAffectHADoorOpr doorOpr(oprCode, doorNumber, action);
    EXPECT_EQ(oprCode, doorOpr.getCode());
    homebot::HADoorRequest doorReq = doorOpr.details();
    EXPECT_EQ(doorNumber, doorReq.doorNumber);
    EXPECT_EQ(action, doorReq.action);

    // This should work if we have a server running...
    EXPECT_FALSE(doorOpr.execute(botOprClients));
  }

}

/*******************************************************************************/
TEST (HomeBotBehavior, BotAffectHAScene) {

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;

  std::string oprCode = "HAScene";
  int sceneNumber(3);
  int action(1);

  // Construct a HAScene operation and determine if the components are as specified
  BotAffectHASceneOpr sceneOpr(oprCode, sceneNumber, action);
  EXPECT_EQ(oprCode, sceneOpr.getCode());
  homebot::HASceneRequest sceneReq = sceneOpr.details();
  EXPECT_EQ(sceneNumber, sceneReq.sceneNumber);
  EXPECT_EQ(action, sceneReq.action);
}

/*******************************************************************************/
TEST (HomeBotBehavior, BotAffectHAShade) {

 // Create a node handle since we are running as a ROS node
 ros::NodeHandle nh;

 std::string oprCode = "HAShade";
 int shadeNumber(5);
 int action(1);

 // Construct a HAScene operation and determine if the components are as specified
  BotAffectHAShade shadeOpr(oprCode, shadeNumber, action);
 EXPECT_EQ(oprCode, shadeOpr.getCode());
  homebot::HAShade::Request shadeReq = shadeOpr.details();
 EXPECT_EQ(shadeNumber, shadeReq.shadeNumber);
 EXPECT_EQ(action, shadeReq.action);
}

/*******************************************************************************/
TEST (HomeBotBehavior, BotOprClients) {

  // Instantiate the BotOprClients object; NO SERVERS SHOULD BE RUNNING FOR THIS TEST

  BotOprClients botOprClients;

// The clients should be reported as not available
  EXPECT_FALSE(botOprClients.allStarted());
}

/*******************************************************************************/

/*
 * @brief Initialize ROS and run all of the tests defined in the test macros
 */
int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "BotBehavior_Component_Standalone_test");

  // Run all of the Google Test defined tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
