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
#include <sstream>
#include <iostream>
#include "boost/shared_ptr.hpp"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "homebot/BotOperation.hpp"
#include "homebot/OperationParameters.hpp"
#include "homebot/BotMoveBaseOpr.hpp"
#include "homebot/BotAffectHADoorOpr.hpp"
#include "homebot/BotAffectHASceneOpr.hpp"
#include "homebot/BotAffectHAShadeOpr.hpp"
#include "homebot/BotOprClients.hpp"


// These are not comprehensive tests; they exist merely to establish that the basic
// function of the class is operational.  Detailed testing, including testing of
// conditions that should result in failure, is conducted in the non-standalone
// version of the BotBehavior_Component tests.

/*******************************************************************************/
TEST(HomeBotBehavior, BotOprClients) {

  // Instantiate the BotOprClients object; NO SERVERS SHOULD BE RUNNING FOR THIS TEST
  BotOprClients botOprClients;

  // The clients should be reported as not available
  EXPECT_FALSE(botOprClients.allStarted());
}

/*******************************************************************************/
TEST(HomeBotBehavior, BotMoveBase) {

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;
  // Specify some parameters to use in testing operation validity
  OperationParameters opParams(5, 15, 8);  // 5 doors, 15 scenes, and 8 shades

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

  // The operation should pass validation
  EXPECT_TRUE(mbOpr.isExecutable(opParams));

  // But the clients should not have any servers because these are standalone tests
  // Note: we can't repeat this test in any other sections or else the test times out
  BotOprClients botOprClients;
  EXPECT_FALSE(botOprClients.allStarted());

  // And so the operation should not be able to execute
  EXPECT_FALSE(mbOpr.execute(botOprClients));
}

/*******************************************************************************/
TEST(HomeBotBehavior, BotAffectHADoor) {

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;
  // Specify some parameters to use in testing operation validity
  OperationParameters opParams(5, 15, 8);  // 5 doors, 15 scenes, and 8 shades

  std::string oprCode = "HADoor";
  int doorNumber(1);
  int action(2);

  // Construct a HADoor operation and determine if the components are as specified
  BotAffectHADoorOpr doorOpr(oprCode, doorNumber, action);
  EXPECT_EQ(oprCode, doorOpr.getCode());
  homebot::HADoorRequest doorReq = doorOpr.details();
  EXPECT_EQ(doorNumber, doorReq.doorNumber);
  EXPECT_EQ(action, doorReq.action);

  // The operation should pass validation
  EXPECT_TRUE(doorOpr.isExecutable(opParams));

  /***********************************************
   * Cannot execute these tests or else gtest times out
  // But the clients should not be started
  BotOprClients botOprClients;
  EXPECT_FALSE(botOprClients.allStarted());

  // And so the operation should not be able to execute
  EXPECT_FALSE(doorOpr.execute(botOprClients));
   ************************************************/

  // Now create an operation through the base class
  std::string operationComponents("HADoor 1 2");
  BotOperation baseOpr(operationComponents);
  boost::shared_ptr<BotOperation> aDerivedOprInBase = baseOpr.transform(
      opParams);

  // And self-validate through polymorphism/virtual function
  EXPECT_TRUE(aDerivedOprInBase->isExecutable(opParams));
}

/*******************************************************************************/
TEST(HomeBotBehavior, BotAffectHAScene) {

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;
  // Specify some parameters to use in testing operation validity
  OperationParameters opParams(5, 15, 8);  // 5 doors, 15 scenes, and 8 shades

  std::string oprCode = "HAScene";
  int sceneNumber(3);
  int action(1);

  // Construct a HAScene operation and determine if the components are as specified
  BotAffectHASceneOpr sceneOpr(oprCode, sceneNumber, action);
  EXPECT_EQ(oprCode, sceneOpr.getCode());
  homebot::HASceneRequest sceneReq = sceneOpr.details();
  EXPECT_EQ(sceneNumber, sceneReq.sceneNumber);
  EXPECT_EQ(action, sceneReq.action);

  // The operation should pass validation
  EXPECT_TRUE(sceneOpr.isExecutable(opParams));

  /***********************************************
   * Cannot execute these tests or else gtest times out
  // But the clients should not be started
  BotOprClients botOprClients;
  EXPECT_FALSE(botOprClients.allStarted());

  // And so the operation should not be able to execute
  EXPECT_FALSE(sceneOpr.execute(botOprClients));
   ************************************************/

  // Now create an operation through the base class
  std::string operationComponents("HAScene 3 1");
  BotOperation baseOpr(operationComponents);
  boost::shared_ptr<BotOperation> aDerivedOprInBase = baseOpr.transform(
      opParams);

  // And self-validate through polymorphism/virtual function
  EXPECT_TRUE(aDerivedOprInBase->isExecutable(opParams));
}

/*******************************************************************************/
TEST(HomeBotBehavior, BotAffectHAShade) {

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;
  // Specify some parameters to use in testing operation validity
  OperationParameters opParams(5, 15, 8);  // 5 doors, 15 scenes, and 8 shades

  std::string oprCode = "HAShade";
  int shadeNumber(5);
  int action(1);

  // Construct a HAScene operation and determine if the components are as specified
  BotAffectHAShadeOpr shadeOpr(oprCode, shadeNumber, action);
  EXPECT_EQ(oprCode, shadeOpr.getCode());
  homebot::HAShade::Request shadeReq = shadeOpr.details();
  EXPECT_EQ(shadeNumber, shadeReq.shadeNumber);
  EXPECT_EQ(action, shadeReq.action);

  // The operation should pass validation
  EXPECT_TRUE(shadeOpr.isExecutable(opParams));

  /***********************************************
   * Cannot execute these tests or else gtest times out
  // But the clients should not be started
  BotOprClients botOprClients;
  EXPECT_FALSE(botOprClients.allStarted());

  // And so the operation should not be able to execute
  EXPECT_FALSE(shadeOpr.execute(botOprClients));
   ************************************************/

  // Now create an operation through the base class
  std::string operationComponents("HAShade 5 1");
  BotOperation baseOpr(operationComponents);
  boost::shared_ptr<BotOperation> aDerivedOprInBase = baseOpr.transform(
      opParams);

  // And self-validate through polymorphism/virtual function
  EXPECT_TRUE(aDerivedOprInBase->isExecutable(opParams));
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
