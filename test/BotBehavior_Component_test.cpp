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
#include <boost/shared_ptr.hpp>
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "homebot/BotOperation.hpp"
#include "homebot/OperationParameters.hpp"
#include "homebot/BotMoveBaseOpr.hpp"
#include "homebot/BotAffectHADoorOpr.hpp"
#include "homebot/BotAffectHASceneOpr.hpp"
#include "homebot/BotAffectHAShadeOpr.hpp"
#include "homebot/BotOprClients.hpp"


/*******************************************************************************

TEST (BotBehaviorOprClients, StartUp) {
  // Create a node handle since we are running as a ROS node and use rosconsole
  ros::NodeHandle nh;

  // Instantiate the BotOprClients object;
  BotOprClients botOprClients;

  // The clients should be reported as not available if they are not all started
  // (and the move_base server is not currently being started)
  EXPECT_FALSE(botOprClients.allStarted());
}
/*******************************************************************************/

TEST (BotBehaviorBaseOpr, Construction) {
  // Define the operation parameters for this test
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node and use rosconsole
  ros::NodeHandle nh;

  // Crank up bot clients just to create enough delay to get some console logging for tests
  BotOprClients botOprClients;

  // Constructors on base with no input
  {
    BotOperation baseOpr;
    EXPECT_EQ("", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should have a null code
    EXPECT_EQ("", someOpr->getCode());
  }

  // Constructors on base with null raw text - invalid result
  {
    BotOperation baseOpr("");
    EXPECT_EQ("", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should have a null code
    EXPECT_EQ("", someOpr->getCode());
  }

  // Constructors on base with "Bogus" raw text - invalid result
  {
    BotOperation baseOpr("Bogus");
    EXPECT_EQ("", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "Bogus" code
    EXPECT_EQ("Bogus", someOpr->getCode());
  }

  // Constructors on base with "HADoor" raw text - invalid result
  {
    BotOperation baseOpr("HADoor");
    EXPECT_EQ("", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "HADoor" code
    EXPECT_EQ("HADoor", someOpr->getCode());
  }

  // Constructors on base with "HADoor garbage" raw text - invalid result
  {
    BotOperation baseOpr("HADoor garbage");
    EXPECT_EQ("", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "HADoor" code
    EXPECT_EQ("HADoor", someOpr->getCode());
  }

  // Constructors on base with "HADoor garbage garbage" raw text - invalid result
  {
    BotOperation baseOpr("HADoor garbage1 in garbage2 our");
    EXPECT_EQ("", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "HADoor" code
    EXPECT_EQ("HADoor", someOpr->getCode());
  }

  // Constructors on base with null raw text, null code - invalid result
  {
    BotOperation baseOpr("", "");
    EXPECT_EQ("", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a null code
    EXPECT_EQ("", someOpr->getCode());
  }

  // Constructors on base with null raw text, Bogus code - invalid result
  {
    BotOperation baseOpr("", "Bogus");
    EXPECT_EQ("Bogus", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "Bogus" code
    EXPECT_EQ("Bogus", someOpr->getCode());
  }

  // Constructors on base with null raw text, HADoor code - invalid result
  {
    BotOperation baseOpr("", "HADoor");
    EXPECT_EQ("HADoor", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "HADoor" code
    EXPECT_EQ("HADoor", someOpr->getCode());
  }

  // Constructors on base with garbage raw text, null code - invalid result
  {
    BotOperation baseOpr("garbage1 in garbage2 out", "");
    EXPECT_EQ("", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "garbage" code
    EXPECT_EQ("garbage1", someOpr->getCode());
  }

  // Constructors on base with garbage raw text, Bogus code - invalid result
  {
    BotOperation baseOpr("garbage1 in garbage2 out", "Bogus");
    EXPECT_EQ("Bogus", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "Bogus" code
    EXPECT_EQ("Bogus", someOpr->getCode());
  }

  // Constructors on base with garbage raw text, HADoor code - invalid result
  {
    BotOperation baseOpr("garbage1 in garbage2 out", "HADoor");
    EXPECT_EQ("HADoor", baseOpr.getCode());
    EXPECT_FALSE(baseOpr.isExecutable(opParams));
    boost::shared_ptr<BotOperation> someOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(someOpr->isExecutable(opParams));
    // the returned Opr should a "Bogus" code
    EXPECT_EQ("HADoor", someOpr->getCode());
  }
}
/*******************************************************************************/

TEST (BotBehaviorMoveBase, Construction) {
  // Define the operation parameters for this test
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node and use rosconsole
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

  // Construct a BotMoveBase operation manually and determine if the components are as specified
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

  // Validate the MoveBase operation constructed above
  EXPECT_TRUE(mbOpr.isExecutable(opParams));

  // Now create an operation through the base class method transform
  BotOperation baseOpr("BotMoveBase map 23 17 11 2 3 5 1");
  boost::shared_ptr<BotOperation> aDerivedOprInBase = baseOpr.transform(
      opParams);

  // And self-validate through polymorphism/virtual function
  EXPECT_TRUE(aDerivedOprInBase->isExecutable(opParams));
}
/*******************************************************************************/

TEST (BotBehaviorMoveBase, Execution) {
  // Define the operation parameters based on the Request Server initialization
  // (Assume Request Server started with -doors 5 -scenes 15 -shades 8
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node with rosconsole
  ros::NodeHandle nh;
  // Spin up some clients
  BotOprClients botOprClients;

  // default test parameters
  std::string oprCode("BotMoveBase");
  std::string frame_id("map");
  int xPos(23);
  int yPos(17);
  int zPos(11);
  int xOrient(2);
  int yOrient(3);
  int zOrient(5);
  int wOrient(1);

  // Test HADoor execution for status (without a move_base action server)
  {
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << frame_id << " " << xPos << " " << yPos << " "
             << zPos << " " << xOrient << " " << yOrient << " " << zOrient
             << " " << wOrient;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> doorOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(doorOpr->isExecutable(opParams));
    EXPECT_FALSE(doorOpr->execute(botOprClients));
  }
}
/*******************************************************************************/

TEST (BotBehaviorAffectHADoor, Construction) {
  // Define the operation parameters for this test
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node and use rosconsole
  ros::NodeHandle nh;

  std::string oprCode = "HADoor";
  int doorNumber(1);
  int action(2);

  // Construct a HADoor operation manually and determine if the components are as specified
  BotAffectHADoorOpr doorOpr(oprCode, doorNumber, action);
  EXPECT_EQ(oprCode, doorOpr.getCode());
  homebot::HADoorRequest doorReq = doorOpr.details();
  EXPECT_EQ(doorNumber, doorReq.doorNumber);
  EXPECT_EQ(action, doorReq.action);

  // Validate the HADoor operation constructed above
  EXPECT_TRUE(doorOpr.isExecutable(opParams));

  // Now create an operation through the base class method transform
  BotOperation baseOpr("HADoor 1 2");
  boost::shared_ptr<BotOperation> aDerivedOprInBase = baseOpr.transform(
      opParams);

  // And self-validate through polymorphism/virtual function
  EXPECT_TRUE(aDerivedOprInBase->isExecutable(opParams));
}
/*******************************************************************************/

TEST (BotBehaviorAffectHADoor, Execution) {
  // Define the operation parameters based on the Request Server initialization
  // (Assume Request Server started with -doors 5 -scenes 15 -shades 8
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node with rosconsole
  ros::NodeHandle nh;
  // Spin up some clients
  BotOprClients botOprClients;

  std::string oprCode = "HADoor";
  int doorNumber(1);

  // Test HADoor execution for status
  {
    int action(homebot::HADoorRequest::STATUS);
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << doorNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> doorOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(doorOpr->isExecutable(opParams));
    EXPECT_TRUE(doorOpr->execute(botOprClients));
  }

  // Test HADoor execution for close
  {
    int action = homebot::HADoorRequest::CLOSE;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << doorNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> doorOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(doorOpr->isExecutable(opParams));
    EXPECT_TRUE(doorOpr->execute(botOprClients));
  }

  // Test HADoor execution for open
  {
    int action = homebot::HADoorRequest::OPEN;

    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << doorNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> doorOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(doorOpr->isExecutable(opParams));
    EXPECT_TRUE(doorOpr->execute(botOprClients));

    // This should work if we have a server running...
    EXPECT_TRUE(doorOpr->execute(botOprClients));
  }

  // Test HADoor execution for invalid action
  {
    int action = 5;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << doorNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> doorOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(doorOpr->isExecutable(opParams));
    EXPECT_FALSE(doorOpr->execute(botOprClients));
  }

  // Test HADoor execution for invalid door number
  // (assumes server was started with only 5 doors)
  {
    int doorNumber = 99;
    int action = homebot::HADoorRequest::OPEN;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << doorNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> doorOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(doorOpr->isExecutable(opParams));
    EXPECT_FALSE(doorOpr->execute(botOprClients));
  }
}
/*******************************************************************************/

TEST (BotBehaviorAffectHAScene, Construction) {
  // Define the operation parameters for this test
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node and use rosconsole
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

  // Validate the HADoor operation constructed above
  EXPECT_TRUE(sceneOpr.isExecutable(opParams));

  // Now create an operation through the base class method transform
  BotOperation baseOpr("HAScene 3 1");
  boost::shared_ptr<BotOperation> aDerivedOprInBase = baseOpr.transform(
      opParams);

  // And self-validate through polymorphism/virtual function
  EXPECT_TRUE(aDerivedOprInBase->isExecutable(opParams));
}
/*******************************************************************************/

TEST (BotBehaviorAffectHAScene, Execution) {
  // Define the operation parameters based on the Request Server initialization
  // (Assume Request Server started with -doors 5 -scenes 15 -shades 8
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;
  // Spin up some clients
  BotOprClients botOprClients;

  // Default testing parameters
  std::string oprCode = "HAScene";
  int sceneNumber(3);

  // Test HAScene execution for status
  {
    int action(homebot::HASceneRequest::STATUS);
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << sceneNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> sceneOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(sceneOpr->isExecutable(opParams));
    EXPECT_TRUE(sceneOpr->execute(botOprClients));
  }

  // Test HAScene execution for turn off
  {
    int action = homebot::HASceneRequest::TURNOFF;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << sceneNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> sceneOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(sceneOpr->isExecutable(opParams));
    EXPECT_TRUE(sceneOpr->execute(botOprClients));
  }

  // Test HAScene execution for turn on
  {
    int action = homebot::HASceneRequest::TURNON;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << sceneNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> sceneOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(sceneOpr->isExecutable(opParams));
    EXPECT_TRUE(sceneOpr->execute(botOprClients));
  }

  // Test HAScene execution for invalid action
  {
    int action = 5;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << sceneNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> sceneOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(sceneOpr->isExecutable(opParams));
    EXPECT_FALSE(sceneOpr->execute(botOprClients));
  }

  // Test HAScene execution for invalid scene number
  // (assumes server was started with only 15 scenes)
  {
    int sceneNumber = 99;
    int action = homebot::HASceneRequest::TURNON;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << sceneNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> sceneOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(sceneOpr->isExecutable(opParams));
    EXPECT_FALSE(sceneOpr->execute(botOprClients));
  }
}
/*******************************************************************************/

TEST (BotBehaviorAffectHAShade, Construction) {
  // Define the operation parameters for this test
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;

  std::string oprCode = "HAShade";
  int shadeNumber(5);
  int action(1);

  // Construct a HAScene operation and determine if the components are as specified
  BotAffectHAShadeOpr shadeOpr(oprCode, shadeNumber, action);
  EXPECT_EQ(oprCode, shadeOpr.getCode());
  homebot::HAShade::Request shadeReq = shadeOpr.details();
  EXPECT_EQ(shadeNumber, shadeReq.shadeNumber);
  EXPECT_EQ(action, shadeReq.action);

  // Validate the HADoor operation constructed above
  EXPECT_TRUE(shadeOpr.isExecutable(opParams));

  // Now create an operation through the base class method transform
  BotOperation baseOpr("HADoor 1 2");
  boost::shared_ptr<BotOperation> aDerivedOprInBase = baseOpr.transform(
      opParams);

  // And self-validate through polymorphism/virtual function
  EXPECT_TRUE(aDerivedOprInBase->isExecutable(opParams));
}
/*******************************************************************************/

TEST (BotBehaviorAffectHAShade, Execution) {
  // Define the operation parameters based on the Request Server initialization
  // (Assume Request Server started with -doors 5 -scenes 15 -shades 8
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node
  ros::NodeHandle nh;
  // Spin up some clients
  BotOprClients botOprClients;

  std::string oprCode = "HAShade";
  int shadeNumber(3);

  // Test HAShade execution for status
  {
    int action(homebot::HAShadeRequest::STATUS);
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << shadeNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> shadeOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(shadeOpr->isExecutable(opParams));
    EXPECT_TRUE(shadeOpr->execute(botOprClients));
  }

  // Test HAShade execution for raise
  {
    int action = homebot::HAShadeRequest::RAISE;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << shadeNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> shadeOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(shadeOpr->isExecutable(opParams));
    EXPECT_TRUE(shadeOpr->execute(botOprClients));
  }

  // Test HAShade execution for lower
  {
    int action = homebot::HAShadeRequest::LOWER;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << shadeNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> shadeOpr = baseOpr.transform(opParams);
    EXPECT_TRUE(shadeOpr->isExecutable(opParams));
    EXPECT_TRUE(shadeOpr->execute(botOprClients));
  }

  // Test HAShade execution for invalid action
  {
    int action = 5;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << shadeNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> shadeOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(shadeOpr->isExecutable(opParams));
    EXPECT_FALSE(shadeOpr->execute(botOprClients));
  }

  // Test HAShade execution for invalid hade number
  // (assumes server was started with only 8 shades)
  {
    int shadeNumber = 99;
    int action = homebot::HAShadeRequest::LOWER;
    std::ostringstream ssRawOpr;
    ssRawOpr << oprCode << " " << shadeNumber << " " << action;
    std::string rawOpr = ssRawOpr.str();
    BotOperation baseOpr(rawOpr);
    boost::shared_ptr<BotOperation> shadeOpr = baseOpr.transform(opParams);
    EXPECT_FALSE(shadeOpr->isExecutable(opParams));
    EXPECT_FALSE(shadeOpr->execute(botOprClients));
  }

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
