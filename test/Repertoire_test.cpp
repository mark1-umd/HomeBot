/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotBehavior_Component_test.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 6, 2017 - Creation
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
#include "homebot/OperationParameters.hpp"
#include "homebot/Repertoire.hpp"


/*******************************************************************************/

TEST(Repertoire, Repertoire) {
  // Define the operation parameters for this test
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node and use rosconsole
  ros::NodeHandle nh;


  Repertoire buttleBotRepertoire("ButtleBot");

  // Bad filename
  EXPECT_FALSE(
      buttleBotRepertoire.load(
          "/home/viki/ROS/hw/catkin_ws/src/homebot/repertoire/ButtleBot-test-nofile.rpt",
          opParams));
  // Bot name in file doesn't match this bot
  EXPECT_FALSE(
      buttleBotRepertoire.load(
          "/home/viki/ROS/hw/catkin_ws/src/homebot/repertoire/ButtleBot-test-badbot.rpt",
          opParams));
  // Bad operation in otherwise good file
  EXPECT_FALSE(
      buttleBotRepertoire.load(
          "/home/viki/ROS/hw/catkin_ws/src/homebot/repertoire/ButtleBot-test-badopr.rpt",
          opParams));
  // Good file
  EXPECT_TRUE(
      buttleBotRepertoire.load(
          "/home/viki/ROS/hw/catkin_ws/src/homebot/repertoire/ButtleBot-test-good.rpt",
          opParams));
  // Behavior that's not in the file
  BotBehavior behavior = buttleBotRepertoire.getBehavior("CleanCat");
  EXPECT_EQ("", behavior.getName());
  // A behavior in the file
  behavior = buttleBotRepertoire.getBehavior("CloseForNight");
  EXPECT_EQ("CloseForNight", behavior.getName());
  // A behavior in the file
  behavior = buttleBotRepertoire.getBehavior("AnswerFrontDoor");
  EXPECT_EQ("AnswerFrontDoor", behavior.getName());
  // A behavior in the file
  behavior = buttleBotRepertoire.getBehavior("PrepareEveningEntertain");
  EXPECT_EQ("PrepareEveningEntertain", behavior.getName());
  // Behavior that's not in the file
  behavior = buttleBotRepertoire.getBehavior("WashPlane");
  EXPECT_EQ("", behavior.getName());
}

/*******************************************************************************/

/*
 * @brief Initialize ROS and run all of the tests defined in the test macros
 */
int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "Repertoire_test");

  // Run all of the Google Test defined tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
