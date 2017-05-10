/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotActor_test.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 9, 2017 - Creation
 *
 * @brief ROS test node for the HomeBot Bot BotActor
 *
 * This ROS node is executed under the rostest environment to test the HomeBot Bot BotActor
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
#include "homebot/BotOprClients.hpp"
#include "homebot/BotActor.hpp"


/*******************************************************************************/

TEST (BotActor, ButtleBot) {
  // Define the operation parameters for this test
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node and use rosconsole
  ros::NodeHandle nh;


  Repertoire buttleBotRepertoire("ButtleBot");
  EXPECT_TRUE(
      buttleBotRepertoire.load(
          "/home/viki/ROS/hw/catkin_ws/src/homebot/repertoire/ButtleBot-test-good.rpt",
          opParams));

  // Start up the Bot Clients
  BotOprClients oprClients;

  // And create the BotActor, which will start things going
  BotActor botActor(buttleBotRepertoire, oprClients);

  // There is no output from BotActor; if this was a main function we would just
  // use ros::spin() here and wait until the node was shutdown
}

/*******************************************************************************/

TEST (BotActor, WatchBot) {
  // Define the operation parameters for this test
  // doors = 5, scenes = 15, shades = 8
  OperationParameters opParams(5, 15, 8);

  // Create a node handle since we are running as a ROS node and use rosconsole
  ros::NodeHandle nh;

  Repertoire watchBotRepertoire("WatchBot");
  EXPECT_TRUE(
      watchBotRepertoire.load(
          "/home/viki/ROS/hw/catkin_ws/src/homebot/repertoire/WatchBot-test-good.rpt",
          opParams));

  // Start up the Bot Clients
  BotOprClients oprClients;

  // And create the BotActor, which will start things going
  BotActor botActor(watchBotRepertoire, oprClients);

  // There is no output from BotActor; if this was a main function we would just
  // use ros::spin() here and wait until the node was shutdown
}

/*******************************************************************************/

/*
 * @brief Initialize ROS and run all of the tests defined in the test macros
 */
int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "BotActor_test");

  // Run all of the Google Test defined tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
