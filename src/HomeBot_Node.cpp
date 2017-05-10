/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HomeBot_Node.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 9, 2017 - Creation
 *
 * @brief A HomeBot Node is the HomeBot Behavior controller for a  HomeBot service robot
 *
 * The HomeBot system creates an interface between typical Home Automation controllers and
 * HomeBot service robots.  This interface works both ways: the HomeBot service robot behaviors
 * include operations that control home systems that are connected to the Home Automation
 * system through the Home Automation system, and the Home Automation system (and the things
 * that are attached to it) can initiate HomeBot service robot behaviors.  An instance of this
 * node is run within the ROS system for each HomeBot service robot.  This node and
 * its BotActor action server receive behavior action goals sent from other ROS nodes, and
 * upon receipt carry out the behaviors by executing the individual operations that are part
 * of each behavior.  At the same time, this node establishes service clients that the individual
 * operations use to send service requests to other ROS nodes, such as the HA Request Server
 * (which is itself connected to the Home Automation system).
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
#include "ros/ros.h"
#include "homebot/OperationParameters.hpp"
#include "homebot/Repertoire.hpp"
#include "homebot/BotOprClients.hpp"
#include "homebot/BotActor.hpp"

// Become a real node
int main(int argc, char **argv) {
  // Initialize the ROS component of the node, which strips some ROS-specific arguments
  // off of the command line
  ros::init(argc, argv, "Bot_Node");

  // Process command line arguments left after ROS strips off remapping arguments; these
  // arguments both customize the HomeBot Node to interface with a specific HomeBot
  // service robot type (BotType) and establish limits for elements of the Home Automation
  // system used to ensure that HomeBot behavior operations are within those limits when
  // the system is initialized.
  std::string botType;
  botType = "<unspecified>Bot";
  std::string rptFile;
  rptFile = "";
  int doorCount = 0;
  int sceneCount = 0;
  int shadeCount = 0;

  for (int i = 1; i < argc; i++) {
    if (i + 1 != argc) {
      // If we match the parameter
      if (strcmp(argv[i], "-doors") == 0) {
        // Grab the number of doors for the HADoorService; should be an integer
        doorCount = atoi(argv[i + 1]);
        // Skip the parameter value
        i++;
      } else if (strcmp(argv[i], "-scenes") == 0) {
        // Grab the number of scenes for the HASceneService; should be an integer
        sceneCount = atoi(argv[i + 1]);
        // Skip the parameter value
        i++;
      } else if (strcmp(argv[i], "-shades") == 0) {
        // Grab the number of shades for the HAShadeService; should be an integer
        shadeCount = atoi(argv[i + 1]);
        // Skip the parameter value
        i++;
      } else if (strcmp(argv[i], "-botType") == 0) {
        // Grab the HomeBot Bot type; should be a string
        botType = argv[i + 1];
        //Skip the parameter value
        i++;
      } else if (strcmp(argv[i], "-rptFile") == 0) {
        // Grab the repertoire filename; should be a string
        rptFile = argv[i + 1];
        //Skip the parameter value
        i++;
      }
    }
  }

  // Use the command line arguments to establish the parameters for HomeBot Operations
  // (doors, scenes, shades)
  OperationParameters opParams(doorCount, sceneCount, shadeCount);

  // ROS startup happens when first node handle is created; shutdown should occur
  // automatically when last node handle is destroyed or SIGINT captures Ctrl-C
  ros::NodeHandle nh;
  ROS_INFO_STREAM(
      "HomeBot(main): '" << botType << "' HomeBot_Node is starting");
  ROS_INFO_STREAM(
      "HomeBot(main): Initialized with operational parameters doors=" << doorCount << ", scenes=" << sceneCount << ", shades=" << shadeCount);

  // Load our repertoire of behaviors so that we can carry them out on command
  Repertoire botRepertoire(botType);
  if (!botRepertoire.load(rptFile, opParams)) {
    ROS_FATAL_STREAM(
        "HomeBot(main): Exiting - could not load repertoire for '" << botType << "' from file '" << rptFile << "'");
    return 1;
  }

  // Start up the Bot Clients
  BotOprClients oprClients;
  if (!oprClients.allStarted()) {
    ROS_WARN_STREAM(
        "HomeBot(main): Some services not immediately available for '" << botType << "'; behaviors may be impacted");
  }

  // And create the BotActor, which will start the action server and manage goal-seeking behaviors
  BotActor botActor(botRepertoire, oprClients);
  ROS_INFO_STREAM(
      "HomeBot(main): Startup complete; behavioral services available");

  // wait for shutdown to be requested
  ros::spin();

  ROS_INFO_STREAM(
      "HomeBot(main): HomeBot_Node for '" << botType << "' shutting down");
  return 0;
}
