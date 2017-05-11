/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAHomeBotClient_Node.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 10, 2017 - Creation
 *
 * @brief A HA HomeBot Client Node is the Home Automation systems interface into the HomeBot system
 *
 * The HomeBot system creates an interface between typical Home Automation controllers and
 * HomeBot service robots.  This interface works both ways: the HomeBot service robot behaviors
 * include operations that control home systems that are connected to the Home Automation
 * system through the Home Automation system, and the Home Automation system (and the things
 * that are attached to it) can initiate HomeBot service robot behaviors.  An instance of this
 * node is run within the ROS system on behalf of the Home Automation system.  This node and
 * its bot_actor action client(s) receive send behavior action goals to BotActors which carry
 * out the behaviors.  At the same time, another ROS node for the Home Automation system
 * establishes service clients that the individual behavior operations being carried out in
 * the BotActor (as well as other ROS nodes) can use to send service requests to the Home
 * Automation system.
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
#include "homebot/HAClients.hpp"
#include "homebot/HADemoService.hpp"

// Become a real node
int main(int argc, char **argv) {
  // Initialize the ROS component of the node, which strips some ROS-specific arguments
  // off of the command line
  ros::init(argc, argv, "HAClient_Node");

  // ROS startup happens when first node handle is created; shutdown should occur
  // automatically when last node handle is destroyed or SIGINT captures Ctrl-C
  ros::NodeHandle nh;
  ROS_INFO_STREAM(
      "HAHomeBotClient_Node(main): HAHomeBotClient_Node is starting");

  // Start up the HA Clients
  HAClients haClients;

  // Create the HADemoService, which will start the service server and dispatch goals
  // to the bot_actor action server
  HADemoService haDemoService(haClients);

  ROS_INFO_STREAM(
      "HAHomeBotClient_Node(main): Startup complete; client demo services available");

  // wait for shutdown to be requested
  ros::spin();

  ROS_INFO_STREAM(
      "HAHomeBotClient_Node(main): HAHomeBotClient_Node for shutting down");
  return 0;
}
