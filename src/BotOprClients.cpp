/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotOprClients.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 6, 2017 - Creation
 *
 * @brief Holds a set of ROS service and action clients necessary to execute Bot Operations
 *
 * The HomeBot Bot Operations are an implementation of behaviors for HomeBot service robots.
 * Each behavior is a set of lower-level operations that act on various entities in the
 * HomeBot system.  For now, those entities are a HomeBot mobile base action server, and various
 * Home Automation services (Doors, Scenes, and Shades).  In order for the operations to
 * execute against the services and actions, clients for the service and actions have to be
 * available.  Rather than continuously start and stop these clients, this Class provides a
 * way to start them up when a ROS node initializes, and to hold the client objects so that
 * they can be used by methods in the instructions when they are executing (assuming that this
 * BotOprClients object is made available to each instruction).
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

#include "homebot/BotOprClients.hpp"

BotOprClients::BotOprClients()
    : totalClients(4),
      clientsStarted(0),
      nh(),
      acBotMoveBase("move_base", true) {
  ROS_INFO_STREAM(
      "HomeBot-BotOprClients(constructor): Starting clients for Bot Operations");

  // Start the services that weren't started above
  scHADoor = nh.serviceClient<homebot::HADoor>("ha_door");
  scHAScene = nh.serviceClient<homebot::HAScene>("ha_scene");
  scHAShade = nh.serviceClient<homebot::HAShade>("ha_shade");

  // Check the move_base action server
  if (!acBotMoveBase.waitForServer(ros::Duration(5.0)))
    ROS_WARN_STREAM(
        "HomeBot-BotOprClients(constructor): The move_base action server is not available");
  else
    clientsStarted++;

  // Check the ha_door service
  if (!scHADoor.waitForExistence(ros::Duration(5.0)))
    ROS_WARN_STREAM(
        "HomeBot-BotOprClients(constructor): The ha_door service is not available");
  else
    clientsStarted++;

  // Check the ha_scene service
  if (!scHAScene.waitForExistence(ros::Duration(5.0)))
    ROS_WARN_STREAM(
        "HomeBot-BotOprClients(constructor): The ha_scene service is not available");
  else
    clientsStarted++;

  // Check he ha_shade service
  if (!scHAShade.waitForExistence(ros::Duration(5.0)))
    ROS_WARN_STREAM(
        "HomeBot-BotOprClients(constructor): The ha_shade service is not available");
  else
    clientsStarted++;
  ROS_DEBUG_STREAM(
      "HomeBot-BotOprClients(constructor): finshed with " << clientsStarted << " clients started out of " << totalClients);
}

BotOprClients::~BotOprClients() {
}

/**
 * @brief Provide status as to whether all of the clients are started (based on whether the services are available)
 * @return boolean indication of whether all clients are started (based on whether services are available)
 */
bool BotOprClients::allStarted() {
  return (clientsStarted == totalClients);
}
