/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotOprClients.hpp
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_BOTOPRCLIENTS_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_BOTOPRCLIENTS_HPP_

#include "ros/ros.h"
#include "homebot/HADoor.h"
#include "homebot/HAScene.h"
#include "homebot/HAShade.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

/** @brief An object to hold ROS action/service clients needed to perform Bot operations in the HomeBot system
 */

class BotOprClients {
  friend class BotAffectHADoorOpr;
  friend class BotAffectHASceneOpr;
  friend class BotAffectHAShadeOpr;
  friend class BotMoveBaseOpr;
 public:
  BotOprClients();
  virtual ~BotOprClients();
  bool allStarted();
 private:
  const int totalClients;
  int clientsStarted;
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> acBotMoveBase;
  ros::ServiceClient scHADoor;
  ros::ServiceClient scHAScene;
  ros::ServiceClient scHAShade;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_BOTOPRCLIENTS_HPP_ */
