/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotActor.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 9, 2017 - Creation
 *
 * @brief The HomeBot BotActor is the action server for HomeBot behaviors
 *
 * The HomeBot BotActor receives goals via the ROS actionlib action protocol.  Each goal names a behavior
 * and an number of repetitions for the behavior.  If the behavior exists, and the number of repetitions
 * is within certain parameters, the action server performs the phases of the behavior that causes the
 * behavior activity to be carried out by various parts of the system.
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_BOTACTOR_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_BOTACTOR_HPP_

#include <vector>
#include <string>
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "homebot/HBBehaviorAction.h"
#include "homebot/Repertoire.hpp"

/** @brief An actionlib action server that receives bot behavior actions and carries them out using its repertoire
 */

class BotActor {
 public:
  BotActor(Repertoire& pRepertoire, BotOprClients& pOprClients);
  virtual ~BotActor();
  void actionExecuteCB(const homebot::HBBehaviorGoalConstPtr &goal);
 private:
  ros::NodeHandle nh;
  Repertoire& repertoire;
  BotOprClients& oprClients;
  actionlib::SimpleActionServer<homebot::HBBehaviorAction> as;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_BOTACTOR_HPP_ */
