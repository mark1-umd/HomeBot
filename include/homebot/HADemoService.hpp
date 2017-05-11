/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HADemoService.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 10, 2017 - Creation
 *
 * @brief Provides a Home Automation "demo" service (sends HBBehavior goals to bot_actor action server)
 *
 * This service provides a way to manually activate Home Automation system requests
 * for BotBehaviors by accepting a service request containing a behavior name and
 * a number of repetitions, then using an action client to send the same behavior
 * name and repetitions to the bot_actor action server, which executes the behavior
 * (if present in its repertoire).  When the BotBehavior action ends, the service
 * call will return to the requestor with an outcome (reached the goal, or not).
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_HADEMOSERVICE_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_HADEMOSERVICE_HPP_

#include <string>
#include "ros/ros.h"
#include "homebot/HADemo.h"
#include "homebot/HBBehaviorAction.h"
#include "homebot/HAClients.hpp"



/** @brief Provides HA Demo Service to a ROS node acting as a Home Automation Demo Server
 */

class HADemoService {
 public:
  HADemoService(HAClients& haClients);
  virtual ~HADemoService();
  bool callback(homebot::HADemo::Request& req, homebot::HADemo::Response& rsp);
  void init();
 private:
  ros::NodeHandle nh;
  HAClients& haClients;
  ros::ServiceServer ss;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_HADEMOSERVICE_HPP_ */
