/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HADemoService.cpp
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

#include "homebot/HADemoService.hpp"

HADemoService::HADemoService(HAClients& pHAClients)
    : haClients(pHAClients),
      nh() {
  // Set the service server object using the node handle's advertiseService method,
  // the service name, and the callback method from this object
  ss = nh.advertiseService("ha_demo", &HADemoService::callback, this);
  ROS_INFO_STREAM(
      "HomeBot-HADemoService(constructor): Initialized ha_demo service");
}

HADemoService::~HADemoService() {
}

/**
 * @brief Service callback for the Home Automation Demo service - handles service calls
 * @param [in] req data specifying the request details (behavior, repetitions)
 * @param [out] rsp data going back to the service requestor (result)
 * @return boolean success or failure of the service call
 */
bool HADemoService::callback(homebot::HADemo::Request& req,
                             homebot::HADemo::Response& rsp) {
  // Validate that the repetition count is between 1 and 10 (to prevent mistaken invocations)
  if (req.repetitions < 1 || req.repetitions > 10) {
    ROS_WARN_STREAM(
        "HomeBot-HADemoService(callback): Invalid number of repetitions '" << static_cast<int>(req.repetitions) << "' requested, no action initiated");
    rsp.result = "Rejected: Invalid number of repetitions requested";
    return false;
  }
  // Validate that the behavior requested is not null
  if (req.behavior == "") {
    ROS_WARN_STREAM(
        "Null behavior requested, no action taken");
    rsp.result = "Rejected: Null behavior requested";
    return false;
  }
  // Check whether the action is still available
  if (!haClients.acHBBehavior.isServerConnected()) {
    ROS_WARN_STREAM(
        "HomeBot-HADemoService(callback): bot_actor action server not ready when trying to activate behavior '" << req.behavior << "' with " << static_cast<int>(req.repetitions) << " repetitions");
    rsp.result = "Failed: action server not ready";
    return false;
  }

  ROS_INFO_STREAM(
      "HomeBot-HADemoService(callback): sending goal to bot_actor action server (behavior '" << req.behavior << "' with " << static_cast<int>(req.repetitions) << " repetitions)");
  // Send goal to move_base action server, then wait for result
  homebot::HBBehaviorGoal goal;
  goal.behavior = req.behavior;
  goal.repetitions = req.repetitions;
  haClients.acHBBehavior.sendGoal(goal);
  haClients.acHBBehavior.waitForResult();

  if (haClients.acHBBehavior.getState()
      == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("HomeBot-HADemoService(callback): Behavior goal reached");
    rsp.result = "Joy: Behavior completed successfully";
    return true;
  } else {
    ROS_WARN_STREAM(
        "HomeBot-HADemoService(callback): Failed to reach behavior goal");
    rsp.result = "Sadness: Behavior did not complete successfully";
    return false;
  }
}
