/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotAffectHAScene.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 4, 2017 - Creation
 *
 * @brief Operation that commands Home Automation system to turn scenes (primarily lighting) on/off
 *
 * In a HomeBot system, the Home Automation system is responsible for turning scenes on/off.
 * This operation provides a way for a HomeBot service robot to turn scenes on/off through
 * the Home Automation system as part of a HomeBot behavior.
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

#include "homebot/BotAffectHASceneOpr.hpp"

BotAffectHASceneOpr::BotAffectHASceneOpr() {
  // TODO(Mark Jenkins): Auto-generated constructor stub
}

BotAffectHASceneOpr::BotAffectHASceneOpr(const std::string pCode,
                                         const int pSceneNumber,
                                         const int pAction)
    : BotOperation("", pCode) {
  request.sceneNumber = pSceneNumber;
  request.action = pAction;
}

BotAffectHASceneOpr::~BotAffectHASceneOpr() {
  // TODO(Mark Jenkins): Auto-generated destructor stub
}

homebot::HAScene::Request BotAffectHASceneOpr::details() {
  return request;
}

bool BotAffectHASceneOpr::isExecutable(const OperationParameters& opParams) {
  // Validate the code, the scene is less than the maximum, and the action is recognized;
  if ((code == "HAScene") && (request.sceneNumber <= opParams.maxSceneNumber)
      && ((request.action == homebot::HASceneRequest::TURNOFF)
          || (request.action == homebot::HASceneRequest::TURNON)
          || (request.action == homebot::HASceneRequest::STATUS)))
    return true;
  else {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHASceneOpr(isExecutable): Validation failed code '" << code << "', scene '" << static_cast<int>(request.sceneNumber) << "', and action '" << static_cast<int>(request.action) << "'");
    return false;
  }
}

bool BotAffectHASceneOpr::execute(BotOprClients& clients) {
  // Check whether the service is still available
  if (!clients.scHAScene.exists()) {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHASceneOpr(execute): HAScene service does not exist when trying to execute action '" << static_cast<int>(request.action) << "' on scene '" << request.sceneNumber << "'");
    return false;
  }
  // Call for service  using the stored request object and a newly created response object
  homebot::HASceneResponse response;
  clients.scHAScene.call(request, response);

  // See if we got a response for the scene that we requested
  if (response.sceneNumber != request.sceneNumber) {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHASceneOpr(execute): HAScene service response was for scene " << response.sceneNumber << " when scene " << request.sceneNumber << " was requested");
    return false;
  }

  // If we checked status, any status is good
  if (request.action == homebot::HASceneRequest::STATUS) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHASceneOpr(execute): Request for status of scene '" << static_cast<int>(request.sceneNumber) << "' returned state '" << response.state << "'");
    return true;
  }

  // If we asked to turn on the scene, it should be turned on
  if ((request.action == homebot::HASceneRequest::TURNON)
      && (response.state == homebot::HASceneResponse::ON)) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHASceneOpr(execute): Request to turn on scene '" << static_cast<int>(request.sceneNumber) << "' succeeded");
    return true;
  }

  // if we asked to close the scene, it should be closed
  if ((request.action == homebot::HASceneRequest::TURNOFF)
      && (response.state == homebot::HASceneResponse::OFF)) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHASceneOpr(execute): Request to turn off scene '" << static_cast<int>(request.sceneNumber) << "' succeeded");
    return true;
  }

  // We didn't get what we wanted
  ROS_INFO_STREAM(
      "HomeBot-BotAffectHASceneOpr(execute): Request for action '" << static_cast<int>(request.action) << "' on scene number '" << static_cast<int>(request.sceneNumber) << "' returned a state of '" << response.state << "'");
  return false;
}
