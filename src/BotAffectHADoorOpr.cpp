/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotAffectHADoorOpr.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 4, 2017 - Creation
 *
 * @brief Operation that commands Home Automation system to open/close doors
 *
 * In a HomeBot system, the Home Automation system is responsible for opening/closing doors.
 * This operation provides a way for a HomeBot service robot to open/close doors through
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

#include "homebot/BotAffectHADoorOpr.hpp"

BotAffectHADoorOpr::BotAffectHADoorOpr() {
  // TODO(Mark Jenkins): Auto-generated constructor stub
}

BotAffectHADoorOpr::BotAffectHADoorOpr(const std::string pCode,
                                       const int pDoorNumber, const int pAction)
    : BotOperation("", pCode) {
  request.doorNumber = pDoorNumber;
  request.action = pAction;
}

BotAffectHADoorOpr::~BotAffectHADoorOpr() {
  // TODO(Mark Jenkins): Auto-generated destructor stub
}

homebot::HADoor::Request BotAffectHADoorOpr::details() {
  return request;
}

bool BotAffectHADoorOpr::isExecutable(const OperationParameters& opParams) {
  ROS_ERROR_STREAM("HomeBot-BotAffectHADoorOpr(isExecutable): Entered");
  // Validate the code, the door is less than the maximum, and the action is recognized;
  if ((code == "HADoor") && (request.doorNumber <= opParams.maxDoorNumber)
      && ((request.action == homebot::HADoorRequest::CLOSE)
          || (request.action == homebot::HADoorRequest::OPEN)
          || (request.action == homebot::HADoorRequest::STATUS))) {
    ROS_ERROR_STREAM(
        "HomeBot-BotAffectHADoorOpr(isExecutable): returning TRUE");
    return true;
  }
  else {
    ROS_ERROR_STREAM(
        "HomeBot-BotAffectHADoorOpr(isExecutable): Validation failed code '" << code << "', door '" << static_cast<int>(request.doorNumber) << "', and action '" << static_cast<int>(request.action) << "'");
  }
    return false;
}

bool BotAffectHADoorOpr::execute(BotOprClients& clients) {
  // Check whether the service is still available
  if (!clients.scHADoor.exists()) {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHADoorOpr(execute): HADoor service does not exist when trying to execute action '" << static_cast<int>(request.action) << "' on door '" << static_cast<int>(request.doorNumber) << "'");
    return false;
  }
  // Call for service  using the stored request object and a newly created response object
  homebot::HADoorResponse response;
  clients.scHADoor.call(request, response);

  // See if we got a response for the door that we requested
  if (response.doorNumber != request.doorNumber) {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHADoorOpr(execute): HADoor service response was for door '" << static_cast<int>(response.doorNumber) << "' when door '" << static_cast<int>(request.doorNumber) << "' was requested");
    return false;
  }

  // If we checked status, any status is good
  if (request.action == homebot::HADoorRequest::STATUS) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHADoorOpr(execute): Request for status of door '" << static_cast<int>(request.doorNumber) << "' returned state '" << static_cast<int>(response.state) << "'");
    return true;
  }

  // If we asked to open the door, it should be opened
  if ((request.action == homebot::HADoorRequest::OPEN)
      && (response.state == homebot::HADoorResponse::OPENED)) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHADoorOpr(execute): Request to open door '" << static_cast<int>(request.doorNumber) << "' succeeded");
    return true;
  }

  // if we asked to close the door, it should be closed
  if ((request.action == homebot::HADoorRequest::CLOSE)
      && (response.state == homebot::HADoorResponse::CLOSED)) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHADoorOpr(execute): Request to close door '" << static_cast<int>(request.doorNumber) << "' succeeded");
    return true;
  }

  // We didn't get what we wanted
  ROS_INFO_STREAM(
      "HomeBot-BotAffectHADoorOpr(execute): Request for action '" << static_cast<int>(request.action) << "' on door number '" << static_cast<int>(request.doorNumber) << "' returned a state of '" << static_cast<int>(response.state) << "'");
  return false;
}
