/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotAffectHAShade.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 4, 2017 - Creation
 *
 * @brief Operation that commands Home Automation system to lower and raise shades
 *
 * In a HomeBot system, the Home Automation system is responsible for lowering/raising shades.
 * This operation provides a way for a HomeBot service robot to lower/raise shades through
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

#include "homebot/BotAffectHAShadeOpr.hpp"

BotAffectHAShadeOpr::BotAffectHAShadeOpr() {
  // TODO(Mark Jenkins): Auto-generated constructor stub
}

BotAffectHAShadeOpr::BotAffectHAShadeOpr(const std::string pCode,
                                         const int pShadeNumber,
                                         const int pAction)
    : BotOperation(pCode) {
  request.shadeNumber = pShadeNumber;
  request.action = pAction;
}

BotAffectHAShadeOpr::~BotAffectHAShadeOpr() {
  // TODO(Mark Jenkins): Auto-generated destructor stub
}

homebot::HAShade::Request BotAffectHAShadeOpr::details() {
  return request;
}

bool BotAffectHAShadeOpr::isValid(const OperationParameters& opParams) {
  // Validate the code, the shade is less than the maximum, and the action is recognized;
  if ((code == "HAShade") && (request.shadeNumber <= opParams.maxShadeNumber)
      && ((request.action == homebot::HAShadeRequest::RAISE)
          || (request.action == homebot::HAShadeRequest::LOWER)
          || (request.action == homebot::HAShadeRequest::STATUS)))
    return true;
  else
    {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHAShadeOpr(isValid): Validation failed code '" << code << "', shade '" << static_cast<int>(request.shadeNumber) << "', and action '" << static_cast<int>(request.action) << "'");
    return false;
  }
}

bool BotAffectHAShadeOpr::execute(BotOprClients& clients) {
  // Check whether the service is still available
  if (!clients.scHAShade.exists()) {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHAShadeOpr(execute): HAShade service does not exist when trying to execute action '" << static_cast<int>(request.action) << "' on " << static_cast<int>(request.shadeNumber) << "'");
    return false;
  }
  // Call for service  using the stored request object and a newly created response object
  homebot::HAShadeResponse response;
  clients.scHAShade.call(request, response);

  // See if we got a response for the shade that we requested
  if (response.shadeNumber != request.shadeNumber) {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHAShadeOpr(execute): HAShade service response was for shade '" << static_cast<int>(response.shadeNumber) << "' when shade '" << static_cast<int>(request.shadeNumber) << "' was requested");
    return false;
  }

  // If we checked status, any status is good
  if (request.action == homebot::HAShadeRequest::STATUS) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHAShadeOpr(execute): Request for status of shade '" << static_cast<int>(request.shadeNumber) << "' returned state '" << static_cast<int>(response.state) << "'");
    return true;
  }

  // If we asked to turn on the shade, it should be turned on
  if ((request.action == homebot::HAShadeRequest::LOWER)
      && (response.state == homebot::HAShadeResponse::LOWERED)) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHAShadeOpr(execute): Request to lower shade '" << static_cast<int>(request.shadeNumber) << "' succeeded");
    return true;
  }

  // if we asked to close the shade, it should be closed
  if ((request.action == homebot::HAShadeRequest::RAISE)
      && (response.state == homebot::HAShadeResponse::RAISED)) {
    ROS_INFO_STREAM(
        "HomeBot-BotAffectHAShadeOpr(execute): Request to raise shade '" << static_cast<int>(request.shadeNumber) << "' succeeded");
    return true;
  }

  // We didn't get what we wanted
  ROS_INFO_STREAM(
      "HomeBot-BotAffectHAShadeOpr(execute): Request for action '" << static_cast<int>(request.action) << "' on shade number '" << static_cast<int>(request.shadeNumber) << "' returned a state of " << response.state);
  return false;
}
